"""
CANopen tab of the plotter (docs/canopen-pc-handoff.md): the bus, the CiA 402 state and controlword, a panel
per mode (PP / PV / HM, and the set-point streamer for CSP / CSV / CST), then sub-tabs: TPDO plots, the OD
browser, PDO mapping, the frame trace, the EMCY log and the test runner (cotest.py).

Everything that blocks (SDO) runs on co_link.Worker; python-can's notifier thread only fills the rings in
co_link. Timers here read them. Nothing is enabled on connect, and the mode combo only sends when the user
picks a mode. E-STOP (Esc) sends disable voltage on RPDO1 and over SDO, and STOP over USB when connected.
"""
import csv
import sys
import threading
import time

import can
import numpy as np
import pyqtgraph as pg
import serial.tools.list_ports
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

import co_link as C
import cotest

RED, GREEN, ORANGE, BLUE, GREY = "#ff6060", "#60c060", "#f0a040", "#7fb2ff", "#80868b"
C_LOCALE = QtCore.QLocale.c()
INTERFACES = ["slcan", "gs_usb", "pcan", "kvaser", "socketcan", "ixxat", "vector", "virtual"]
BITRATES = [1000000, 500000, 250000, 125000, 50000, 20000]
MODE_LIST = [C.MODE_PP, C.MODE_PV, C.MODE_HM, C.MODE_CSP, C.MODE_CSV, C.MODE_CST]
MODE_LONG = {C.MODE_PP: "PP profile position", C.MODE_PV: "PV profile velocity", C.MODE_HM: "HM homing",
             C.MODE_CSP: "CSP cyclic sync position", C.MODE_CSV: "CSV cyclic sync velocity",
             C.MODE_CST: "CST cyclic sync torque"}
STATE_COLOR = {C.OE: GREEN, C.FAULT: RED, C.FRA: RED, C.QSA: ORANGE}
SW_SHOWN = (4, 5, 7, 9, 10, 11, 12, 13)
DT_NAMES = {1: "BOOL", 2: "INT8", 3: "INT16", 4: "INT32", 5: "UINT8", 6: "UINT16", 7: "UINT32", 8: "REAL32",
            9: "STRING", 0xA: "OCTETS", 0xB: "UNICODE", 0xF: "DOMAIN", 0x15: "INT64", 0x1B: "UINT64"}

# plots: (title, [(signal, label, color)], shown by default). signal: (index, sub) of a TPDO entry, or "sent"
PLOTS = [
    ("Position (turn)", [((0x6064, 0), "actual 6064", "#40d0e0"), ((0x6062, 0), "demand 6062", "#f0c040"),
                         ("sent", "target sent", "#ff5555")], True),
    ("Velocity (rpm)", [((0x606C, 0), "actual 606C", "#40d0e0"), ((0x606B, 0), "demand 606B", "#f0c040"),
                        ("sent", "target sent", "#ff5555")], True),
    ("Torque (‰ rated)", [((0x6077, 0), "actual 6077", "#40d0e0"), ((0x6074, 0), "demand 6074", "#f0c040"),
                          ((0x6078, 0), "current 6078", "#ff9040"), ("sent", "target sent", "#ff5555")], True),
    ("Following error (deg)", [((0x60F4, 0), "60F4", "#ff9040")], False),
    ("DC link (V)", [((0x6079, 0), "6079", "#f0c040")], False),
    ("Statusword bits", [((0x6041, 0), "bits", "#e0e0e0")], False),
]
SENT_PLOT = {C.MODE_CSP: 0, C.MODE_CSV: 1, C.MODE_CST: 2}  # which plot shows the streamed target
SW_TRACE_BITS = [(7, "warning", "#f0a040"), (10, "reached", "#60c060"), (11, "limit", "#ff9040"),
                 (12, "op 12", "#40d0e0"), (13, "op 13", "#ff5555")]

# streamer generators: (name, [(param label, default, decimals)]), units per mode in UNITS
GENS = [
    ("sine", [("amplitude", 0.5, 3), ("frequency (Hz)", 0.5, 3), ("offset", 0.0, 3)]),
    ("step", [("amplitude", 0.25, 3), ("at (s)", 0.2, 3), ("square period (s, 0 = single)", 0.0, 3)]),
    ("ramp", [("slope (/s)", 0.5, 3), ("limit", 1.0, 3)]),
    ("chirp", [("amplitude", 0.2, 3), ("f0 (Hz)", 0.2, 3), ("f1 (Hz)", 3.0, 3), ("duration (s)", 10.0, 2)]),
    ("trapezoid", [("distance", 1.0, 3), ("max speed (/s)", 1.0, 3), ("acceleration (/s²)", 4.0, 3)]),
    ("file", []),
]
UNITS = {C.MODE_CSP: "turn", C.MODE_CSV: "rpm", C.MODE_CST: "‰"}

# PDO presets for the spare TPDO4: (label, [(index, sub)])
TPDO4_PRESETS = [("6062 position demand + 60F4 following error", [(0x6062, 0), (0x60F4, 0)]),
                 ("606B velocity demand + 6074 torque demand", [(0x606B, 0), (0x6074, 0)])]


def dspin(decimals=3, lo=-1e9, hi=1e9, step=1.0, value=0.0, suffix=""):
    w = QtWidgets.QDoubleSpinBox(decimals=decimals, minimum=lo, maximum=hi, singleStep=step, suffix=suffix)
    w.setLocale(C_LOCALE)
    w.setValue(value)
    w.setKeyboardTracking(False)
    return w


def ispin(lo, hi, value, suffix=""):
    w = QtWidgets.QSpinBox(minimum=lo, maximum=hi, suffix=suffix)
    w.setValue(value)
    w.setLocale(C_LOCALE)
    return w


def colored(text, color):
    return f"<span style='color:{color}'>{text}</span>" if color else text


def parse_value(text, dt):
    """typed text -> python value for an OD data type"""
    text = text.strip()
    if dt in (9, 0xB):
        return text
    if dt == 0xA:
        return bytes.fromhex(text)
    if dt == 8:
        return float(text)
    return int(text, 0)


def fmt_value(v, dt=None):
    if v is None:
        return ""
    if isinstance(v, bytes):
        return v.hex(" ")
    if isinstance(v, float):
        return f"{v:.6g}"
    if isinstance(v, int) and dt in (6, 7, 0x1B) and v > 9:
        return f"{v} (0x{v:X})"
    return str(v)


def parse_ids(text):
    """'0x181, 0x280-0x2FF, !0x80' -> (include set of ranges or None, exclude ranges)"""
    inc, exc = [], []
    for part in text.replace(";", ",").split(","):
        part = part.strip()
        if not part:
            continue
        neg = part.startswith("!")
        part = part.lstrip("!").strip()
        lo, _, hi = part.partition("-")
        try:
            r = (int(lo, 0), int(hi, 0) if hi else int(lo, 0))
        except ValueError:
            continue
        (exc if neg else inc).append(r)
    return inc or None, exc


class CanopenTab(QtWidgets.QWidget):
    def __init__(self, main):
        super().__init__()
        self.main = main  # .settings, .link, .config (ConfigTab: stop(), proto_ok)
        self.settings = main.settings
        self.bus = None
        self.node_id = 1
        self._drive = None
        self.worker = C.Worker()
        self.worker.start()
        # the GUI thread holds the GIL for up to the switch interval (5 ms) at a time: with it, the SYNC / set-point
        # thread was measured 20 ms late every cycle. 0.5 ms keeps it on time
        sys.setswitchinterval(0.0005)
        self.streamer = None
        self.mode_sel = None  # mode in the combo (what 6060 was last set to / read as)
        self.testing = False
        self.cancel = threading.Event()
        self.test_lines = []
        self.test_lock = threading.Lock()
        self.trace_total = 0
        self.emcy_total = 0
        self.od_items = {}  # (index, sub) -> QTreeWidgetItem
        self.od_vals = {}
        self.refusal_for = None  # statusword the refusal text was read for
        self.rec = None  # (path, t0, {key: [t list, v list]}, heads)
        self.last_stats = 0.0
        self.last_slow = 0.0
        self.ident = ""

        left = QtWidgets.QWidget()
        lv = QtWidgets.QVBoxLayout(left)
        lv.addWidget(self._build_bus())
        lv.addWidget(self._build_drive())
        lv.addWidget(self._build_modes())
        lv.addWidget(self._build_log(), 1)
        scroll = QtWidgets.QScrollArea(widgetResizable=True)
        scroll.setWidget(left)
        scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setMinimumWidth(left.minimumSizeHint().width() + 24)

        self.sub = QtWidgets.QTabWidget()
        self.sub.addTab(self._build_plots(), "Plots")
        self.sub.addTab(self._build_od(), "Object dictionary")
        self.sub.addTab(self._build_pdo(), "PDO mapping")
        self.sub.addTab(self._build_trace(), "Trace")
        self.sub.addTab(self._build_emcy(), "EMCY")
        self.sub.addTab(self._build_tests(), "Tests")

        split = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        split.addWidget(scroll)
        split.addWidget(self.sub)
        split.setStretchFactor(1, 1)
        split.setSizes([scroll.minimumWidth(), 10000])
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.addWidget(split)

        self._load_settings()
        self.set_connected(False)
        self.timer = QtCore.QTimer(interval=40)
        self.timer.timeout.connect(self.refresh)
        self.timer.start()
        self.poll_timer = QtCore.QTimer(interval=250)
        self.poll_timer.timeout.connect(self.poll)
        self.poll_timer.start()
        sc = QtGui.QShortcut(QtGui.QKeySequence("Esc"), main)
        sc.setContext(QtCore.Qt.ShortcutContext.ApplicationShortcut)
        sc.activated.connect(self.estop)

    # ------------------------------------------------------------------ layout: left column
    def _build_bus(self):
        g = QtWidgets.QGroupBox("CAN bus")
        f = QtWidgets.QFormLayout(g)
        self.iface = QtWidgets.QComboBox(editable=True)
        self.iface.addItems(INTERFACES)
        self.chan = QtWidgets.QComboBox(editable=True)
        self.chan.setMinimumWidth(120)
        rescan = QtWidgets.QToolButton(text="⟳")
        rescan.setToolTip("Rescan serial ports")
        rescan.clicked.connect(self.scan_ports)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.iface)
        row.addWidget(self.chan, 1)
        row.addWidget(rescan)
        f.addRow("Adapter", row)
        self.bitrate = QtWidgets.QComboBox()
        for b in BITRATES:
            self.bitrate.addItem(f"{b // 1000} kbit/s", b)
        self.node_sp = ispin(1, 127, 1)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.bitrate)
        row.addWidget(QtWidgets.QLabel("node"))
        row.addWidget(self.node_sp)
        f.addRow("Bit rate", row)
        self.conn_btn = QtWidgets.QPushButton("Connect")
        self.conn_btn.clicked.connect(self.toggle_connection)
        f.addRow(self.conn_btn)
        self.bus_lbl = QtWidgets.QLabel("not connected")
        self.bus_lbl.setWordWrap(True)
        f.addRow("Drive", self.bus_lbl)
        self.stats_lbl = QtWidgets.QLabel("-")
        self.stats_lbl.setWordWrap(True)
        f.addRow("Bus", self.stats_lbl)

        nmt = QtWidgets.QHBoxLayout()
        self.nmt_btns = []
        for label, cmd in (("Start", "OPERATIONAL"), ("Pre-op", "PRE-OPERATIONAL"), ("Stop", "STOPPED"),
                           ("Reset", "RESET"), ("Reset comm", "RESET COMMUNICATION")):
            b = QtWidgets.QPushButton(label)
            b.setToolTip(f"NMT {cmd.lower()} (node)")
            b.clicked.connect(lambda _=False, c=cmd: self.nmt(c))
            nmt.addWidget(b)
            self.nmt_btns.append(b)
        f.addRow("NMT", nmt)

        self.hb_on = QtWidgets.QCheckBox("PC heartbeat")
        self.hb_on.setToolTip("The PC produces a heartbeat (state operational) at this node ID and period")
        self.hb_on.toggled.connect(self.apply_heartbeat)
        self.hb_id = ispin(1, 127, C.PC_HB_ID)
        self.hb_ms = ispin(10, 5000, 100, " ms")
        self.hb_pause = QtWidgets.QCheckBox("pause")
        self.hb_pause.setToolTip("Stop sending it: simulates a lost master (the drive faults 0x8130 if it watches it)")
        self.hb_pause.toggled.connect(lambda on: setattr(self.bus.hb, "paused", on) if self.bus and self.bus.hb else None)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.hb_on)
        row.addWidget(self.hb_id)
        row.addWidget(self.hb_ms)
        row.addWidget(self.hb_pause)
        f.addRow(row)
        self.watch_ms = ispin(0, 10000, 300, " ms")
        self.watch_ms.setToolTip("0x1016:1 consumer time (0 = not watched)")
        watch = QtWidgets.QPushButton("Watch PC heartbeat")
        watch.setToolTip("Write 0x1016:1 = PC node ID << 16 | time: losing the PC then faults the drive (0x6007)")
        watch.clicked.connect(self.watch_heartbeat)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(watch)
        row.addWidget(self.watch_ms)
        f.addRow(row)
        self.sync_on = QtWidgets.QCheckBox("SYNC producer")
        self.sync_on.setToolTip("TPDO2/TPDO3 are sent on SYNC. Software timed (busy-wait): 5-10 ms is realistic over SLCAN")
        self.sync_on.toggled.connect(self.apply_sync)
        self.sync_ms = dspin(1, 1, 1000, 1, 5, " ms")
        self.sync_ms.valueChanged.connect(lambda _: self.apply_sync(self.sync_on.isChecked()))
        self.jit_lbl = QtWidgets.QLabel("")
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.sync_on)
        row.addWidget(self.sync_ms)
        row.addWidget(self.jit_lbl, 1)
        f.addRow(row)
        self.bus_widgets = [*self.nmt_btns, watch]
        return g

    def _build_drive(self):
        g = QtWidgets.QGroupBox("CiA 402 drive")
        v = QtWidgets.QVBoxLayout(g)
        self.state_lbl = QtWidgets.QLabel("-")
        f = self.state_lbl.font()
        f.setPointSize(f.pointSize() + 5)
        f.setBold(True)
        self.state_lbl.setFont(f)
        self.sw_lbl = QtWidgets.QLabel("")
        self.sw_lbl.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.state_lbl, 1)
        row.addWidget(self.sw_lbl)
        v.addLayout(row)
        grid = QtWidgets.QGridLayout()
        self.bit_lbls = {}
        for i, b in enumerate(SW_SHOWN):
            lb = QtWidgets.QLabel(f"{b}")
            self.bit_lbls[b] = lb
            grid.addWidget(lb, i // 2, i % 2)
        v.addLayout(grid)
        self.fault_lbl = QtWidgets.QLabel("")
        self.fault_lbl.setWordWrap(True)
        v.addWidget(self.fault_lbl)

        grid = QtWidgets.QGridLayout()
        self.cw_btns = []
        cmds = [("Shutdown 06", "shutdown"), ("Switch on 07", "switch_on"), ("Enable 0F", "enable"),
                ("Disable op 07", "disable_op"), ("Quick stop 02", "quick_stop"), ("Disable voltage 00", "disable_voltage"),
                ("Fault reset", "fault_reset")]
        for i, (label, fn) in enumerate(cmds):
            b = QtWidgets.QPushButton(label)
            b.clicked.connect(lambda _=False, fn=fn, label=label: self.cw_cmd(fn, label))
            grid.addWidget(b, i // 3, i % 3)
            self.cw_btns.append(b)
        self.halt_btn = QtWidgets.QPushButton("Halt", checkable=True)
        self.halt_btn.setToolTip("Controlword bit 8 (kept in every following command until released)")
        self.halt_btn.toggled.connect(self.halt)
        grid.addWidget(self.halt_btn, 2, 1)
        self.cw_btns.append(self.halt_btn)
        v.addLayout(grid)

        row = QtWidgets.QHBoxLayout()
        self.mode_cb = QtWidgets.QComboBox()
        for m in MODE_LIST:
            self.mode_cb.addItem(MODE_LONG[m], m)
        self.mode_cb.setToolTip("0x6060: sent only when you pick a mode here")
        self.mode_cb.activated.connect(self.mode_picked)  # user action only: never sends on load
        self.mode_disp = QtWidgets.QLabel("6061 -")
        self.enable_btn = QtWidgets.QPushButton("Enable in mode")
        self.enable_btn.setToolTip("6060 = this mode, then 0x06 -> 0x07 -> 0x0F, checking the state after each step")
        self.enable_btn.clicked.connect(self.goto_enabled)
        row.addWidget(self.mode_cb, 1)
        row.addWidget(self.mode_disp)
        row.addWidget(self.enable_btn)
        v.addLayout(row)
        self.cw_btns += [self.mode_cb, self.enable_btn]

        self.estop_btn = QtWidgets.QPushButton("E-STOP  (Esc)")
        self.estop_btn.setObjectName("stop")
        self.estop_btn.setMinimumHeight(40)
        self.estop_btn.setToolTip("Disable voltage on RPDO1 and over SDO, and STOP over USB when connected")
        self.estop_btn.clicked.connect(self.estop)
        v.addWidget(self.estop_btn)
        return g

    def _build_modes(self):
        g = QtWidgets.QGroupBox("Mode")
        v = QtWidgets.QVBoxLayout(g)
        self.mode_stack = QtWidgets.QStackedWidget()
        self.mode_pages = {}

        # PP
        w = QtWidgets.QWidget()
        f = QtWidgets.QFormLayout(w)
        self.pp_target = dspin(4, -1000, 1000, 0.25, 1.0, " turn")
        self.pp_rel = QtWidgets.QComboBox()
        self.pp_rel.addItems(["absolute", "relative (to the last target)"])
        self.pp_imm = QtWidgets.QCheckBox("change immediately (bit 5)")
        self.pp_imm.setChecked(True)
        f.addRow("Target", self.pp_target)
        f.addRow("", self.pp_rel)
        f.addRow("", self.pp_imm)
        self.pp_vel = dspin(1, 0, 100000, 10, 60, " rpm")
        self.pp_acc = dspin(0, 0, 1e6, 60, 240, " rpm/s")
        self.pp_dec = dspin(0, 0, 1e6, 60, 240, " rpm/s")
        f.addRow("Profile velocity 6081", self.pp_vel)
        f.addRow("Acceleration 6083", self.pp_acc)
        f.addRow("Deceleration 6084", self.pp_dec)
        row = QtWidgets.QHBoxLayout()
        b = QtWidgets.QPushButton("Write profile")
        b.clicked.connect(self.pp_profile)
        go = QtWidgets.QPushButton("Go (bit 4)")
        go.clicked.connect(self.pp_go)
        row.addWidget(b)
        row.addWidget(go)
        f.addRow(row)
        self.pp_lbl = QtWidgets.QLabel("")
        self.pp_lbl.setWordWrap(True)
        f.addRow(self.pp_lbl)
        self._add_page(C.MODE_PP, w, [b, go])

        # PV
        w = QtWidgets.QWidget()
        f = QtWidgets.QFormLayout(w)
        self.pv_target = dspin(1, -100000, 100000, 10, 60, " rpm")
        self.pv_acc = dspin(0, 0, 1e6, 60, 240, " rpm/s")
        self.pv_dec = dspin(0, 0, 1e6, 60, 240, " rpm/s")
        f.addRow("Target 60FF", self.pv_target)
        f.addRow("Acceleration 6083", self.pv_acc)
        f.addRow("Deceleration 6084", self.pv_dec)
        row = QtWidgets.QHBoxLayout()
        b1 = QtWidgets.QPushButton("Set")
        b1.clicked.connect(self.pv_set)
        b2 = QtWidgets.QPushButton("Zero")
        b2.clicked.connect(lambda: self.job(lambda: self.node.write(0x60FF, 0, 0), what="60FF = 0"))
        row.addWidget(b1)
        row.addWidget(b2)
        f.addRow(row)
        self.pv_target.findChild(QtWidgets.QLineEdit).returnPressed.connect(self.pv_set)
        self._add_page(C.MODE_PV, w, [b1, b2])

        # HM
        w = QtWidgets.QWidget()
        f = QtWidgets.QFormLayout(w)
        self.hm_method = QtWidgets.QComboBox()
        self.hm_method.addItem("37 home here", 37)
        self.hm_method.addItem("35 home here (old number)", 35)
        self.hm_offset = dspin(4, -1000, 1000, 0.25, 0.0, " turn")
        self.hm_offset.setToolTip("607C: the position at home becomes -607C")
        f.addRow("Method 6098", self.hm_method)
        f.addRow("Home offset 607C", self.hm_offset)
        b = QtWidgets.QPushButton("Start homing")
        b.setToolTip("6098, 607C, enable in HM, bit 4 rising")
        b.clicked.connect(self.hm_start)
        f.addRow(b)
        self.hm_lbl = QtWidgets.QLabel("")
        f.addRow(self.hm_lbl)
        self._add_page(C.MODE_HM, w, [b])

        # CSP / CSV / CST: the streamer
        w = QtWidgets.QWidget()
        f = QtWidgets.QFormLayout(w)
        self.st_title = QtWidgets.QLabel("")
        f.addRow(self.st_title)
        self.gen_cb = QtWidgets.QComboBox()
        for name, _ in GENS:
            self.gen_cb.addItem(name)
        self.gen_cb.currentIndexChanged.connect(self.gen_changed)
        f.addRow("Generator", self.gen_cb)
        self.gen_lbls, self.gen_sp = [], []
        for i in range(4):
            lb = QtWidgets.QLabel("")
            sp = dspin(4, -1e6, 1e6, 0.1, 0.0)
            self.gen_lbls.append(lb)
            self.gen_sp.append(sp)
            f.addRow(lb, sp)
        self.gen_file = QtWidgets.QLineEdit()
        self.gen_file.setPlaceholderText("CSV of t,value rows")
        pick = QtWidgets.QToolButton(text="…")
        pick.clicked.connect(self.pick_gen_file)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.gen_file, 1)
        row.addWidget(pick)
        self.gen_file_row = QtWidgets.QWidget()
        self.gen_file_row.setLayout(row)
        f.addRow("File", self.gen_file_row)
        self.st_period = dspin(1, 1, 100, 1, 5, " ms")
        self.st_period.setToolTip("RPDO + SYNC period; CSP sets 0x60C2 to it. Below ~5 ms SLCAN on Windows jitters")
        self.st_dur = dspin(1, 0, 3600, 1, 0, " s")
        self.st_dur.setSpecialValueText("until Stop")
        f.addRow("Period", self.st_period)
        f.addRow("Duration", self.st_dur)
        self.st_vmax = dspin(0, 0, 100000, 10, 300, " rpm")
        self.st_vmax.setSpecialValueText("off")
        self.st_pmax = dspin(2, 0, 1000, 0.5, 2.0, " turn")
        self.st_pmax.setSpecialValueText("off")
        f.addRow("Guard |speed|", self.st_vmax)
        f.addRow("Guard |position|", self.st_pmax)
        self.st_enable = QtWidgets.QCheckBox("NMT start and enable in the mode first")
        self.st_enable.setChecked(True)
        f.addRow(self.st_enable)
        row = QtWidgets.QHBoxLayout()
        self.st_start = QtWidgets.QPushButton("Start stream")
        self.st_start.clicked.connect(self.stream_start)
        self.st_stop = QtWidgets.QPushButton("Stop stream")
        self.st_stop.clicked.connect(self.stream_stop)
        row.addWidget(self.st_start)
        row.addWidget(self.st_stop)
        f.addRow(row)
        self.st_lbl = QtWidgets.QLabel("")
        self.st_lbl.setWordWrap(True)
        f.addRow(self.st_lbl)
        self.jit_plot = pg.PlotWidget()
        self.jit_plot.setMaximumHeight(110)
        self.jit_plot.setTitle("PC SYNC period (ms)", size="8pt")
        self.jit_plot.setMouseEnabled(False, False)
        self.jit_curve = self.jit_plot.plot(stepMode="center", fillLevel=0, brush="#3d6fd9")
        f.addRow(self.jit_plot)
        self.stream_page = w
        for m in (C.MODE_CSP, C.MODE_CSV, C.MODE_CST):
            self.mode_pages[m] = w
        self.mode_stack.addWidget(w)
        self.mode_widgets = getattr(self, "mode_widgets", []) + [self.st_start]
        v.addWidget(self.mode_stack)
        self.gen_changed(0)
        return g

    def _add_page(self, mode, w, widgets):
        self.mode_pages[mode] = w
        self.mode_stack.addWidget(w)
        self.mode_widgets = getattr(self, "mode_widgets", []) + widgets

    def _build_log(self):
        g = QtWidgets.QGroupBox("Log")
        v = QtWidgets.QVBoxLayout(g)
        self.log_box = QtWidgets.QPlainTextEdit(readOnly=True)
        self.log_box.setMaximumBlockCount(3000)
        self.log_box.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont))
        self.log_box.setMinimumHeight(120)
        v.addWidget(self.log_box)
        return g

    # ------------------------------------------------------------------ layout: sub-tabs
    def _build_plots(self):
        w = QtWidgets.QWidget()
        h = QtWidgets.QHBoxLayout(w)
        side = QtWidgets.QVBoxLayout()
        self.plot_checks = []
        for title, _, default in PLOTS:
            cb = QtWidgets.QCheckBox(title)
            cb.setChecked(default)
            cb.toggled.connect(self.build_plots)
            side.addWidget(cb)
            self.plot_checks.append(cb)
        self.win_s = dspin(1, 0.5, 120, 1, 10, " s")
        self.win_s.valueChanged.connect(self.build_plots)
        side.addWidget(QtWidgets.QLabel("Window"))
        side.addWidget(self.win_s)
        self.plot_pause = QtWidgets.QPushButton("Pause", checkable=True)
        side.addWidget(self.plot_pause)
        self.rec_btn = QtWidgets.QPushButton("Record…", checkable=True)
        self.rec_btn.setToolTip("Every TPDO sample and every streamed set-point from now, to .npz or .csv")
        self.rec_btn.toggled.connect(self.toggle_record)
        side.addWidget(self.rec_btn)
        auto = QtWidgets.QPushButton("Auto-scale")
        auto.clicked.connect(self.reset_view)
        side.addWidget(auto)
        note = QtWidgets.QLabel("Data: TPDO2/3 (on SYNC: switch on the SYNC producer and NMT start). "
                                "Demand values and 60F4 need TPDO4: see PDO mapping.")
        note.setWordWrap(True)
        note.setMaximumWidth(170)
        side.addWidget(note)
        side.addStretch(1)
        h.addLayout(side)
        self.glw = pg.GraphicsLayoutWidget()
        self.glw.scene().sigMouseClicked.connect(lambda ev: ev.double() and self.reset_view())
        h.addWidget(self.glw, 1)
        self.curves = []
        self.plot_items = []
        return w

    def _build_od(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        row = QtWidgets.QHBoxLayout()
        self.od_filter = QtWidgets.QLineEdit()
        self.od_filter.setPlaceholderText("filter: index or name")
        self.od_filter.textChanged.connect(self.od_apply_filter)
        rd = QtWidgets.QPushButton("Read selected")
        rd.clicked.connect(self.od_read_selected)
        ra = QtWidgets.QPushButton("Read all")
        ra.clicked.connect(self.od_read_all)
        self.store_cb = QtWidgets.QComboBox()
        self.store_cb.addItem("1 all", 1)
        self.store_cb.addItem("2 communication", 2)
        self.store_cb.addItem("3 application (+ motor params)", 3)
        st = QtWidgets.QPushButton("Store 0x1010")
        st.clicked.connect(self.od_store)
        rs = QtWidgets.QPushButton("Restore 0x1011")
        rs.clicked.connect(self.od_restore)
        for x in (self.od_filter, rd, ra, self.store_cb, st, rs):
            row.addWidget(x)
        row.setStretch(0, 1)
        v.addLayout(row)
        t = self.od_tree = QtWidgets.QTreeWidget()
        t.setHeaderLabels(["Index", "Name", "Type", "Access", "Value", "Default", "Description"])
        t.setAlternatingRowColors(True)
        t.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        t.itemDoubleClicked.connect(self.od_begin_edit)
        t.itemChanged.connect(self.od_item_changed)
        v.addWidget(t, 1)
        self.od_info = QtWidgets.QLabel("Double-click a value to write it. Bold: a writable value that differs from the EDS default.")
        v.addWidget(self.od_info)
        self.od_widgets = [rd, ra, st, rs]
        self._od_updating = False
        return w

    def _build_pdo(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        t = self.pdo_table = QtWidgets.QTableWidget(8, 6)
        t.setHorizontalHeaderLabels(["COB-ID", "Enabled", "Transmission", "Inhibit (100 µs)", "Event (ms)", "Mapping"])
        t.setVerticalHeaderLabels([f"TPDO{i}" for i in range(1, 5)] + [f"RPDO{i}" for i in range(1, 5)])
        t.horizontalHeader().setStretchLastSection(True)
        t.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        t.itemSelectionChanged.connect(self.pdo_selected)
        v.addWidget(t, 1)
        g = QtWidgets.QGroupBox("Edit (pre-operational only; 'Store comm' keeps it over a reset)")
        f = QtWidgets.QFormLayout(g)
        self.pdo_sel = QtWidgets.QComboBox()
        for i in range(1, 5):
            self.pdo_sel.addItem(f"TPDO{i}", ("tpdo", i))
        for i in range(1, 5):
            self.pdo_sel.addItem(f"RPDO{i}", ("rpdo", i))
        self.pdo_map = QtWidgets.QLineEdit()
        self.pdo_map.setPlaceholderText("6064:0 606C:0   (empty = disabled)")
        self.pdo_tt = ispin(0, 255, 1)
        self.pdo_tt.setToolTip("1-240: every n-th SYNC; 254/255: event driven")
        self.pdo_ev = ispin(0, 65535, 0, " ms")
        f.addRow("PDO", self.pdo_sel)
        f.addRow("Mapping", self.pdo_map)
        f.addRow("Transmission type", self.pdo_tt)
        f.addRow("Event timer", self.pdo_ev)
        row = QtWidgets.QHBoxLayout()
        rd = QtWidgets.QPushButton("Read all")
        rd.clicked.connect(self.pdo_refresh)
        wr = QtWidgets.QPushButton("Write")
        wr.clicked.connect(self.pdo_write)
        stc = QtWidgets.QPushButton("Store comm 0x1010:2")
        stc.clicked.connect(lambda: self.store(2))
        for x in (rd, wr, stc):
            row.addWidget(x)
        f.addRow(row)
        row = QtWidgets.QHBoxLayout()
        self.preset_cb = QtWidgets.QComboBox()
        for label, _ in TPDO4_PRESETS:
            self.preset_cb.addItem(label)
        pm = QtWidgets.QPushButton("Map on TPDO4 (every SYNC)")
        pm.clicked.connect(self.pdo_preset)
        rv = QtWidgets.QPushButton("Revert TPDO4")
        rv.setToolTip("TPDO4 disabled and empty, as exported")
        rv.clicked.connect(lambda: self.pdo_apply("tpdo", 4, [], 254, 0))
        row.addWidget(self.preset_cb, 1)
        row.addWidget(pm)
        row.addWidget(rv)
        f.addRow("For the plots", row)
        v.addWidget(g)
        self.pdo_widgets = [rd, wr, stc, pm, rv]
        return w

    def _build_trace(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        row = QtWidgets.QHBoxLayout()
        self.tr_filter = QtWidgets.QLineEdit()
        self.tr_filter.setPlaceholderText("IDs: 0x181, 0x580-0x5FF, !0x80")
        self.tr_hide = QtWidgets.QCheckBox("hide SYNC, TPDO2/3, PC heartbeat")
        self.tr_hide.setChecked(True)
        self.tr_pause = QtWidgets.QPushButton("Pause", checkable=True)
        clr = QtWidgets.QPushButton("Clear")
        clr.clicked.connect(self.trace_clear)
        sv = QtWidgets.QPushButton("Save…")
        sv.setToolTip("All frames in the buffer (50000): .csv, or .asc / .blf / .log through python-can")
        sv.clicked.connect(self.trace_save)
        for x in (self.tr_filter, self.tr_hide, self.tr_pause, clr, sv):
            row.addWidget(x)
        row.setStretch(0, 1)
        v.addLayout(row)
        t = self.tr_table = QtWidgets.QTableWidget(0, 7)
        t.setHorizontalHeaderLabels(["t (s)", "dir", "COB-ID", "kind", "len", "data", "decoded"])
        t.horizontalHeader().setStretchLastSection(True)
        t.verticalHeader().setVisible(False)
        t.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        t.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont))
        t.verticalHeader().setDefaultSectionSize(18)
        for c, wd in enumerate((90, 30, 60, 80, 30, 190)):
            t.setColumnWidth(c, wd)
        v.addWidget(t, 1)
        self.tr_t0 = None
        return w

    def _build_emcy(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        t = self.em_table = QtWidgets.QTableWidget(0, 7)
        t.setHorizontalHeaderLabels(["t (s)", "code", "meaning", "error register", "CANopenNode bit", "info", "note"])
        t.horizontalHeader().setStretchLastSection(True)
        t.verticalHeader().setVisible(False)
        t.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        v.addWidget(t, 1)
        row = QtWidgets.QHBoxLayout()
        lb = QtWidgets.QLabel("EMCYs in the first 0.5 s after connect are tagged: the drive was error passive while "
                              "nothing acknowledged its frames. Don't chase them.")
        lb.setWordWrap(True)
        clr = QtWidgets.QPushButton("Clear")
        clr.clicked.connect(lambda: self.em_table.setRowCount(0))
        row.addWidget(lb, 1)
        row.addWidget(clr)
        v.addLayout(row)
        return w

    def _build_tests(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        t = self.test_tree = QtWidgets.QTreeWidget()
        t.setHeaderLabels(["Test", "Needs", "What", "Result"])
        t.setRootIsDecorated(False)
        self.test_items = {}
        for test in cotest.discover():
            needs = " ".join(x for x, on in (("USB", test.usb), ("motion", test.motion)) if on)
            it = QtWidgets.QTreeWidgetItem([test.name, needs, test.doc, ""])
            it.setFlags(it.flags() | QtCore.Qt.ItemFlag.ItemIsUserCheckable)
            it.setCheckState(0, QtCore.Qt.CheckState.Unchecked)
            t.addTopLevelItem(it)
            self.test_items[test.name] = (test, it)
        for c in range(3):
            t.resizeColumnToContents(c)
        v.addWidget(t, 2)
        row = QtWidgets.QHBoxLayout()
        self.t_motion = QtWidgets.QCheckBox("allow motion (shaft free)")
        rs = QtWidgets.QPushButton("Run checked")
        rs.clicked.connect(lambda: self.run_tests(False))
        ra = QtWidgets.QPushButton("Run all")
        ra.clicked.connect(lambda: self.run_tests(True))
        self.t_stop = QtWidgets.QPushButton("Stop")
        self.t_stop.clicked.connect(self.cancel.set)
        self.t_result = QtWidgets.QLabel("")
        self.t_result.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        for x in (self.t_motion, rs, ra, self.t_stop):
            row.addWidget(x)
        row.addWidget(self.t_result, 1)
        v.addLayout(row)
        self.t_log = QtWidgets.QPlainTextEdit(readOnly=True)
        self.t_log.setMaximumBlockCount(20000)
        self.t_log.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont))
        v.addWidget(self.t_log, 3)
        lb = QtWidgets.QLabel("The same tests run headless: python software/cotest.py --can slcan:COM8 --usb COM6. "
                              "With the USB link connected they also check the loop time and encoder 'late'.")
        lb.setWordWrap(True)
        v.addWidget(lb)
        self.test_widgets = [rs, ra]
        return w

    # ------------------------------------------------------------------ settings
    def _load_settings(self):
        s = self.settings
        self.iface.setCurrentText(str(s.value("co/iface", "slcan")))
        self.scan_ports(prefer=s.value("co/chan", "COM8"))
        i = self.bitrate.findData(int(s.value("co/bitrate", 1000000)))
        self.bitrate.setCurrentIndex(max(0, i))
        self.node_sp.setValue(int(s.value("co/node", 1)))
        self.hb_id.setValue(int(s.value("co/hb_id", C.PC_HB_ID)))
        self.hb_ms.setValue(int(s.value("co/hb_ms", 100)))
        self.hb_on.blockSignals(True)
        self.hb_on.setChecked(s.value("co/hb_on", "true") in (True, "true"))
        self.hb_on.blockSignals(False)
        self.watch_ms.setValue(int(s.value("co/watch_ms", 300)))
        self.sync_ms.setValue(float(s.value("co/sync_ms", 5.0)))
        self.win_s.setValue(float(s.value("co/window", 10.0)))
        shown = s.value("co/plots", None)
        if isinstance(shown, list):
            for cb in self.plot_checks:
                cb.blockSignals(True)
                cb.setChecked(cb.text() in shown)
                cb.blockSignals(False)
        self.build_plots()

    def save_settings(self):
        s = self.settings
        s.setValue("co/iface", self.iface.currentText())
        s.setValue("co/chan", self.chan.currentText().split()[0] if self.chan.currentText() else "")
        s.setValue("co/bitrate", self.bitrate.currentData())
        s.setValue("co/node", self.node_sp.value())
        s.setValue("co/hb_id", self.hb_id.value())
        s.setValue("co/hb_ms", self.hb_ms.value())
        s.setValue("co/hb_on", self.hb_on.isChecked())
        s.setValue("co/watch_ms", self.watch_ms.value())
        s.setValue("co/sync_ms", self.sync_ms.value())
        s.setValue("co/window", self.win_s.value())
        s.setValue("co/plots", [cb.text() for cb in self.plot_checks if cb.isChecked()])

    def scan_ports(self, prefer=None):
        cur = prefer or self.chan.currentText().split(" ")[0]
        self.chan.clear()
        for p in sorted(serial.tools.list_ports.comports(), key=lambda p: p.device):
            self.chan.addItem(p.device)
        if cur:
            if self.chan.findText(cur) < 0:
                self.chan.addItem(cur)
            self.chan.setCurrentText(cur)

    # ------------------------------------------------------------------ helpers
    @property
    def node(self):
        return self.bus.drives.get(self.node_id) if self.bus else None

    @property
    def drive(self):
        n = self.node
        if n is None:
            return None
        if self._drive is None or self._drive.node is not n:
            self._drive = C.Drive402(n)
            self._drive.halted = self.halt_btn.isChecked()
        return self._drive

    def log(self, text, color=None):
        stamp = time.strftime("%H:%M:%S ")
        self.log_box.appendHtml(colored(stamp + text.replace("<", "&lt;"), color))

    def job(self, fn, done=None, what=None, urgent=False):
        """run fn on the worker; errors go to the log"""
        def cb(res, err):
            if err is not None:
                self.log(f"{what or 'request'}: {err}", RED)
            elif done:
                done(res)
        self.worker.submit(fn, cb, urgent)

    def set_connected(self, on):
        self.conn_btn.setText("Disconnect" if on else "Connect")
        for w in (self.iface, self.chan, self.bitrate, self.node_sp):
            w.setEnabled(not on)
        self.set_controls(on and not self.testing)
        if not on:
            self.bus_lbl.setText("not connected")
            self.stats_lbl.setText("-")
            self.state_lbl.setText("-")

    def set_controls(self, on):
        for w in (*self.bus_widgets, *self.cw_btns, *self.mode_widgets, *self.od_widgets, *self.pdo_widgets,
                  *self.test_widgets):
            w.setEnabled(on)
        self.st_stop.setEnabled(bool(self.bus))
        self.estop_btn.setEnabled(True)

    # ------------------------------------------------------------------ connection
    def toggle_connection(self):
        if self.bus:
            self.disconnect()
            return
        iface = self.iface.currentText().strip()
        chan = self.chan.currentText().split(" ")[0].strip()
        rate = self.bitrate.currentData()
        try:
            self.bus = C.CoBus().connect(iface, chan, rate)
        except Exception as e:  # noqa: BLE001 (python-can raises many kinds)
            self.log(f"cannot open {iface}:{chan}: {e}", RED)
            self.bus = None
            return
        self.node_id = self.node_sp.value()
        self.bus.add_drive(self.node_id)
        self._drive = None
        self.trace_total = self.emcy_total = 0
        self.tr_t0 = None
        self.save_settings()
        self.set_connected(True)
        self.log(f"connected: {iface}:{chan} @ {rate // 1000} kbit/s, node {self.node_id}", GREEN)
        if self.hb_on.isChecked():
            self.apply_heartbeat(True)
        if self.sync_on.isChecked():
            self.apply_sync(True)
        self.build_od_tree()
        self.after_connect()

    def after_connect(self):
        """units, PDO maps, 6060, identity: read-only, never enables"""
        node = self.node

        def work():
            node.refresh_units()
            maps = node.pdo_read()
            mode = node.read(0x6060)
            ident = (node.read(0x1018, 2), node.read(0x1018, 3), node.read(0x1008))
            return maps, mode, ident

        def done(r):
            maps, mode, (prod, rev, name) = r
            self.set_mode_combo(mode)
            self.ident = f"{name}, product {prod}, rev {rev >> 16}.{rev & 0xFFFF}"
            self.pdo_show()
            self.log(f"drive: {self.ident}; rated current {node.units.rated_ma} mA, {node.units.inc_rev} inc/rev", GREEN)
        self.job(work, done, "connect")

    def disconnect(self):
        """disable voltage first, then the heartbeat stops (the drive must not stay enabled unwatched)"""
        if self.rec_btn.isChecked():
            self.rec_btn.setChecked(False)
        if self.testing:
            self.cancel.set()
        if self.streamer:
            self.streamer.stop()
            self.streamer = None
        self.worker.clear()
        node = self.node
        if node:
            try:
                node.write(0x6040, 0, 0, retry=False)
            except Exception as e:  # noqa: BLE001
                self.log(f"disable voltage on disconnect: {e}", ORANGE)
        if self.bus:
            self.bus.disconnect()
        self.bus = None
        self._drive = None
        self.set_connected(False)
        self.log("disconnected")

    def shutdown(self):
        """window closing"""
        self.save_settings()
        if self.bus:
            self.disconnect()
        self.worker.stop()

    # ------------------------------------------------------------------ bus box
    def nmt(self, cmd):
        node = self.node
        self.job(lambda: node.nmt(cmd), lambda _: self.log(f"NMT {cmd.lower()}"), f"NMT {cmd}")
        if cmd in ("OPERATIONAL", "RESET", "RESET COMMUNICATION"):
            QtCore.QTimer.singleShot(1500 if cmd != "OPERATIONAL" else 100, self.after_nmt)

    def after_nmt(self):
        if self.node:
            self.job(self.node.pdo_read, lambda _: self.pdo_show(), "PDO mapping")

    def apply_heartbeat(self, on):
        if not self.bus:
            return
        if on:
            hb = self.bus.start_heartbeat(self.hb_id.value(), self.hb_ms.value() / 1000.0)
            hb.paused = self.hb_pause.isChecked()
        else:
            self.bus.stop_heartbeat()

    def watch_heartbeat(self):
        ms = self.watch_ms.value()
        val = (self.hb_id.value() << 16) | ms if ms else 0
        node = self.node
        self.job(lambda: node.write(0x1016, 1, val),
                 lambda _: self.log(f"0x1016:1 = 0x{val:08X}: " + (f"the drive watches the PC heartbeat ({ms} ms)"
                                                                   if ms else "not watched"), GREEN), "0x1016:1")
        if ms and not self.hb_on.isChecked():
            self.log("the PC heartbeat is off: the drive will fault as soon as it is enabled", ORANGE)

    def apply_sync(self, on):
        if not self.bus:
            return
        if on:
            self.bus.start_sync(self.sync_ms.value() / 1000.0)
        elif not self.streamer:
            self.bus.stop_sync()

    # ------------------------------------------------------------------ controlword
    def cw_cmd(self, fn, label):
        d = self.drive
        if d is None:
            return
        self.job(lambda: getattr(d, fn)(), lambda cw: self.log(f"{label}: 6040 = 0x{cw:04X}"), label)

    def halt(self, on):
        d = self.drive
        if d is None:
            return
        self.job(lambda: d.halt(on), lambda cw: self.log(f"halt {'on' if on else 'off'}: 6040 = 0x{cw:04X}"), "halt")

    def set_mode_combo(self, mode):
        i = self.mode_cb.findData(mode)
        if i >= 0:
            self.mode_cb.blockSignals(True)
            self.mode_cb.setCurrentIndex(i)
            self.mode_cb.blockSignals(False)
        self.mode_sel = mode
        self.show_mode_page(mode)

    def mode_picked(self, i):
        mode = self.mode_cb.itemData(i)
        node = self.node
        self.show_mode_page(mode)
        self.mode_sel = mode
        self.job(lambda: node.write(0x6060, 0, mode), lambda _: self.log(f"6060 = {mode} ({C.MODES.get(mode)})"), "6060")

    def show_mode_page(self, mode):
        w = self.mode_pages.get(mode)
        if w is not None:
            self.mode_stack.setCurrentWidget(w)
        if mode in UNITS:
            self.st_title.setText(f"<b>{MODE_LONG[mode]}</b>: set-points in {UNITS[mode]}"
                                  + (" from the position at start" if mode == C.MODE_CSP else ""))
            self.gen_changed(self.gen_cb.currentIndex())

    def goto_enabled(self):
        d, mode = self.drive, self.mode_cb.currentData()
        if d is None:
            return
        self.mode_sel = mode
        self.job(lambda: d.goto_enabled(mode), lambda sw: self.log(f"operation enabled in {C.MODES[mode]} "
                                                                   f"(sw 0x{sw:04X})", GREEN), "enable")

    def estop(self):
        """always: RPDO1 (if operational), SDO, USB STOP"""
        if self.testing:
            self.cancel.set()
        if self.streamer:
            self.streamer.stop()
        node = self.node
        sent = []
        if node:
            if node.hb_state == 5 and ("rpdo", 1) in node.pdo_maps:
                try:
                    vals = {(0x6040, 0): 0}
                    if any(e[0] == 0x6060 for e in node.pdo_maps[("rpdo", 1)]):
                        vals[(0x6060, 0)] = self.mode_sel or 0
                    node.send_rpdo(1, vals)
                    sent.append("RPDO1")
                except Exception as e:  # noqa: BLE001
                    self.log(f"E-STOP RPDO1: {e}", RED)
            self.worker.clear()
            d = self.drive
            d.halted = False
            self.halt_btn.blockSignals(True)
            self.halt_btn.setChecked(False)
            self.halt_btn.blockSignals(False)
            self.job(d.disable_voltage, None, "E-STOP disable voltage", urgent=True)
            sent.append("SDO")
        if self.main.link:
            self.main.config.stop()
            sent.append("USB STOP")
        self.log("E-STOP: " + (", ".join(sent) or "nothing connected")
                 + (". The USB STOP is a local stop: the drive faults 0xFF01, Fault reset after the coast"
                    if self.main.link and node else ""), RED)

    # ------------------------------------------------------------------ PP / PV / HM
    def pp_profile(self):
        node = self.node
        u = node.units
        v, a, dc = u.rpm_to_ips(self.pp_vel.value()), u.rpms_to_ips2(self.pp_acc.value()), u.rpms_to_ips2(self.pp_dec.value())

        def work():
            node.write(0x6081, 0, v)
            node.write(0x6083, 0, a)
            node.write(0x6084, 0, dc)
        self.job(work, lambda _: self.log("PP profile written (6081 / 6083 / 6084)"), "PP profile")

    def pp_go(self):
        d = self.drive
        u = d.units
        rel = self.pp_rel.currentIndex() == 1
        tgt = u.turn_to_inc(self.pp_target.value())
        imm = self.pp_imm.isChecked()

        def done(ok):
            self.pp_lbl.setText(colored("set-point acknowledged (bit 12), waiting for target reached (bit 10)", GREEN)
                                if ok else colored("not acknowledged: buffer full, or not enabled in PP", RED))
        self.job(lambda: d.pp_move(tgt, rel, imm), done, "PP set-point")

    def pv_set(self):
        node = self.node
        u = node.units
        t, a, dc = u.rpm_to_ips(self.pv_target.value()), u.rpms_to_ips2(self.pv_acc.value()), u.rpms_to_ips2(self.pv_dec.value())

        def work():
            node.write(0x6083, 0, a)
            node.write(0x6084, 0, dc)
            node.write(0x60FF, 0, t)
        self.job(work, lambda _: self.log(f"PV: 60FF = {self.pv_target.value():.1f} rpm"), "PV")

    def hm_start(self):
        d = self.drive
        m, off = self.hm_method.currentData(), d.units.turn_to_inc(self.hm_offset.value())
        self.hm_lbl.setText("homing…")

        def done(r):
            att, err = r
            self.hm_lbl.setText(colored("attained" if att else "homing error" if err else "no result", GREEN if att else RED))
        self.job(lambda: d.home(m, off), done, "homing")

    # ------------------------------------------------------------------ streamer
    def gen_changed(self, i):
        name, params = GENS[i]
        unit = UNITS.get(self.mode_cb.currentData(), "")
        for k in range(4):
            vis = k < len(params)
            self.gen_lbls[k].setVisible(vis)
            self.gen_sp[k].setVisible(vis)
            if vis:
                label, default, dec = params[k]
                self.gen_lbls[k].setText(label.replace("(/s)", f"({unit}/s)").replace("(/s²)", f"({unit}/s²)")
                                         + (f" ({unit})" if label in ("amplitude", "offset", "limit", "distance") else ""))
                self.gen_sp[k].setDecimals(dec)
                key = f"co/gen/{name}/{k}"
                self.gen_sp[k].setValue(float(self.settings.value(key, default)))
        self.gen_file_row.setVisible(name == "file")

    def pick_gen_file(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Set-point file", "", "CSV (*.csv *.txt)")
        if path:
            self.gen_file.setText(path)

    def make_gen(self):
        name, params = GENS[self.gen_cb.currentIndex()]
        vals = [self.gen_sp[k].value() for k in range(len(params))]
        for k, v in enumerate(vals):
            self.settings.setValue(f"co/gen/{name}/{k}", v)
        if name == "file":
            return C.gen_file(self.gen_file.text())
        return C.GENERATORS[name](*vals)

    def stream_start(self):
        d = self.drive
        mode = self.mode_cb.currentData()
        if d is None or mode not in UNITS:
            return
        if self.streamer and self.streamer.running:
            self.log("a stream is running: stop it first", ORANGE)
            return
        try:
            gen = self.make_gen()
        except (OSError, ValueError) as e:
            self.log(f"generator: {e}", RED)
            return
        period = self.st_period.value() / 1000.0
        s = C.Streamer(d, mode, gen, period, self.st_vmax.value() or None, self.st_pmax.value() or None,
                       self.st_dur.value() or None)
        enable = self.st_enable.isChecked()
        node = d.node

        def work():
            if enable:
                node.nmt("OPERATIONAL")
                time.sleep(0.05)
                node.pdo_read()
                if mode == C.MODE_CSV:
                    node.write(0x60FF, 0, 0)
                elif mode == C.MODE_CST:
                    node.write(0x6071, 0, 0)
                d.goto_enabled(mode)
            elif C.fsa_state(d.sw()) != C.OE:
                raise C.CoError("not in operation enabled (tick 'enable in the mode first')")
            return s.start()

        def done(st):
            self.streamer = st
            self.mode_sel = mode
            self.log(f"streaming {GENS[self.gen_cb.currentIndex()][0]} in {C.MODES[mode]} every {period * 1000:.1f} ms", GREEN)
        self.job(work, done, "stream")

    def stream_stop(self):
        s = self.streamer
        if not s:
            return
        s.stop()
        node, mode = self.node, s.mode
        if node and mode in (C.MODE_CSV, C.MODE_CST):
            idx = 0x60FF if mode == C.MODE_CSV else 0x6071
            self.job(lambda: node.write(idx, 0, 0), None, "zero target")
        if self.bus and not self.sync_on.isChecked():
            self.bus.stop_sync()
        self.log(f"stream stopped after {s.sent} set-points" + (f", {s.errors} send errors" if s.errors else ""))

    # ------------------------------------------------------------------ plots
    def build_plots(self):
        self.glw.clear()
        self.curves = []
        self.plot_items = []
        first = None
        row = 0
        for pi, (cb, (title, chans, _)) in enumerate(zip(self.plot_checks, PLOTS)):
            if not cb.isChecked():
                continue
            p = self.glw.addPlot(row=row, col=0, title=title)
            row += 1
            p.showGrid(x=True, y=True, alpha=0.25)
            p.addLegend(offset=(10, 5))
            p.setClipToView(True)
            p.setDownsampling(auto=True, mode="peak")
            p.setXRange(-self.win_s.value(), 0, padding=0)
            p.enableAutoRange(axis="x", enable=False)
            p.setAutoVisible(y=True)
            self.plot_items.append(p)
            if first is None:
                first = p
            else:
                p.setXLink(first)
            if title.startswith("Statusword"):
                for b, name, color in SW_TRACE_BITS:
                    self.curves.append((pi, ("bit", b), p.plot(pen=pg.mkPen(color, width=1), name=f"{b} {name}")))
                continue
            for sig, name, color in chans:
                self.curves.append((pi, sig, p.plot(pen=pg.mkPen(color, width=1), name=name)))
        if first is not None:
            first.setLabel("bottom", "time", "s")
        self.settings.setValue("co/plots", [cb.text() for cb in self.plot_checks if cb.isChecked()])

    def reset_view(self):
        for p in self.plot_items:
            p.setXRange(-self.win_s.value(), 0, padding=0)
            p.enableAutoRange(axis="y", enable=True)

    def convert(self, key, v):
        u = self.node.units
        idx = key[0]
        if idx in (0x6064, 0x6062):
            return v / u.inc_rev
        if idx in (0x606C, 0x606B):
            return v * 60.0 / u.inc_rev
        if idx == 0x60F4:
            return v * 360.0 / u.inc_rev
        if idx == 0x6079:
            return v / 1000.0
        return v

    def update_plots(self):
        node = self.node
        if node is None or self.plot_pause.isChecked() or not self.curves:
            return
        now = time.time()
        win = self.win_s.value()
        n = int(win * 1100) + 50  # up to ~1 kHz SYNC
        s = self.streamer
        for pi, sig, curve in self.curves:
            if sig == "sent":
                if s is None or SENT_PLOT.get(s.mode) != pi:
                    curve.setData([], [])
                    continue
                rows = s.log.snapshot()[-n:]
                if not rows:
                    curve.setData([], [])
                    continue
                a = np.array(rows, float)
                t = a[:, 0]
                y = a[:, 1] / node.units.inc_rev if s.mode == C.MODE_CSP else a[:, 2]
            elif isinstance(sig, tuple) and sig[0] == "bit":
                t, v = node.series.tail((0x6041, 0), n)
                b = sig[1]
                rank = [x[0] for x in SW_TRACE_BITS].index(b)
                y = ((v.astype(np.int64) >> b) & 1) * 0.8 + rank
                if len(t):  # TPDO1 comes on change: hold the last value up to now
                    t = np.append(t, now)
                    y = np.append(y, y[-1])
                curve.setData(t - now, y, stepMode=None)
                continue
            else:
                t, v = node.series.tail(sig, n)
                y = self.convert(sig, v)
            k = t >= now - win - 0.5
            curve.setData(t[k] - now, y[k])

    def toggle_record(self, on):
        if on:
            if not self.bus:
                self.rec_btn.setChecked(False)
                return
            path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Record CANopen data", time.strftime("co_%Y%m%d_%H%M%S.npz"),
                                                            "NumPy (*.npz);;CSV (*.csv)")
            if not path:
                self.rec_btn.blockSignals(True)
                self.rec_btn.setChecked(False)
                self.rec_btn.blockSignals(False)
                return
            node = self.node
            heads = {k: node.series.head(k) for k in node.series.keys()}
            self.rec = (path, time.time(), {}, heads, self.streamer.log.total if self.streamer else 0)
            self.rec_btn.setText("Stop rec")
            self.log(f"recording to {path}", GREEN)
        else:
            self.rec_btn.setText("Record…")
            if self.rec:
                self.rec_collect()
                self.rec_save()
            self.rec = None

    def rec_collect(self):
        """pull the samples since the last call into the recording (called from the timer and at the end)"""
        if not self.rec or not self.node:
            return
        path, t0, acc, heads, _ = self.rec
        series = self.node.series
        for k in series.keys():
            t, v, h = series.since(k, heads.get(k, 0))
            heads[k] = h
            if len(t):
                a = acc.setdefault(k, ([], []))
                a[0].append(t)
                a[1].append(v)

    def rec_save(self):
        path, t0, acc, heads, sent0 = self.rec
        s = self.streamer
        sent = [r for r in s.log.snapshot() if r[0] >= t0] if s else []
        names = {}
        for (idx, sub), (ts, vs) in acc.items():
            names[f"x{idx:04X}_{sub}"] = (np.concatenate(ts), np.concatenate(vs))
        try:
            if path.lower().endswith(".csv"):
                with open(path, "w", newline="", encoding="utf-8") as f:
                    w = csv.writer(f)
                    w.writerow(["t", "signal", "value"])
                    for name, (t, v) in names.items():
                        for a, b in zip(t, v):
                            w.writerow([f"{a - t0:.5f}", name, f"{b:.10g}"])
                    for a, raw, e in sent:
                        w.writerow([f"{a - t0:.5f}", "sent_raw", raw])
                        w.writerow([f"{a - t0:.5f}", "sent", f"{e:.10g}"])
            else:
                arrays = {}
                for name, (t, v) in names.items():
                    arrays[name + "_t"] = t - t0
                    arrays[name] = v
                if sent:
                    a = np.array(sent, float)
                    arrays["sent_t"], arrays["sent_raw"], arrays["sent"] = a[:, 0] - t0, a[:, 1], a[:, 2]
                arrays["t0"] = np.array(t0)
                arrays["inc_rev"] = np.array(self.node.units.inc_rev if self.node else C.INC_REV)
                arrays["rated_ma"] = np.array(self.node.units.rated_ma if self.node else 0)
                np.savez(path, **arrays)
            n = sum(len(t) for t, _ in names.values())
            self.log(f"recording saved: {path} ({n:,} samples, {len(sent):,} set-points)", GREEN)
        except OSError as e:
            self.log(f"cannot write {path}: {e}", RED)

    # ------------------------------------------------------------------ OD browser
    def build_od_tree(self):
        t = self.od_tree
        t.clear()
        self.od_items.clear()
        self.od_vals.clear()
        node = self.node
        if node is None:
            return
        od = node.node.object_dictionary
        self._od_updating = True
        for idx in sorted(i for i in od.indices if i >= 0x1000):  # not the 0x0002-0x0007 data type entries
            e = od[idx]
            if isinstance(e, C.canopen.objectdictionary.ODVariable):
                it = self._od_item(e, f"0x{idx:04X}")
                t.addTopLevelItem(it)
            else:
                top = QtWidgets.QTreeWidgetItem([f"0x{idx:04X}", e.name, "", "", "", "", ""])
                f = top.font(1)
                f.setBold(True)
                top.setFont(1, f)
                t.addTopLevelItem(top)
                for sub, v in sorted(e.subindices.items()):
                    top.addChild(self._od_item(v, f"  :{sub}"))
        for c in range(4):
            t.resizeColumnToContents(c)
        t.setColumnWidth(4, 160)
        self._od_updating = False

    def _od_item(self, v, label):
        dt = v.data_type
        dflt = v.default
        it = QtWidgets.QTreeWidgetItem([label, v.name, DT_NAMES.get(dt, hex(dt)), v.access_type, "",
                                        fmt_value(dflt, dt) if dflt is not None else "", (v.description or "").strip()])
        it.setData(0, QtCore.Qt.ItemDataRole.UserRole, (v.index, v.subindex))
        it.setFlags(it.flags() | QtCore.Qt.ItemFlag.ItemIsEditable)
        tip = f"0x{v.index:04X}:{v.subindex} {v.name}\n{DT_NAMES.get(dt, dt)}, {v.access_type}" + (
            ", PDO mappable" if v.pdo_mappable else "") + (f"\nrange {v.min} … {v.max}" if v.min is not None else "")
        for c in range(7):
            it.setToolTip(c, tip)
        self.od_items[(v.index, v.subindex)] = it
        return it

    def od_apply_filter(self, text):
        text = text.strip().lower()
        t = self.od_tree
        for i in range(t.topLevelItemCount()):
            top = t.topLevelItem(i)
            hay = (top.text(0) + " " + top.text(1)).lower()
            kids = [top.child(k) for k in range(top.childCount())]
            hit = not text or text in hay or any(text in (c.text(1).lower()) for c in kids)
            top.setHidden(not hit)

    def od_targets(self, items):
        out = []
        for it in items:
            key = it.data(0, QtCore.Qt.ItemDataRole.UserRole)
            if key:
                out.append(key)
            else:
                out += [it.child(k).data(0, QtCore.Qt.ItemDataRole.UserRole) for k in range(it.childCount())]
        return out

    def od_read_selected(self):
        self.od_read(self.od_targets(self.od_tree.selectedItems()))

    def od_read_all(self):
        self.od_read(list(self.od_items))

    def od_read(self, keys):
        node = self.node
        if node is None:
            return
        for key in keys:
            v = node.od_entry(*key)
            if "r" not in v.access_type or v.data_type == 0xF:
                continue
            self.job(lambda k=key: node.read(*k), lambda r, k=key: self.od_show(k, r), f"read 0x{key[0]:04X}:{key[1]}")

    def od_show(self, key, val):
        it = self.od_items.get(key)
        if it is None:
            return
        self.od_vals[key] = val
        v = self.node.od_entry(*key) if self.node else None
        self._od_updating = True
        it.setText(4, fmt_value(val, v.data_type if v else None))
        changed = v is not None and "w" in v.access_type and v.default is not None and val != v.default
        f = it.font(4)
        f.setBold(changed)
        it.setFont(4, f)
        self._od_updating = False

    def od_begin_edit(self, it, col):
        key = it.data(0, QtCore.Qt.ItemDataRole.UserRole)
        if not key or "w" not in self.node.od_entry(*key).access_type:
            return
        self._od_updating = True
        if key in self.od_vals:
            it.setText(4, str(self.od_vals[key]))  # plain value to edit
        self._od_updating = False
        self.od_tree.editItem(it, 4)

    def od_item_changed(self, it, col):
        if self._od_updating or col != 4:
            return
        key = it.data(0, QtCore.Qt.ItemDataRole.UserRole)
        node = self.node
        if not key or node is None:
            return
        v = node.od_entry(*key)
        try:
            val = parse_value(it.text(4), v.data_type)
        except ValueError:
            self.log(f"0x{key[0]:04X}:{key[1]}: '{it.text(4)}' is not a value for {DT_NAMES.get(v.data_type)}", RED)
            self.od_show(key, self.od_vals.get(key))
            return

        def work():
            node.write(key[0], key[1], val, retry=key[0] not in (0x6040, 0x1010, 0x1011))
            return node.read(*key) if "r" in v.access_type else val

        def failed_or_done(r):
            self.od_show(key, r)
            self.log(f"0x{key[0]:04X}:{key[1]} {v.name} = {fmt_value(r, v.data_type)}", GREEN)
        self.worker.submit(work, lambda r, err: (self.log(f"write 0x{key[0]:04X}:{key[1]}: {err}", RED),
                                                 self.od_show(key, self.od_vals.get(key))) if err else failed_or_done(r))

    def store(self, sub):
        node = self.node
        if node is None:
            return
        self.job(lambda: node.store(sub), lambda _: self.log(f"0x1010:{sub} 'save': stored", GREEN), f"store 0x1010:{sub}")

    def od_store(self):
        sub = self.store_cb.currentData()
        r = QtWidgets.QMessageBox.question(self, "Store", f"Write 'save' to 0x1010:{sub}?\n\nOnly while the motor is off "
                                           "(the flash erase stalls the CPU).")
        if r == QtWidgets.QMessageBox.StandardButton.Yes:
            self.store(sub)

    def od_restore(self):
        sub = self.store_cb.currentData()
        r = QtWidgets.QMessageBox.question(self, "Restore", f"Write 'load' to 0x1011:{sub}?\n\nThe exported defaults load "
                                           "at the next communication reset (comm) or boot (application).")
        if r == QtWidgets.QMessageBox.StandardButton.Yes:
            node = self.node
            self.job(lambda: node.restore(sub), lambda _: self.log(f"0x1011:{sub} 'load': defaults from the next reset",
                                                                   GREEN), f"restore 0x1011:{sub}")

    # ------------------------------------------------------------------ PDO mapping
    def pdo_refresh(self):
        node = self.node
        if node:
            self.job(node.pdo_read, lambda _: self.pdo_show(), "PDO mapping")

    def pdo_show(self):
        node = self.node
        if node is None:
            return
        for r, (kind, n) in enumerate([("tpdo", i) for i in range(1, 5)] + [("rpdo", i) for i in range(1, 5)]):
            try:
                info = node.pdo_info(kind, n)
            except (KeyError, AttributeError):
                continue
            cells = [f"0x{info['cob_id'] or 0:03X}", "yes" if info["enabled"] else "no", str(info["trans_type"]),
                     "" if info["inhibit"] is None or kind == "rpdo" else str(info["inhibit"]),
                     "" if info["event"] is None else str(info["event"]),
                     ", ".join(f"{i:04X}:{s} {name} ({bits})" for i, s, name, bits in info["map"])]
            for c, text in enumerate(cells):
                self.pdo_table.setItem(r, c, QtWidgets.QTableWidgetItem(text))
        self.pdo_table.resizeColumnsToContents()
        self.pdo_selected()

    def pdo_selected(self):
        rows = {i.row() for i in self.pdo_table.selectedItems()}
        if rows:
            self.pdo_sel.setCurrentIndex(min(rows))
        node = self.node
        kind, n = self.pdo_sel.currentData()
        if node is None:
            return
        try:
            info = node.pdo_info(kind, n)
        except (KeyError, AttributeError):
            return
        self.pdo_map.setText(" ".join(f"{i:04X}:{s}" for i, s, _, _ in info["map"]))
        self.pdo_tt.setValue(info["trans_type"] or 0)
        self.pdo_ev.setValue(info["event"] or 0)

    def pdo_write(self):
        kind, n = self.pdo_sel.currentData()
        try:
            entries = []
            for part in self.pdo_map.text().replace(",", " ").split():
                i, _, s = part.partition(":")
                entries.append((int(i, 16), int(s or "0", 0)))
        except ValueError:
            self.log(f"mapping '{self.pdo_map.text()}': write index:sub in hex, e.g. 6064:0 606C:0", RED)
            return
        self.pdo_apply(kind, n, entries, self.pdo_tt.value(), self.pdo_ev.value())

    def pdo_apply(self, kind, n, entries, tt, ev):
        node = self.node
        if node is None:
            return
        if node.hb_state == 5:
            self.log("PDO mapping is changed in pre-operational only: NMT pre-op first", ORANGE)
            return
        self.job(lambda: node.pdo_remap(kind, n, entries, tt, ev),
                 lambda _: (self.pdo_show(), self.log(f"{kind.upper()}{n} mapped: " + (
                     ", ".join(f"{i:04X}:{s}" for i, s in entries) or "disabled") + "; 'Store comm' keeps it", GREEN)),
                 f"{kind.upper()}{n} mapping")

    def pdo_preset(self):
        label, entries = TPDO4_PRESETS[self.preset_cb.currentIndex()]
        self.pdo_apply("tpdo", 4, entries, 1, 0)

    # ------------------------------------------------------------------ trace and EMCY
    def update_trace(self):
        b = self.bus
        if b is None:
            return
        items, self.trace_total = b.trace.since(self.trace_total)
        if self.tr_pause.isChecked() or not items:
            return
        inc, exc = parse_ids(self.tr_filter.text())
        hide = self.tr_hide.isChecked()
        noisy = {"SYNC", "TPDO2", "TPDO3", "HB (PC)"}
        t = self.tr_table
        if self.tr_t0 is None:
            self.tr_t0 = items[0][0]
        rows = []
        for ts, cid, data, d, fl in items:
            kind = "error frame" if fl else C.classify(cid)[0]
            if hide and kind in noisy:
                continue
            if inc and not any(lo <= cid <= hi for lo, hi in inc):
                continue
            if any(lo <= cid <= hi for lo, hi in exc):
                continue
            rows.append((f"{ts - self.tr_t0:10.4f}", d, f"0x{cid:03X}", kind, str(len(data)), data.hex(" "),
                         C.describe(cid, data)))
        if not rows:
            return
        rows = rows[-2000:]
        t.setUpdatesEnabled(False)
        start = t.rowCount()
        t.setRowCount(start + len(rows))
        for r, cells in enumerate(rows):
            for c, text in enumerate(cells):
                it = QtWidgets.QTableWidgetItem(text)
                if c == 1 and text == "tx":
                    it.setForeground(QtGui.QBrush(QtGui.QColor(BLUE)))
                if c == 3 and text in ("EMCY", "error frame"):
                    it.setForeground(QtGui.QBrush(QtGui.QColor(RED)))
                t.setItem(start + r, c, it)
        extra = t.rowCount() - 3000
        if extra > 0:
            for _ in range(extra):
                t.removeRow(0)
        t.setUpdatesEnabled(True)
        t.scrollToBottom()

    def trace_clear(self):
        self.tr_table.setRowCount(0)
        if self.bus:
            self.bus.trace.clear()

    def trace_save(self):
        if not self.bus:
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save trace", time.strftime("can_%Y%m%d_%H%M%S.csv"),
                                                        "CSV (*.csv);;Vector ASC (*.asc);;BLF (*.blf);;candump log (*.log)")
        if not path:
            return
        frames = self.bus.trace.snapshot()
        try:
            if path.lower().endswith(".csv"):
                with open(path, "w", newline="", encoding="utf-8") as f:
                    w = csv.writer(f)
                    w.writerow(["t", "dir", "cob_id", "kind", "data", "decoded"])
                    for ts, cid, data, d, fl in frames:
                        w.writerow([f"{ts:.6f}", d, f"0x{cid:03X}", "error" if fl else C.classify(cid)[0], data.hex(" "),
                                    C.describe(cid, data)])
            else:
                lg = can.Logger(path)
                for ts, cid, data, d, fl in frames:
                    lg.on_message_received(can.Message(timestamp=ts, arbitration_id=cid, data=data, is_extended_id=False,
                                                       is_rx=(d == "rx"), is_error_frame=bool(fl)))
                lg.stop()
            self.log(f"trace saved: {path} ({len(frames):,} frames)", GREEN)
        except Exception as e:  # noqa: BLE001 (python-can loggers raise various kinds)
            self.log(f"cannot save {path}: {e}", RED)

    def update_emcy(self):
        node = self.node
        if node is None:
            return
        items, self.emcy_total = node.emcy.since(self.emcy_total)
        t = self.em_table
        for ts, code, reg, bit, info, text, at_connect in items:
            r = t.rowCount()
            t.insertRow(r)
            cells = [f"{ts - self.bus.connected_at:.3f}", f"0x{code:04X}", text,
                     f"0x{reg:02X} {C.bits_text(reg, C.ER_BITS)}", f"0x{bit:02X} {C.EM_BITS.get(bit, '')}",
                     f"0x{info:08X}", "at connect" if at_connect else ""]
            for c, s in enumerate(cells):
                it = QtWidgets.QTableWidgetItem(s)
                if code and not at_connect:
                    it.setForeground(QtGui.QBrush(QtGui.QColor(RED)))
                elif at_connect:
                    it.setForeground(QtGui.QBrush(QtGui.QColor(GREY)))
                t.setItem(r, c, it)
            if code and not at_connect:
                self.log(f"EMCY 0x{code:04X} {text}", RED)
        if items:
            t.resizeColumnsToContents()
            t.scrollToBottom()

    # ------------------------------------------------------------------ tests
    def run_tests(self, all_):
        if not self.bus or self.testing:
            return
        tests = [t for t, it in self.test_items.values() if all_ or it.checkState(0) == QtCore.Qt.CheckState.Checked]
        if not tests:
            self.log("no test checked", ORANGE)
            return
        motion = self.t_motion.isChecked()
        if motion and any(t.motion for t in tests):
            r = QtWidgets.QMessageBox.warning(self, "Motion tests", "These tests spin the motor (up to a few hundred rpm, "
                                              "CSP/PP moves of a few turns). The shaft must be free.\n\nStart?",
                                              QtWidgets.QMessageBox.StandardButton.Ok | QtWidgets.QMessageBox.StandardButton.Cancel)
            if r != QtWidgets.QMessageBox.StandardButton.Ok:
                return
        usb = C.UsbSync(self.main.link) if self.main.link and self.main.config.proto_ok else None
        if self.streamer:
            self.stream_stop()
        self.cancel.clear()
        self.testing = True
        self.set_controls(False)
        self.t_stop.setEnabled(True)
        self.worker.clear()
        for _, it in self.test_items.values():
            it.setText(3, "")
        self.t_log.clear()
        runner = cotest.Runner(self.bus, self.node_id, usb, log=self._test_line, cancel=self.cancel, motion=motion)
        self.t_result.setText("running…" + ("" if usb else "  (no USB link: USB tests skip)"))

        def go():
            try:
                runner.run(tests)
            except Exception as e:  # noqa: BLE001
                self._test_line(f"runner: {type(e).__name__}: {e}")
            self._test_line(("__done__", runner))
        threading.Thread(target=go, daemon=True, name="cotest").start()

    def _test_line(self, line):
        with self.test_lock:
            self.test_lines.append(line)

    def drain_tests(self):
        with self.test_lock:
            lines, self.test_lines = self.test_lines, []
        for line in lines:
            if isinstance(line, tuple):
                runner = line[1]
                self.testing = False
                self.set_controls(bool(self.bus))
                for r in runner.results:
                    if r["name"] in self.test_items:
                        it = self.test_items[r["name"]][1]
                        it.setText(3, r["result"])
                        it.setForeground(3, QtGui.QBrush(QtGui.QColor(
                            {"pass": GREEN, "skip": GREY}.get(r["result"], RED))))
                n_ok = sum(r["result"] == "pass" for r in runner.results)
                self.t_result.setText(f"{n_ok}/{len(runner.results)} passed  ->  {runner.path}.json")
                self._drive = None  # a test may have reconnected: new node objects
                if self.bus:
                    self.pdo_refresh()
                continue
            self.t_log.appendPlainText(line)

    # ------------------------------------------------------------------ timers
    def poll(self):
        """statusword over SDO when TPDO1 is not coming (not operational), the refusal text when bit 7 is set"""
        node = self.node
        if node is None or self.testing or self.worker.pending() > 2:
            return
        if time.time() - node.sw_t > 0.35:
            def work():
                sw = node.read(0x6041)
                node.sw, node.sw_t = sw, time.time()
                node.mode_disp = node.read(0x6061)
                node.err_code = node.read(0x603F)
            self.job(work, None, "status")
        sw = node.sw
        if sw is not None and sw & 0x88 and self.refusal_for != sw:
            self.refusal_for = sw
            d = self.drive

            def work2():
                st = d.refusal() if sw & 0x80 else (0, "")
                return st, d.fault_text() if C.fsa_state(sw) in (C.FAULT, C.FRA) else None
            self.job(work2, self.show_refusal, "refusal")

    def show_refusal(self, r):
        (st, why), fault = r
        parts = []
        if fault:
            parts.append(colored(f"fault {fault[1]}", RED))
        if st:
            parts.append(colored(f"refused: {why}", ORANGE))
        self.fault_lbl.setText("<br>".join(parts))

    def refresh(self):
        self.worker.drain()
        self.drain_tests()
        node = self.node
        if node is None:
            return
        now = time.time()
        sw = node.sw
        if sw is not None:
            mode = node.mode_disp
            st, bits = C.decode_sw(sw, mode)
            self.state_lbl.setText(colored(C.FSA_NAMES.get(st, "?"), STATE_COLOR.get(st)))
            self.sw_lbl.setText(f"6041 0x{sw:04X}")
            for b, name, on in bits:
                self.bit_lbls[b].setText(colored(f"{b} {name}", GREEN if on else GREY))
            self.mode_disp.setText(f"6061 {C.MODES.get(mode, mode)}")
            if not sw & 0x88 and self.fault_lbl.text():
                self.fault_lbl.setText("")
                self.refusal_for = None
            if self.mode_cb.currentData() == C.MODE_PP:
                self.pp_lbl.setText(f"bit 12 ack {'1' if sw & 0x1000 else '0'}, bit 10 reached {'1' if sw & 0x400 else '0'}, "
                                    f"bit 13 following error {'1' if sw & 0x2000 else '0'}")
        if now - self.last_stats >= 0.5:
            self.last_stats = now
            b = self.bus
            exc = b.network.notifier.exception if b.network else None
            if exc is not None and not getattr(b, "_exc_logged", False):
                b._exc_logged = True
                self.log(f"CAN reception stopped: {exc!r} (cause {exc.__cause__!r}). Disconnect and connect again", RED)
            fps, load = b.rates()
            alive = node.hb_alive()
            col = GREEN if node.hb_state == 5 else ORANGE if node.hb_state == 127 else RED
            self.bus_lbl.setText(colored(node.nmt_state.lower() if alive else "no heartbeat", col if alive else RED)
                                 + f"  (node {node.id})" + (f"<br>{self.ident}" if self.ident else ""))
            err = f", <span style='color:{RED}'>{b.err_frames} error frames</span>" if b.err_frames else ""
            txe = f", <span style='color:{RED}'>{b.tx_errors} TX errors</span>" if b.tx_errors else ""
            self.stats_lbl.setText(f"{fps:,.0f} frames/s, load {load * 100:.1f} %<br>RX {b.rx_frames:,}, TX {b.tx_frames:,}"
                                   f"{err}{txe}<br>SDO {node.sdo_count:,}, retried {node.retries}, EMCY {node.emcy_count}"
                                   + (f"<br>PC heartbeat {'PAUSED' if b.hb.paused else 'on'}" if b.hb else ""))
            sp = b.sync
            j = sp.jitter() * 1000 if sp else np.zeros(0)
            if len(j) > 5:
                j = j[-2000:]
                self.jit_lbl.setText(f"{j.mean():.2f} ms, {j.min():.2f}–{j.max():.2f}")
                hist, edges = np.histogram(j, bins=40)
                self.jit_curve.setData(edges, hist)
            else:
                self.jit_lbl.setText("")
            s = self.streamer
            if s:
                txt = (f"{'running' if s.running else 'stopped'}: {s.elapsed():.1f} s, {s.sent:,} set-points"
                       + (f", {s.errors} send errors" if s.errors else ""))
                if s.tripped:
                    txt += "<br>" + colored(f"GUARD: {s.tripped} (disable voltage sent)", RED)
                self.st_lbl.setText(txt)
                if s.tripped and not getattr(s, "_logged", False):
                    s._logged = True
                    self.log(f"stream guard: {s.tripped}", RED)
        if now - self.last_slow >= 0.2:
            self.last_slow = now
            self.update_emcy()
            self.update_trace()
            if self.rec:
                self.rec_collect()
        if self.sub.currentIndex() == 0 and self.isVisible():
            self.update_plots()
