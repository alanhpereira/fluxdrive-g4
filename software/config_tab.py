"""
Config tab of the plotter: parameter grid, calibration wizard and log, plus the run panel and the
status/statistics boxes that the plotter shows in its sidebar, all over the binary protocol (protocol.py, objects.py). docs/design-notes.md has the design.
"""
import decimal
import html
import json
import time

from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

import objects as O
import protocol as P

RED, GREEN, ORANGE, BLUE = "#ff6060", "#60c060", "#f0a040", "#7fb2ff"
MODES = {0: "off", 1: "calibration", 2: "run"}
LOOPS = {0: "current", 1: "speed", 2: "position"}
DRV_BITS = [(0x01, "VCC_UVLO"), (0x02, "THSD"), (0x04, "VDS_P"), (0x08, "RESET"), (0x80, "locked")]
ENC_BITS = [(0x01, "OVERSPEED"), (0x02, "WEAK_FIELD"), (0x04, "UNDERVOLT")]
CAN_BITS = [(0x0004, "BUS-OFF"), (0x0002, "TX passive"), (0x0001, "TX warning"), (0x0008, "TX overflow"),
            (0x0080, "PDO late"), (0x0200, "RX passive"), (0x0100, "RX warning"), (0x0800, "RX overflow")]  # CO_CAN_ERR*
CPU_MHZ = 170.0
C_LOCALE = QtCore.QLocale.c()  # '.' decimals whatever the OS locale: with ',' locales "0.75" was read as 75 / 7

# calibration steps: (key, CAL which, label, done flag (None: optional), argument (False none, True a test current,
# "pp" pole pairs), safety prompt)
STEPS = [
    ("c", P.CAL_CUR_OFF, "Current-sense offsets (rotor still)", O.PF_CUR_CAL_VALID, False, None),
    ("e", P.CAL_ENC, "Encoder: dir, pole pairs, offset", O.PF_ELEC_OFFSET_VALID, False,
     "The rotor turns slowly under a forced field (about 20 electrical turns, then one mechanical turn each way).\n"
     "The shaft must be free to turn."),
    ("H", P.CAL_HALL, "Hall table (set pp first without an encoder)", O.PF_HALL_VALID, False,
     "The rotor turns slowly under a forced field: 5 electrical turns forward, then 5 back.\n"
     "The shaft must be free to turn. Pole pairs (pp) must be right: 'e' measures them, else set them by hand."),
    ("r", P.CAL_R, "Resistance", O.PF_R_VALID, False, None),
    ("l", P.CAL_LD, "Inductance d", O.PF_L_VALID, True, None),
    ("q", P.CAL_LQ, "Inductance q (optional, hold shaft)", None, True,
     "HOLD THE SHAFT FIRMLY.\nThe q-axis test applies torque; it starts 5 s after the rotor has aligned."),
    ("t", P.CAL_TUNE, "Current PI autotune (R, L, bw)", O.PF_PI_VALID, False, None),
    ("k", P.CAL_KV, "kv (spins freely)", "kv", False,
     "The motor spins up (up to half the bus voltage or 10000 rpm) and coasts down.\nUnload the shaft."),
    ("z", P.CAL_ZERO, "Reset encoder zero (EEPROM)", None, False,
     "Resets the MT6835 zero position and saves it to the encoder EEPROM.\n"
     "The electrical offset must be recalibrated ('e') afterwards."),
    ("u", P.CAL_UVW, "Encoder UVW outputs as Halls (bench)", None, "pp",
     "Sets the MT6835's U/V/W outputs to the pole pairs and saves them to its EEPROM (about 6 s).\n"
     "After 'e' it also moves the encoder zero onto electrical zero (the offset is adjusted; the Hall table is cleared).\n"
     "Wire U, V, W to J13 HALL1..3 to use them as Hall sensors. Then run 'H', home, and save."),
    ("h", P.CAL_HFI_MAP, "HFI saliency map (go/no-go)", "hsal", True,
     "Injects at 24 angles around the rotor with a d bias and iq = 0 (audible whine).\n"
     "The rotor should stay still; the encoder is the reference."),
    ("p", P.CAL_HFI_POL, "HFI lock + polarity test", None, False,
     "HFI locks from a random angle, then +d/-d pulses pick the polarity.\n"
     "Repeat from many rotor positions: it must be right every time."),
]

ARG_TIPS = {
    "u": "UVW pole pairs, 1..16. 'default' (0) uses pp",
    "h": "d-axis bias. 'default' (0) maps at 0, 0.25 and 0.5 x rated and at hid, and stores hsal from the hid one;\n"
         "an explicit current maps that bias only and stores nothing",
}

STATUS_KEYS = ["mode", "loop", "st_flags", "moe", "nfault", "brk", "ready", "trip", "trip_moe", "coasting", "vin", "busy",
               "dirty", "seq", "t_board", "t_die", "t_motor", "i_temp", "t_state"]
DIAG_KEYS = ["drv", "loop_cyc", "loop_cyc_max", "enc_samples", "enc_crc", "enc_late", "enc_stalls", "enc_status",
             "enc_crc_check", "proto_err", "ov_resumes", "hall_state", "hall_bad", "hyb_reseeds", "coff_a_use", "coff_b_use", "coff_c_use", "coff_src",
             "can_id_act", "can_kbit_act", "can_nmt", "can_err", "can_rx", "can_tx", "can_lost", "fsa", "fsa_refused",
             "fsa_fault", "can_owner"]
TRIPS = {1: "bus overvoltage", 2: "overcurrent"}
COFF_SRC = {0: "default", 1: "stored", 2: "measured at boot"}
CLEAR_KEYS = ["loop_cyc_max", "enc_samples", "enc_crc", "enc_late", "enc_stalls", "proto_err", "ov_resumes", "can_rx", "can_tx",
              "can_lost", "hall_bad", "hyb_reseeds"]


def bits(v, table):
    return " ".join(name for b, name in table if v & b) or "-"


def same(o, a, b):
    """equal as the device stores them (float32 rounding)"""
    try:
        return P.pack_value(o.type, a) == P.pack_value(o.type, b)
    except (OverflowError, ValueError, TypeError):
        return False


class ChoiceDelegate(QtWidgets.QStyledItemDelegate):
    """Value column: a drop-down for objects with named choices (angsrc, hmode...), the plain line edit otherwise"""

    def _obj(self, index):
        key = index.siblingAtColumn(0).data(QtCore.Qt.ItemDataRole.UserRole)
        return O.BY_KEY.get(key) if key else None

    def createEditor(self, parent, option, index):
        o = self._obj(index)
        if not (o and o.choices):
            return super().createEditor(parent, option, index)
        cb = QtWidgets.QComboBox(parent)
        cb.addItems(list(o.choices.values()))
        cb.activated.connect(lambda _: (self.commitData.emit(cb), self.closeEditor.emit(cb)))
        return cb

    def setEditorData(self, editor, index):
        if isinstance(editor, QtWidgets.QComboBox):
            i = editor.findText(index.data())
            editor.setCurrentIndex(max(i, 0))
            editor.showPopup()
        else:
            super().setEditorData(editor, index)

    def setModelData(self, editor, model, index):
        if isinstance(editor, QtWidgets.QComboBox):
            model.setData(index, editor.currentText())
        else:
            super().setModelData(editor, model, index)


class ConfigTab(QtWidgets.QWidget):
    def __init__(self, main):
        super().__init__()
        self.main = main  # .link (protocol.Link or None), .say(text, color)
        self.dev = {}  # key -> value on the device (device units)
        self.edit = {}  # key -> pending value (device units)
        self.err = {}  # key -> last write error
        self.st = {}  # status/diagnostic values
        self.items = {}
        self.disabled = set()  # keys the device does not have (table mismatch)
        self.proto_ok = False
        self.cal_running = None
        self.countdown = None  # (end time, text)
        self.poll_n = 0
        self._updating = False
        self.run_panels = []

        split = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        split.addWidget(self._build_params())
        right = QtWidgets.QWidget()
        rl = QtWidgets.QVBoxLayout(right)
        rl.addWidget(self._build_cal())
        rl.addWidget(self._build_log())
        rl.addStretch(1)
        self.status_box, self.stats_box = self._build_status()  # the plotter puts them in its sidebar
        scroll = QtWidgets.QScrollArea(widgetResizable=True)
        scroll.setWidget(right)
        scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)  # wrap, never clip
        scroll.setMinimumWidth(right.minimumSizeHint().width() + 20)  # the parameter table gives way first: it scrolls sideways
        split.addWidget(scroll)
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 2)
        split.setSizes([600, 420])
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.addWidget(split)

        self.poll_timer = QtCore.QTimer(interval=250)
        self.poll_timer.timeout.connect(self.poll)
        self.poll_timer.start()
        self.set_enabled(False)

    # ------------------------------------------------------------------ layout
    def _build_params(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        v.setContentsMargins(0, 0, 0, 0)
        self.info = QtWidgets.QLabel("not connected")
        self.info.setWordWrap(True)
        v.addWidget(self.info)
        self.btn_read = QtWidgets.QPushButton("Read all")
        self.btn_read.clicked.connect(lambda: self.read_all())
        self.btn_write = QtWidgets.QPushButton("Write changed")
        self.btn_write.clicked.connect(self.write_changed)
        self.btn_revert = QtWidgets.QPushButton("Revert")
        self.btn_revert.setToolTip("Drop the pending edits")
        self.btn_revert.clicked.connect(self.revert)
        self.btn_save = QtWidgets.QPushButton("Save to flash")
        self.btn_save.clicked.connect(self.save)
        self.btn_export = QtWidgets.QPushButton("Export JSON…")
        self.btn_export.clicked.connect(self.export_json)
        self.btn_import = QtWidgets.QPushButton("Import JSON…")
        self.btn_import.clicked.connect(self.import_json)
        self.btn_reboot = QtWidgets.QPushButton("Reboot")
        self.btn_reboot.setToolTip("Reset the MCU (refused while the motor runs)")
        self.btn_reboot.clicked.connect(lambda: self.reboot(False))
        self.btn_dfu = QtWidgets.QPushButton("Reboot to DFU")
        self.btn_dfu.setToolTip("Reset into the ST bootloader: flash over USB with STM32CubeProgrammer")
        self.btn_dfu.clicked.connect(lambda: self.reboot(True))
        grid = QtWidgets.QGridLayout()  # rows of three: one row of six sets a wide minimum window width
        for i, b in enumerate((self.btn_read, self.btn_write, self.btn_revert, self.btn_save, self.btn_export, self.btn_import,
                               self.btn_reboot, self.btn_dfu)):
            grid.addWidget(b, i // 3, i % 3)
        v.addLayout(grid)
        self.dirty_lbl = QtWidgets.QLabel("")
        v.addWidget(self.dirty_lbl)

        t = self.tree = QtWidgets.QTreeWidget()
        t.setColumnCount(5)
        t.setHeaderLabels(["Parameter", "Value", "Unit", "Device", "Key"])
        t.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        t.setAlternatingRowColors(True)
        t.setItemDelegateForColumn(1, ChoiceDelegate(t))
        t.itemDoubleClicked.connect(self._begin_edit)
        t.itemChanged.connect(self._item_changed)
        for g in O.PARAM_GROUPS:
            top = QtWidgets.QTreeWidgetItem([g])
            f = top.font(0)
            f.setBold(True)
            top.setFont(0, f)
            t.addTopLevelItem(top)
            for o in O.OBJECTS:
                if o.group != g:
                    continue
                it = QtWidgets.QTreeWidgetItem([o.label, "", o.unit, "", o.key])
                it.setData(0, QtCore.Qt.ItemDataRole.UserRole, o.key)
                rng = ""
                if o.choices:
                    rng = "\n" + o.choices_text()
                elif o.lo is not None:
                    rng = f"\nrange {o.fmt(o.lo)} … {o.fmt(o.hi)} {o.unit}"
                flags = []
                if o.flags & O.NOTRUN:
                    flags.append("only while stopped")
                if o.flags & O.PI:
                    flags.append("applied to the controllers at once")
                tip = (f"{o.help}\n" if o.help else "") + f"0x{o.index:04X}:{o.sub}{rng}" + (
                    f"\n{', '.join(flags)}" if flags else "") + "\nDouble-click the value to edit."
                for c in range(5):
                    it.setToolTip(c, tip)
                it.setFlags(it.flags() | QtCore.Qt.ItemFlag.ItemIsEditable)
                it.setTextAlignment(1, QtCore.Qt.AlignmentFlag.AlignRight | QtCore.Qt.AlignmentFlag.AlignVCenter)
                it.setTextAlignment(3, QtCore.Qt.AlignmentFlag.AlignRight | QtCore.Qt.AlignmentFlag.AlignVCenter)
                top.addChild(it)
                self.items[o.key] = it
        t.expandAll()
        for i in range(t.topLevelItemCount()):
            t.topLevelItem(i).setFirstColumnSpanned(True)  # group titles do not set the Parameter column width
        for c in range(5):
            t.resizeColumnToContents(c)
        t.setColumnWidth(1, max(t.columnWidth(1), 110))
        t.setColumnWidth(3, max(t.columnWidth(3), 110))
        v.addWidget(t, 1)
        return w

    def _build_cal(self):
        g = QtWidgets.QGroupBox("Calibration")
        grid = QtWidgets.QGridLayout(g)
        self.step_lbl, self.step_btn, self.step_arg = {}, {}, {}
        for r, (key, which, label, flag, arg, prompt) in enumerate(STEPS):
            done = QtWidgets.QLabel("")
            done.setMinimumWidth(70)
            lb = QtWidgets.QLabel(f"<b>{key}</b>  {label}")
            lb.setWordWrap(True)
            if prompt:
                lb.setToolTip(prompt)
            grid.addWidget(lb, r, 0)
            grid.addWidget(done, r, 1)
            if arg == "pp":
                sp = QtWidgets.QDoubleSpinBox(decimals=0, minimum=0.0, maximum=16.0, singleStep=1.0, suffix=" pp")
            elif arg:
                sp = QtWidgets.QDoubleSpinBox(decimals=2, minimum=0.0, maximum=50.0, singleStep=0.5, suffix=" A")
            if arg:
                sp.setLocale(C_LOCALE)
                sp.setSpecialValueText("default")
                sp.setToolTip(ARG_TIPS.get(key, "Test current. 'default' (0) uses 0.4 x rated and stores the result;\n"
                                                "an explicit current measures that point only and stores nothing"))
                grid.addWidget(sp, r, 2)
                self.step_arg[key] = sp
            b = QtWidgets.QPushButton("Run")
            b.clicked.connect(lambda _=False, k=key: self.run_step(k))
            grid.addWidget(b, r, 3)
            self.step_lbl[key], self.step_btn[key] = done, b
        r = len(STEPS)
        self.cal_state = QtWidgets.QLabel("")
        self.cal_state.setWordWrap(True)
        grid.addWidget(self.cal_state, r, 0, 1, 3)
        self.abort_btn = QtWidgets.QPushButton("Abort")
        self.abort_btn.setObjectName("stop")
        self.abort_btn.clicked.connect(self.abort)
        grid.addWidget(self.abort_btn, r, 3)
        grid.setColumnStretch(0, 1)
        self.cd_timer = QtCore.QTimer(interval=100)
        self.cd_timer.timeout.connect(self._tick_countdown)
        return g

    def _build_log(self):
        g = QtWidgets.QGroupBox("Log")
        v = QtWidgets.QVBoxLayout(g)
        self.ev_log = QtWidgets.QPlainTextEdit(readOnly=True)
        self.ev_log.setMaximumBlockCount(5000)
        self.ev_log.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont))
        self.ev_log.setMinimumHeight(160)
        v.addWidget(self.ev_log)
        clear = QtWidgets.QPushButton("Clear")
        clear.clicked.connect(self.ev_log.clear)
        v.addWidget(clear, 0, QtCore.Qt.AlignmentFlag.AlignRight)
        return g

    def _build_status(self):
        """two sidebar boxes: state and faults, then the loop/encoder/link counters"""
        self.st_lbl = {}

        def box(title, rows):
            g = QtWidgets.QGroupBox(title)
            f = QtWidgets.QFormLayout(g)
            for key, label in rows:
                lb = QtWidgets.QLabel("-")
                lb.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
                lb.setWordWrap(True)
                f.addRow(label, lb)
                self.st_lbl[key] = lb
            return g, f
        status, _ = box("Status / faults", [("state", "State"), ("bridge", "Bridge"), ("trip", "Trip"),
                                             ("drv", "Gate driver"), ("vin", "Bus voltage"), ("temp", "Temperature"),
                                             ("flash", "Flash")])
        stats, f = box("Statistics", [("loop", "Loop time"), ("enc", "Encoder"), ("coff", "Current offsets"),
                                      ("link", "Link"), ("can", "CAN")])
        self.st_lbl["loop"].setToolTip("currentLoop() time: the max must stay under ~19 µs")
        b = QtWidgets.QPushButton("Clear max / counters")
        b.clicked.connect(self.clear_counters)
        f.addRow(b)
        self.btn_can_apply = QtWidgets.QPushButton("Apply CAN settings")
        self.btn_can_apply.setToolTip("CANopen communication reset with the node ID and bit rate now in the device\n"
                                      "(write them first; 'Save to flash' keeps them over a reboot). Only while stopped")
        self.btn_can_apply.clicked.connect(self.can_apply)
        f.addRow(self.btn_can_apply)
        return status, stats

    # ------------------------------------------------------------------ helpers
    @property
    def link(self):
        return self.main.link

    def log(self, text, color=None):
        """plain text (escaped, spaces kept: the event formatters align columns)"""
        style = "white-space:pre-wrap" + (f";color:{color}" if color else "")
        self.ev_log.appendHtml(f'<span style="{style}">{html.escape(text)}</span>')

    def set_enabled(self, on):
        for wdg in (self.btn_read, self.btn_write, self.btn_revert, self.btn_save, self.btn_export, self.btn_import,
                    self.btn_reboot, self.btn_dfu, self.btn_can_apply,
                    self.tree, *self.step_btn.values()):
            wdg.setEnabled(on)
        for rp in self.run_panels:
            rp.set_enabled(on)
        self._refresh_cal_buttons()

    def _req(self, mtype, payload=b"", cb=None, urgent=False):
        if not self.link:
            return
        self.link.request(mtype, payload, cb, urgent=urgent)

    # ------------------------------------------------------------------ connection
    def on_connect(self):
        self.proto_ok = False
        self.dev.clear()
        self.st.clear()
        self.disabled.clear()
        self.info.setText("connecting…")
        self._req(P.PING, cb=self._on_ping)

    def on_disconnect(self):
        self.proto_ok = False
        self.cal_running = None
        self.info.setText("not connected")
        self.set_enabled(False)

    def _on_ping(self, res):
        if res.status != P.ST_OK or len(res.payload) < 12:
            self.info.setText("<span style='color:%s'>no reply to PING: firmware without the binary protocol? "
                              "The Scope tab and the text console still work.</span>" % RED)
            return
        pv, h, parv, bid = P.parse_ping(res.payload)
        self.proto_ok = True
        self.device = (pv, h, parv, bid)
        warn = []
        if pv != P.PROTO_VERSION:
            warn.append(f"protocol version {pv}, GUI speaks {P.PROTO_VERSION}")
        if h != O.table_hash():
            warn.append("object table differs from objects.py: checking entries")
            self._list_from(0, [])
        text = f"device: protocol {pv}, PARAM_VERSION {parv}, build {bid:08X}, table {h:08X}"
        if warn:
            text += f"<br><span style='color:{ORANGE}'>{'; '.join(warn)}</span>"
        self.info.setText(text)
        self.set_enabled(True)
        self.read_all()

    def _list_from(self, first, acc):
        def cb(res):
            if res.status != P.ST_OK:
                self.log("LIST failed: " + P.status_text(res.status), RED)
                return
            entries = P.parse_list(res.payload)
            acc.extend(entries)
            if entries:
                self._list_from(first + len(entries), acc)
            else:
                self._apply_list(acc)
        self._req(P.LIST, P.p_list(first), cb)

    def _apply_list(self, entries):
        dev = {(i, s): (t, f) for i, s, t, f in entries}
        for o in O.OBJECTS:
            if dev.get((o.index, o.sub)) != (o.type, o.flags):
                self.disabled.add(o.key)
                self.log(f"0x{o.index:04X}:{o.sub} {o.key}: {'missing on the device' if (o.index, o.sub) not in dev else 'type/flags differ'}, disabled", ORANGE)
                if o.key in self.items:
                    self.items[o.key].setDisabled(True)
        known = {(o.index, o.sub) for o in O.OBJECTS}
        for i, s in dev:
            if (i, s) not in known:
                self.log(f"device has 0x{i:04X}:{s}, unknown to objects.py", ORANGE)

    # ------------------------------------------------------------------ parameters
    def params(self):
        return [o for o in O.OBJECTS if o.stored and o.key not in self.disabled]

    def read_all(self, then=None):
        objs = self.params()
        left = [len(objs)]
        fails = []

        def cb(o, res):
            if res.ok:
                self.dev[o.key] = P.unpack_value(o.type, res.payload)
                if o.key in self.edit and same(o, self.edit[o.key], self.dev[o.key]):
                    del self.edit[o.key]
            else:
                fails.append(f"{o.key}: {P.status_text(res.status)}")
            self.refresh_row(o.key)
            left[0] -= 1
            if left[0] == 0:
                if fails:
                    self.log("read failed: " + ", ".join(fails), RED)
                self._after_params()
                if then:
                    then(not fails)
        for o in objs:
            self._req(P.READ, P.p_read(o.index, o.sub), lambda r, o=o: cb(o, r))

    def _after_params(self):
        vl = self.dev.get("vlim")
        for rp in self.run_panels:
            rp.vlim_lbl.setText(f"limit ±{vl * 60:.0f} rpm" if vl is not None else "vlim ?")
        self._refresh_cal_buttons()

    def refresh_row(self, key):
        o, it = O.BY_KEY[key], self.items.get(key)
        if it is None:
            return
        self._updating = True
        dv, ev = self.dev.get(key), self.edit.get(key)
        pending = ev is not None and (dv is None or not same(o, ev, dv))
        it.setText(3, o.fmt(dv))
        it.setText(1, o.fmt(ev if pending else dv))
        f = it.font(1)
        f.setBold(pending)
        it.setFont(1, f)
        err = self.err.get(key)
        if err:
            it.setBackground(1, QtGui.QBrush(QtGui.QColor("#5a2020")))
        elif pending:
            it.setBackground(1, QtGui.QBrush(QtGui.QColor("#4a4020")))
        else:
            it.setBackground(1, QtGui.QBrush())
        it.setText(0, o.label + (" *" if pending else ""))
        self._updating = False

    def _begin_edit(self, it, col):
        key = it.data(0, QtCore.Qt.ItemDataRole.UserRole)
        if not key or key in self.disabled or O.BY_KEY[key].ro:
            return
        self.tree.editItem(it, 1)

    def _item_changed(self, it, col):
        if self._updating or col != 1:
            return
        key = it.data(0, QtCore.Qt.ItemDataRole.UserRole)
        if not key:
            return
        o = O.BY_KEY[key]
        text = it.text(1).strip()
        names = {n: k for k, n in (o.choices or {}).items()}
        try:
            if text in names:
                d = float(names[text])
            else:
                d = float(int(text, 0)) if o.type != P.T_F32 and text.lower().startswith("0x") else float(text)
            v = o.from_display(d)
        except ValueError:
            self.log(f"{key}: '{text}' is not a number", RED)
            self.refresh_row(key)
            return
        why = o.check(v)
        if why:
            self.log(f"{key}: {why}", RED)
            self.refresh_row(key)
            return
        self.err.pop(key, None)
        if key in self.dev and same(o, v, self.dev[key]):
            self.edit.pop(key, None)
        else:
            self.edit[key] = v
        self.refresh_row(key)

    def revert(self):
        keys = list(self.edit) + list(self.err)
        self.edit.clear()
        self.err.clear()
        for k in keys:
            self.refresh_row(k)

    def write_changed(self):
        todo = [k for k, v in self.edit.items() if k not in self.dev or not same(O.BY_KEY[k], v, self.dev[k])]
        if not todo:
            self.log("nothing to write")
            return
        left = [len(todo)]
        fails = []

        def cb(o, v, res):
            if res.ok:
                self.err.pop(o.key, None)
                self._req(P.READ, P.p_read(o.index, o.sub), lambda r: rb(o, r))  # read back what the device holds
            else:
                self.err[o.key] = P.status_text(res.status)
                fails.append(f"{o.key}: {self.err[o.key]}")
                self.refresh_row(o.key)
            left[0] -= 1
            if left[0] == 0:
                if fails:
                    self.log("write failed: " + ", ".join(fails), RED)
                else:
                    self.log(f"wrote {len(todo)} value(s) to RAM; 'Save to flash' keeps them", GREEN)

        def rb(o, res):
            if res.ok:
                self.dev[o.key] = P.unpack_value(o.type, res.payload)
                if o.key in self.edit and same(o, self.edit[o.key], self.dev[o.key]):
                    del self.edit[o.key]
            self.refresh_row(o.key)
            self._after_params()
        for k in todo:
            o = O.BY_KEY[k]
            v = self.edit[k]
            self._req(P.WRITE, P.p_write(o.index, o.sub, o.type, v), lambda r, o=o, v=v: cb(o, v, r))

    def save(self):
        if self.edit:
            self.log(f"{len(self.edit)} edit(s) not written yet: only what is on the device gets saved", ORANGE)
        r = QtWidgets.QMessageBox.question(
            self, "Save to flash",
            "Save the device parameters to flash?\n\nThe bridge is disabled for ~20 ms while the page is erased "
            "(refused while the motor runs).",
            QtWidgets.QMessageBox.StandardButton.Ok | QtWidgets.QMessageBox.StandardButton.Cancel)
        if r != QtWidgets.QMessageBox.StandardButton.Ok:
            return

        def cb(res):
            self.log("saved to flash" if res.ok else "save FAILED: " + P.status_text(res.status), GREEN if res.ok else RED)
        self._req(P.SAVE, b"", cb)

    def reboot(self, dfu):
        what = "Reboot to DFU" if dfu else "Reboot"
        text = ("Reset into the ST bootloader (USB DFU)?\n\nThe driver leaves this port; flash it with STM32CubeProgrammer "
                "(USB). A power cycle or a reset returns to the firmware." if dfu else
                "Reset the driver?\n\nRAM edits not saved to flash are lost.")
        r = QtWidgets.QMessageBox.question(self, what, text + "\n(Refused while the motor runs.)",
                                           QtWidgets.QMessageBox.StandardButton.Ok | QtWidgets.QMessageBox.StandardButton.Cancel)
        if r != QtWidgets.QMessageBox.StandardButton.Ok:
            return

        def cb(res):
            if not res.ok:
                self.log(f"{what}: " + P.status_text(res.status), RED)
                return
            self.log("rebooting into the bootloader: port closed" if dfu else "rebooting: port closed, reconnecting when it is back",
                     GREEN)
            QtCore.QTimer.singleShot(0, lambda: self.main.disconnect(reconnect=not dfu))  # the device is about to drop off the bus
        self._req(P.REBOOT, P.p_reboot(dfu), cb)

    def export_json(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Export parameters", time.strftime("params_%Y%m%d_%H%M%S.json"),
                                                        "JSON (*.json)")
        if not path:
            return

        def write(ok):
            if not ok:
                self.log("export aborted: not every value could be read", RED)
                return
            pv, h, parv, bid = getattr(self, "device", (0, 0, 0, 0))
            doc = {"format": "driverTCC-params", "param_version": parv, "build_id": f"{bid:08X}",
                   "saved": time.strftime("%Y-%m-%d %H:%M:%S"),
                   "values": {o.key: self.dev[o.key] for o in self.params() if o.key in self.dev},
                   "units": {o.key: o.json_unit for o in self.params()}}
            with open(path, "w", encoding="utf-8") as f:
                json.dump(doc, f, indent=1)
            self.log(f"exported {len(doc['values'])} values to {path}", GREEN)
        self.read_all(then=write)  # export what the device holds right now

    def import_json(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Import parameters", "", "JSON (*.json)")
        if not path:
            return
        try:
            with open(path, encoding="utf-8") as f:
                doc = json.load(f)
            vals = doc["values"]
        except (OSError, ValueError, KeyError, TypeError) as e:
            self.log(f"cannot read {path}: {e}", RED)
            return
        n, bad = 0, []
        for k, v in vals.items():
            o = O.BY_KEY.get(k)
            if o is None or not o.stored or o.ro or k in self.disabled:
                bad.append(f"{k} (unknown)")
                continue
            why = o.unit_changed(doc) or (o.check(v) if isinstance(v, (int, float)) else "not a number")
            if why:
                bad.append(f"{k} ({why})")
                continue
            if k not in self.dev or not same(o, v, self.dev[k]):
                self.edit[k] = v
                n += 1
            self.refresh_row(k)
        self.log(f"imported {path}: {n} value(s) differ from the device, now pending; 'Write changed' then "
                 f"'Save to flash'" + (f". skipped: {', '.join(bad)}" if bad else ""), ORANGE if bad else GREEN)

    # ------------------------------------------------------------------ calibration
    def _refresh_cal_buttons(self):
        flags = self.st.get("st_flags", self.dev.get("flags", 0)) or 0
        on = self.proto_ok and self.cal_running is None
        for key, which, label, flag, arg, prompt in STEPS:
            if flag is None:
                txt, col = "", None
            elif isinstance(flag, str):  # done when that parameter is non-zero
                ok = (self.dev.get(flag) or 0) != 0
                txt, col = ("done", GREEN) if ok else ("missing", ORANGE)
            else:
                txt, col = ("done", GREEN) if flags & flag else ("missing", ORANGE)
            if self.cal_running == which:
                txt, col = "running…", BLUE
            self.step_lbl[key].setText(f"<span style='color:{col}'>{txt}</span>" if col else txt)
            self.step_btn[key].setEnabled(on)

    def run_step(self, key):
        step = next(s for s in STEPS if s[0] == key)
        _, which, label, flag, arg, prompt = step
        if prompt:
            r = QtWidgets.QMessageBox.warning(self, label, prompt + "\n\nStart?",
                                              QtWidgets.QMessageBox.StandardButton.Ok | QtWidgets.QMessageBox.StandardButton.Cancel)
            if r != QtWidgets.QMessageBox.StandardButton.Ok:
                return
        a = self.step_arg[key].value() if arg else 0.0
        self.log(f"> {key}: {label}" + ((f" at {a:.0f} pp" if arg == "pp" else f" at {a:.2f} A") if a else ""), BLUE)

        def cb(res):
            if res.ok:
                self.cal_running = which
                self.busy_seen = False
                self.cal_state.setText(f"<span style='color:{BLUE}'>running: {label}</span>")
            else:
                self.log(f"{label}: FAILED: {P.status_text(res.status)}", RED)
            self._refresh_cal_buttons()
        self._req(P.CAL, P.p_cal(which, a), cb)

    def abort(self):
        self._req(P.ABORT, b"", lambda r: None, urgent=True)

    def _tick_countdown(self):
        if not self.countdown:
            self.cd_timer.stop()
            return
        end, text = self.countdown
        left = end - time.monotonic()
        if left <= 0:
            self.countdown = None
            self.cal_state.setText(f"<span style='color:{BLUE}'>running…</span>" if self.cal_running else "")
            return
        self.cal_state.setText(f"<span style='color:{RED}; font-size:15px; font-weight:bold'>{text}: {left:.1f} s</span>")

    # ------------------------------------------------------------------ run
    def run_cmd(self, mtype, payload, what):
        def cb(res):
            if res.ok:
                self.set_banner("")
                self.log(f"{what}: ok", GREEN)
            else:
                self.log(f"{what}: FAILED: {P.status_text(res.status)}", RED)
        self.log(f"> {what}", BLUE)
        self._req(mtype, payload, cb)

    def run_pos(self, t):
        t = t.strip()
        try:
            if abs(decimal.Decimal(t)) >= 2 ** 31:
                raise decimal.InvalidOperation
        except decimal.InvalidOperation:
            self.log(f"position: '{t}' is not a number of turns (|turns| < 2^31)", RED)
            return
        self.run_cmd(P.RUN_POS, P.p_run_pos(t), f"position {t} turns")

    def set_banner(self, html):
        for rp in self.run_panels:
            rp.banner.setText(html)

    def stop(self):
        """always available: jumps the queue; also ends a running calibration"""
        if not self.link:
            self.log("not connected", RED)
            return
        self.log("> STOP", BLUE)
        self.link.clear_queue()
        self._req(P.STOP, b"", lambda r: self.log("stopped" if r.ok else "STOP: " + P.status_text(r.status),
                                                  GREEN if r.ok else RED), urgent=True)

    # ------------------------------------------------------------------ events (GUI thread)
    def on_event(self, tok, code, vals):
        text = P.event_text(code, vals)
        if code == P.EV_DONE:
            ok = int(vals[0]) == P.ST_OK
            self.log(text, GREEN if ok else RED)
            self.cal_running = None
            self.countdown = None
            self.cal_state.setText("")
            self._refresh_cal_buttons()
            self.read_all()
            return
        if code == P.EV_STOPPED:
            self.log(text, RED)
            self.set_banner(f"<span style='color:{RED}; font-weight:bold'>{text}</span>")
            return
        if code == P.EV_RESUMED:
            self.log(text, ORANGE)
            self.set_banner(f"<span style='color:{ORANGE}; font-weight:bold'>{text}</span>")
            return
        if code in (P.EV_HOLD_SHAFT, P.EV_UNLOAD_SHAFT, P.EV_EEPROM_WAIT) and vals:
            label = {P.EV_HOLD_SHAFT: "HOLD THE SHAFT: q test starts in", P.EV_UNLOAD_SHAFT: "motor spins in",
                     P.EV_EEPROM_WAIT: "encoder EEPROM write, wait"}[code]
            self.countdown = (time.monotonic() + vals[0], label)
            self.cd_timer.start()
        if code == P.EV_HFI_FIT and len(vals) >= 5:
            self.log(text, GREEN if abs(vals[4]) >= 0.10 else ORANGE if abs(vals[4]) >= 0.05 else RED)
            return
        self.log(text, ORANGE if code in (P.EV_HOLD_SHAFT, P.EV_UNLOAD_SHAFT, P.EV_PP_BAD, P.EV_BW_CLAMPED,
                                          P.EV_RPM_LIMITED, P.EV_CUR_LIMITED) else None)

    # ------------------------------------------------------------------ status polling
    def poll(self):
        """4 Hz while the sidebar shows the status (every tab): status every tick, diagnostics every 4th.
        Each reply takes one telemetry slot (2500/s), so this costs well under 1 % of the frames"""
        link = self.link
        if not (link and self.proto_ok and self.status_box.isVisible()) or link.busy():
            self._show_link()
            return
        self.poll_n += 1
        keys = STATUS_KEYS + (DIAG_KEYS if self.poll_n % 4 == 1 else [])
        for k in keys:
            o = O.BY_KEY[k]
            if k in self.disabled:
                continue
            self._req(P.READ, P.p_read(o.index, o.sub), lambda r, o=o: self._got_status(o, r))
        self._show_link()

    def _got_status(self, o, res):
        if not res.ok:
            return
        self.st[o.key] = P.unpack_value(o.type, res.payload)
        if o.key == "st_flags":
            self._refresh_cal_buttons()
        if o.key == "busy" and self.cal_running is not None:
            # EV_DONE ends a calibration; if that event got lost, the busy flag going back to 0 does
            if self.st["busy"] == self.cal_running:
                self.busy_seen = True
            elif self.st["busy"] == 0 and getattr(self, "busy_seen", False):
                self.log("calibration ended (its result event was lost: see the parameters)", ORANGE)
                self.cal_running = None
                self.cal_state.setText("")
                self._refresh_cal_buttons()
                self.read_all()
        self._show_status()

    def _show_status(self):
        self._show_can()
        s = self.st
        g = s.get
        if "mode" in s:
            busy = g("busy", 0)
            btxt = f"  — calibrating ({P.CAL_NAMES.get(busy, busy)})" if busy else ""
            self.st_lbl["state"].setText(f"{MODES.get(g('mode'), g('mode'))}"
                                         + ((", coasting to a stop" if g("coasting") else f", {LOOPS.get(g('loop'), g('loop'))} loop")
                                            if g("mode") == 2 else "") + btxt)
        if "moe" in s:
            moe = g("moe")
            warn = [t for k, bad, t in (("nfault", 0, "nFAULT low"), ("brk", 1, "break latched"), ("ready", 0, "READY low"))
                    if g(k) == bad]
            self.st_lbl["bridge"].setText(
                ("<span style='color:%s'>enabled</span>" % GREEN if moe else "<span style='color:%s'>DISABLED</span>" % RED)
                + "".join("  <span style='color:%s'>%s</span>" % (RED, t) for t in warn))
        if "trip" in s:
            t = g("trip")
            self.st_lbl["trip"].setText(
                ("none" if not t else "<span style='color:%s'>%s%s</span>"
                 % (RED, TRIPS.get(t, t), ", bridge switched off — STOP resets" if g("trip_moe") else
                    (", coasting at zero current, resumes when the bus drops" if t == 1 else
                     ", coasting at zero current — STOP resets at standstill") if g("mode") == 2 else " — STOP resets"))
                + (f"<br>{g('ov_resumes')} overvoltage resumes" if g("ov_resumes") else ""))
        if all(k in s for k in ("coff_a_use", "coff_b_use", "coff_c_use", "coff_src")):
            self.st_lbl["coff"].setText(f"{g('coff_a_use'):+d} {g('coff_b_use'):+d} {g('coff_c_use'):+d} counts "
                                        f"({COFF_SRC.get(g('coff_src'), g('coff_src'))})")
        if "drv" in s:
            d = g("drv")
            self.st_lbl["drv"].setText("<span style='color:%s'>I2C error</span>" % RED if d & 0x100 else f"0x{d:02X}  {bits(d, DRV_BITS)}")
        if "vin" in s:
            self.st_lbl["vin"].setText(f"{g('vin'):.2f} V")
        if "t_state" in s:
            def deg(k):
                v = g(k)
                return "—" if v is None or v != v else f"{v:.1f} °C"
            ts = g("t_state")
            txt = f"board {deg('t_board')}, die {deg('t_die')}" + (f", motor {deg('t_motor')}" if self.dev.get("mtmax") else "")
            if ts:
                col = ORANGE if ts == 1 else RED
                what = {1: f"derating to {g('i_temp', 0):.2f} A", 2: "over temperature", 3: "sensor fault"}.get(ts, ts)
                txt += f"<br><span style='color:{col}'>{what}</span>"
            self.st_lbl["temp"].setText(txt)
        if "dirty" in s:
            dirty = g("dirty")
            self.st_lbl["flash"].setText(("<span style='color:%s'>unsaved changes</span>" % ORANGE if dirty else "saved")
                                         + f"  (save #{g('seq', 0)})")
            self.dirty_lbl.setText("<span style='color:%s'>● device RAM differs from flash: 'Save to flash' keeps it</span>" % ORANGE
                                   if dirty else "")
        if "loop_cyc" in s:
            mx = g("loop_cyc_max", 0) / CPU_MHZ
            self.st_lbl["loop"].setText(f"{g('loop_cyc') / CPU_MHZ:.1f} µs, max "
                                        + (f"<span style='color:{RED}'>{mx:.1f} µs</span>" if mx >= 19.0 else f"{mx:.1f} µs"))
        if "enc_samples" in s:
            es = g("enc_status", 0)
            crc = "" if g("enc_crc_check", 1) else "<br><span style='color:%s'>CRC check off</span>" % RED

            def n(k):
                return f"<span style='color:{RED}'>{g(k)}</span>" if g(k) else "0"
            self.st_lbl["enc"].setText(f"{g('enc_samples'):,} reads<br>CRC {n('enc_crc')}, late {n('enc_late')}, "
                                       f"stalls {n('enc_stalls')}<br>status {bits(es, ENC_BITS)}{crc}"
                                       + (f"<br>Hall state {g('hall_state')}, bad {n('hall_bad')}, HFI re-seeds {n('hyb_reseeds')}"
                                          if "hall_state" in s else ""))

    def _show_can(self):
        s, g = self.st, self.st.get
        if "can_nmt" not in s:
            return
        nmt = g("can_nmt")
        color = GREEN if nmt == 5 else ORANGE if nmt == 127 else RED
        err = g("can_err", 0)
        pend = ""
        if "can_id" in self.dev and (self.dev["can_id"], self.dev.get("can_kbit")) != (g("can_id_act"), g("can_kbit_act")):
            pend = f"<br><span style='color:{ORANGE}'>pending: node {self.dev['can_id']} @ {self.dev.get('can_kbit')} kbit/s (Apply)</span>"
        self.st_lbl["can"].setText(
            f"node {g('can_id_act')} @ {g('can_kbit_act')} kbit/s, <span style='color:{color}'>"
            f"{O.NMT_STATES.get(nmt, nmt)}</span><br>RX {g('can_rx', 0):,}, TX {g('can_tx', 0):,}, overruns "
            + (f"<span style='color:{RED}'>{g('can_lost')}</span>" if g("can_lost") else "0")
            + (f"<br><span style='color:{RED if err & 0x0006 else ORANGE}'>{bits(err, CAN_BITS)}</span>" if err else "") + pend)
        if "fsa" in s:
            fsa = g("fsa")
            col = RED if fsa >= 6 else GREEN if fsa == 4 else None
            txt = O.FSA_STATES.get(fsa, fsa)
            txt = f"<span style='color:{col}'>{txt}</span>" if col else txt
            if g("can_owner"):
                txt += " — <b>CAN has the motor</b>"
            if g("fsa_fault"):
                txt += f"<br><span style='color:{RED}'>fault: {P.status_text(g('fsa_fault'))}</span>"
            if g("fsa_refused"):
                txt += f"<br><span style='color:{ORANGE}'>refused: {P.status_text(g('fsa_refused'))}</span>"
            self.st_lbl["can"].setText(self.st_lbl["can"].text() + "<br>CiA 402: " + txt)

    def _show_link(self):
        link = self.link
        if not link:
            self.st_lbl["link"].setText("-")
            return
        self.st_lbl["link"].setText(f"{link.timeouts} timeouts, {link.unmatched} unmatched<br>"
                                    f"device RX errors {self.st.get('proto_err', '?')}")

    def can_apply(self):
        o = O.BY_KEY["can_apply"]

        def done(res):
            if res.ok:
                self.log("CAN communication reset requested: node ID / bit rate in use from now on", GREEN)
            else:
                self.log(f"CAN apply refused: {P.status_text(res.status)}", RED)
        self._req(P.WRITE, P.p_write(o.index, o.sub, o.type, 1), done)

    def clear_counters(self):
        for k in CLEAR_KEYS:
            o = O.BY_KEY[k]
            self._req(P.WRITE, P.p_write(o.index, o.sub, o.type, 0))


class RunPanel(QtWidgets.QGroupBox):
    """Current / speed / position setpoints, Home and STOP. Enter in a field sends that command.
    Lives in the plotter's left sidebar, next to every tab; drives ConfigTab (run_panels allows more)."""

    def __init__(self, tab):
        super().__init__("Run")
        self.tab = tab
        tab.run_panels.append(self)
        grid = QtWidgets.QGridLayout(self)
        self.id_sp = QtWidgets.QDoubleSpinBox(decimals=3, minimum=-50, maximum=50, singleStep=0.1, prefix="id ", suffix=" A")
        self.iq_sp = QtWidgets.QDoubleSpinBox(decimals=3, minimum=-50, maximum=50, singleStep=0.1, prefix="iq ", suffix=" A")
        self.rpm_sp = QtWidgets.QDoubleSpinBox(decimals=1, minimum=-60000, maximum=60000, singleStep=100, suffix=" rpm")
        self.turns_ed = QtWidgets.QLineEdit("0")
        self.turns_ed.setToolTip("Turns from home, exact to 1e-9 turn (sent as Q32.32). Enter sends")
        for w in (self.id_sp, self.iq_sp, self.rpm_sp):
            w.setLocale(C_LOCALE)
            w.setToolTip("Enter sends")
            w.setKeyboardTracking(False)
        cur = QtWidgets.QPushButton("Current")
        vel = QtWidgets.QPushButton("Speed")
        pos = QtWidgets.QPushButton("Position")
        cur.clicked.connect(self.send_cur)
        vel.clicked.connect(self.send_vel)
        pos.clicked.connect(self.send_pos)
        # Enter: the spin boxes interpret the typed text first, then their line edit emits returnPressed
        self.id_sp.findChild(QtWidgets.QLineEdit).returnPressed.connect(self.send_cur)
        self.iq_sp.findChild(QtWidgets.QLineEdit).returnPressed.connect(self.send_cur)
        self.rpm_sp.findChild(QtWidgets.QLineEdit).returnPressed.connect(self.send_vel)
        self.turns_ed.returnPressed.connect(self.send_pos)
        self.vlim_lbl = QtWidgets.QLabel("vlim ?")
        grid.addWidget(self.id_sp, 0, 0)
        grid.addWidget(self.iq_sp, 0, 1)
        grid.addWidget(cur, 0, 2)
        grid.addWidget(self.rpm_sp, 1, 0)
        grid.addWidget(self.vlim_lbl, 1, 1)
        grid.addWidget(vel, 1, 2)
        grid.addWidget(self.turns_ed, 2, 0)
        grid.addWidget(QtWidgets.QLabel("turns"), 2, 1)
        grid.addWidget(pos, 2, 2)
        home = QtWidgets.QPushButton("Home")
        home.setToolTip("Make the present position 0 (stops first)")
        home.clicked.connect(lambda: tab.run_cmd(P.HOME, b"", "home"))
        stop = QtWidgets.QPushButton("STOP")
        stop.setObjectName("stop")
        stop.setMinimumHeight(40)
        stop.setToolTip("Ramped stop; also ends a running calibration")
        stop.clicked.connect(tab.stop)
        grid.addWidget(home, 3, 0)
        grid.addWidget(stop, 3, 1, 1, 2)
        self.banner = QtWidgets.QLabel("")
        self.banner.setWordWrap(True)
        grid.addWidget(self.banner, 4, 0, 1, 3)
        self.widgets = [self.id_sp, self.iq_sp, self.rpm_sp, self.turns_ed, cur, vel, pos, home]
        self.set_enabled(False)

    def set_enabled(self, on):
        for w in self.widgets:
            w.setEnabled(on)

    def send_cur(self):
        i_d, i_q = self.id_sp.value(), self.iq_sp.value()
        self.tab.run_cmd(P.RUN_CUR, P.p_run_cur(i_d, i_q), f"current id {i_d:.3f} A iq {i_q:.3f} A")

    def send_vel(self):
        rpm = self.rpm_sp.value()
        self.tab.run_cmd(P.RUN_VEL, P.p_run_vel(rpm), f"speed {rpm:.1f} rpm")

    def send_pos(self):
        self.tab.run_pos(self.turns_ed.text())
