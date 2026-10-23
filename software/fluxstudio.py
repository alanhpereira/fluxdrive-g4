#!/usr/bin/env python3
"""
Telemetry viewer and config GUI for the STSPIN32G4 FOC firmware.

Telemetry frame (72 bytes, little-endian):
    0xA5 0x5A | seq (uint16) | 16 x float32 | sum (uint32)
    sum = rotate-left-by-1-and-add over the first 17 words, seed 0x5A5AA5A5 (frame_sum() in motorCtrl.c,
    protocol.telem_ok() here)
Config messages (A5 5C ..., protocol.py) share the link; the Config tab (config_tab.py) sends all commands.
The CANopen tab (canopen_tab.py, co_link.py) is the CAN master side, over its own adapter (python-can).

Usage:
    pip install pyserial numpy pyqtgraph PyQt6        (PyQt5 / PySide6 also work)
    python fluxstudio.py                 pick the port in the window
    python fluxstudio.py COM5            connect on start
    pythonw fluxstudio.py                same, without a console window (Windows)

Load a recording later with:
    dt = np.dtype([("sync", "u1", 2), ("seq", "<u2"), ("v", "<f4", 16), ("sum", "<u4")])
    frames = np.fromfile("run1.bin", dtype=dt)
"""
import argparse
import collections
import struct
import threading
import time

import numpy as np
import pyqtgraph as pg
import serial
import serial.tools.list_ports
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

import protocol as P
import serial_fix  # noqa: F401  (pyserial on Windows: spurious ReadFile/WriteFile errors, see the module)
from canopen_tab import CanopenTab
from config_tab import ConfigTab, RunPanel

NF = 16  # floats per frame

# One entry per plot: (title, [(float index, label, color), ...], shown by default)
PLOTS = [
    ("Angle (deg)", [(0, "theta", "#e0e0e0")], False),
    ("Bus voltage (V)", [(1, "Vin", "#f0c040")], False),
    # angsrc 0-2: HFI - encoder; 3: Hall - encoder; 4: HFI - Hall
    ("HFI / Hall error (deg elec)", [(2, "err", "#ff5555")], False),
    ("Id / Iq (A)", [(5, "Id", "#ff55ff"), (6, "Iq", "#f0f0f0")], True),
    ("Speed (rpm)", [(7, "rpm", "#f0c040"), (8, "ref", "#ff5555")], True),
    ("Position (turns)", [(9, "pos", "#40d0e0"), (10, "ref", "#ff5555")], True),
    ("Observer - encoder (deg elec)", [(11, "err", "#ff9040")], False),
    ("Observer flux / blend", [(12, "|flux|/lambda", "#40d0e0"), (13, "weight", "#f0c040")], False),
    # v[15]: the encoder PLL, or the Halls in angsrc 3 and 4
    ("Speed sources (rpm)", [(7, "used", "#f0f0f0"), (14, "observer", "#ff9040"), (15, "encoder/Hall", "#40d0e0")], False),
    ("HFI speed (rpm)", [(3, "HFI", "#55dd55"), (15, "encoder/Hall", "#40d0e0")], False),
    ("HFI signal (A)", [(4, "HF step", "#55aaff")], False),
]


# ---------------------------------------------------------------------------------------------- serial
USB_VID_PID = (0x1209, 0x0001)  # the driver's own ids (USER CODE PRIVATE_DEFINES in usbd_desc.c), not ST's 0483:5740
RECONNECT_S = 10.0  # how long a lost port is retried (a reset drive is back on USB after 1-2 s)
class Receiver(threading.Thread):
    """Reads the port, separates telemetry frames and config messages (anything else is dropped),
    keeps a ring buffer of frames."""

    def __init__(self, ser, capacity):
        super().__init__(daemon=True)
        self.ser = ser
        self.lock = threading.Lock()
        self.running = True
        self.error = None
        self.frames = 0
        self.dropped = 0
        self.split = P.Splitter()
        self.on_message = None  # protocol.Link.on_message(type, tok, payload), called from this thread
        self.log = None
        self.last_raw = None
        self.useq = 0
        self.resize(capacity)

    def resize(self, capacity):
        with self.lock:
            self.cap = capacity
            self.data = np.full((capacity, NF), np.nan, np.float32)
            self.seqs = np.zeros(capacity, np.int64)
            self.head = 0

    @property
    def resyncs(self):
        return self.split.resyncs

    def run(self):
        while self.running:
            try:
                n = self.ser.in_waiting
                chunk = self.ser.read(n if n else 1)
            except (serial.SerialException, OSError) as e:
                self.error = str(e)
                break
            if not chunk:
                continue
            for kind, d in self.split.feed(chunk):
                if kind == "telem":
                    self._store(d)
                elif kind == "msg" and self.on_message:
                    self.on_message(*d)
        if self.log:
            self.log.close()
            self.log = None

    def _store(self, frame):
        seq = struct.unpack_from("<H", frame, 2)[0]
        vals = np.frombuffer(frame, "<f4", NF, 4)
        if self.last_raw is None:
            self.useq = seq
        else:
            d = (seq - self.last_raw) & 0xFFFF
            if d > 0x8000:  # counter went backwards: MCU reset
                d = 1
            elif d > 1:
                self.dropped += d - 1
            self.useq += d
        self.last_raw = seq
        log = self.log
        if log:
            try:
                log.write(frame)
            except (ValueError, OSError):  # closed by Stop rec meanwhile (or disk full): must not end this thread
                pass
        with self.lock:
            k = self.head % self.cap
            self.data[k] = vals
            self.seqs[k] = self.useq
            self.head += 1
            self.frames += 1

    def snapshot(self):
        with self.lock:
            if self.head <= self.cap:
                return self.data[: self.head].copy(), self.seqs[: self.head].copy()
            k = self.head % self.cap
            return np.roll(self.data, -k, axis=0), np.roll(self.seqs, -k)


class Sender(threading.Thread):
    """The only writer of the port: whole protocol frames, one at a time."""

    def __init__(self, ser):
        super().__init__(daemon=True)
        self.ser = ser
        self.q = collections.deque()
        self.ev = threading.Event()
        self.running = True
        self.error = None

    def send_raw(self, frame):
        self.q.append(bytes(frame))
        self.ev.set()

    def run(self):
        while self.running:
            self.ev.wait(0.1)
            self.ev.clear()
            while self.q and self.running:
                try:
                    self.ser.write(self.q.popleft())
                except (serial.SerialException, OSError) as e:
                    self.error = str(e)  # update_view() reconnects: without a sender every request would time out
                    return


# ---------------------------------------------------------------------------------------------- widgets
class Main(QtWidgets.QMainWindow):
    def __init__(self, port=None):
        super().__init__()
        self.setWindowTitle("Flux Studio")
        self.settings = QtCore.QSettings("driverTCC", "plotter")
        self.ser = self.rx = self.tx = self.link = None
        self.reconnect = None  # (port, baud, give-up time, next try) while a lost port is retried
        self.config = ConfigTab(self)
        self.canopen = CanopenTab(self)
        self.paused = False
        self.curves = []
        self.last_frames, self.last_t = 0, time.monotonic()

        # ---- side panel
        side = QtWidgets.QWidget()
        sl = QtWidgets.QVBoxLayout(side)

        g = QtWidgets.QGroupBox("Connection")
        f = QtWidgets.QFormLayout(g)
        self.port = QtWidgets.QComboBox()
        self.port.setMinimumWidth(150)
        refresh = QtWidgets.QToolButton(text="⟳")
        refresh.setToolTip("Rescan ports")
        refresh.clicked.connect(self.scan_ports)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.port, 1)
        row.addWidget(refresh)
        f.addRow("Port", row)
        self.baud = QtWidgets.QComboBox(editable=True)
        self.baud.addItems(["2000000", "1000000", "921600", "460800", "115200"])
        self.baud.setToolTip("USART1 only: the USB port (default) ignores it")
        f.addRow("Baud", self.baud)
        self.conn_btn = QtWidgets.QPushButton("Connect")
        self.conn_btn.clicked.connect(self.toggle_connection)
        f.addRow(self.conn_btn)
        sl.addWidget(g)
        sl.addWidget(RunPanel(self.config))  # Connection, Run, Status and Statistics stay visible on every tab
        sl.addWidget(self.config.status_box)
        sl.addWidget(self.config.stats_box)

        g = self.display_box = QtWidgets.QGroupBox("Display")
        f = QtWidgets.QFormLayout(g)
        self.window_s = QtWidgets.QDoubleSpinBox(decimals=1, minimum=0.1, maximum=60.0, singleStep=0.5, suffix=" s")
        self.loop_hz = QtWidgets.QDoubleSpinBox(decimals=0, minimum=100, maximum=200000, singleStep=1000, suffix=" Hz")
        self.loop_hz.setToolTip("Control loop rate (CTRL_FS_HZ): sets the time axis")
        self.cpf = QtWidgets.QSpinBox(minimum=1, maximum=1000)
        self.cpf.setToolTip("Control cycles between telemetry frames")
        self.refresh = QtWidgets.QSpinBox(minimum=5, maximum=200, singleStep=5, suffix=" Hz")
        for w in (self.window_s, self.loop_hz, self.cpf, self.refresh):
            w.setLocale(QtCore.QLocale.c())
        self.refresh.setToolTip("Plot redraw rate. Higher is smoother but costs CPU; data is never lost either way")
        f.addRow("Window", self.window_s)
        f.addRow("Loop rate", self.loop_hz)
        f.addRow("Cycles/frame", self.cpf)
        f.addRow("Refresh", self.refresh)
        self.pause_btn = QtWidgets.QPushButton("Pause", checkable=True)
        self.pause_btn.toggled.connect(lambda on: setattr(self, "paused", on))
        self.rec_btn = QtWidgets.QPushButton("Record…", checkable=True)
        self.rec_btn.toggled.connect(self.toggle_record)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.pause_btn)
        row.addWidget(self.rec_btn)
        f.addRow(row)
        auto = QtWidgets.QPushButton("Auto-scale")
        auto.setToolTip("Back to the full window with automatic vertical scale (or double-click a plot)")
        auto.clicked.connect(self.reset_view)
        f.addRow(auto)
        sl.addWidget(g)

        g = self.plots_box = QtWidgets.QGroupBox("Plots")
        v = QtWidgets.QVBoxLayout(g)
        self.plot_checks = []
        for title, _, _ in PLOTS:
            cb = QtWidgets.QCheckBox(title)
            cb.toggled.connect(self.build_plots)
            v.addWidget(cb)
            self.plot_checks.append(cb)
        sl.addWidget(g)
        sl.addStretch(1)

        # ---- plots
        self.glw = pg.GraphicsLayoutWidget()
        self.glw.scene().sigMouseClicked.connect(lambda ev: ev.double() and self.reset_view())
        scroll = QtWidgets.QScrollArea(widgetResizable=True)
        scroll.setWidget(side)
        scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        scroll.setMinimumWidth(side.minimumSizeHint().width() + 24)
        self.tabs = QtWidgets.QTabWidget()  # USB pages: Scope and Config
        self.tabs.addTab(self.glw, "Scope")
        self.tabs.addTab(self.config, "Config")
        self.tabs.currentChanged.connect(self.tab_changed)
        hs = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)  # USB sidebar left of Scope/Config, whichever is shown
        hs.addWidget(scroll)
        hs.addWidget(self.tabs)
        hs.setStretchFactor(1, 1)
        # both pages' sidebars get the same width: the wider minimum, and dragging one splitter moves the other
        w = max(scroll.minimumWidth(), self.canopen.side_scroll.minimumWidth())
        for sc, sp in ((scroll, hs), (self.canopen.side_scroll, self.canopen.split)):
            sc.setMinimumWidth(w)
            sp.setSizes([w, 10000])
        self.splits = (hs, self.canopen.split)
        self.top = QtWidgets.QTabWidget()  # one page per link: USB (config protocol + telemetry) and CANopen
        self.top.setDocumentMode(True)
        self.top.addTab(hs, "USB")
        self.top.addTab(self.canopen, "CANopen")
        self.top.currentChanged.connect(self.page_changed)
        self.setCentralWidget(self.top)

        self.status = QtWidgets.QLabel()
        self.statusBar().addPermanentWidget(self.status, 1)

        # ---- settings
        s = self.settings
        self.window_s.setValue(float(s.value("window", 2.0)))
        self.loop_hz.setValue(float(s.value("loop_hz", 25000.0)))
        self.cpf.setValue(int(s.value("cpf", 10)))
        self.refresh.setValue(int(s.value("refresh", 60)))
        self.baud.setCurrentText(str(s.value("baud", "2000000")))
        shown = s.value("plots", None)
        for cb, (title, _, default) in zip(self.plot_checks, PLOTS):
            cb.blockSignals(True)
            cb.setChecked(title in shown if isinstance(shown, list) else default)
            cb.blockSignals(False)
        geo = s.value("geometry")
        if geo is not None:
            self.restoreGeometry(geo)
        else:
            self.resize(1400, 900)
        self.tabs.setCurrentIndex(min(int(s.value("tab", 0)), self.tabs.count() - 1))
        self.top.setCurrentIndex(int(s.value("top_tab", 0)))
        self.tab_changed(self.tabs.currentIndex())
        for w in (self.window_s, self.loop_hz, self.cpf):
            w.valueChanged.connect(self.resize_buffer)

        self.scan_ports(prefer=port or s.value("port"))
        self.build_plots()
        self.set_connected(False)

        self.timer = QtCore.QTimer(timerType=QtCore.Qt.TimerType.PreciseTimer)
        self.timer.timeout.connect(self.update_view)
        self.timer.start(int(1000 / self.refresh.value()))
        self.refresh.valueChanged.connect(lambda hz: self.timer.setInterval(int(1000 / hz)))
        if port:
            self.toggle_connection()

    # ------------------------------------------------------------------ helpers
    def tab_changed(self, i):
        """Display and Plots only matter for the Scope tab"""
        scope = self.tabs.widget(i) is self.glw
        self.display_box.setVisible(scope)
        self.plots_box.setVisible(scope)

    def page_changed(self, i):
        """The page now shown takes the sidebar width of the one left (a hidden splitter has no valid sizes)"""
        new, old = self.splits[i], self.splits[1 - i]
        new.setSizes([old.sizes()[0], max(1, sum(old.sizes()) - old.sizes()[0])])

    def frame_dt(self):
        return self.cpf.value() / self.loop_hz.value()

    def capacity(self):
        return max(10, int(self.window_s.value() / self.frame_dt()))

    def scan_ports(self, prefer=None):
        cur = prefer or self.port.currentData()
        self.port.clear()
        usb = None
        for p in sorted(serial.tools.list_ports.comports(), key=lambda p: p.device):
            is_drv = (p.vid, p.pid) == USB_VID_PID
            self.port.addItem(f"{p.device}  {'[driver USB]  ' if is_drv else ''}{p.description}", p.device)
            if is_drv and usb is None:
                usb = p.device
        i = self.port.findData(cur)
        if i < 0 and usb:  # the remembered port is gone: take the driver's USB CDC port
            i = self.port.findData(usb)
        if i >= 0:
            self.port.setCurrentIndex(i)

    def say(self, text, color=None):
        """to the log in the Config tab"""
        self.config.log(text, color)

    # ------------------------------------------------------------------ plots
    def build_plots(self):
        self.glw.clear()
        self.curves = []
        self.plot_items = []
        first = None
        row = 0
        for cb, (title, chans, _) in zip(self.plot_checks, PLOTS):
            if not cb.isChecked():
                continue
            p = self.glw.addPlot(row=row, col=0, title=title)
            row += 1
            p.showGrid(x=True, y=True, alpha=0.25)
            p.addLegend(offset=(10, 5))
            p.setClipToView(True)
            p.setDownsampling(auto=True, mode="peak")
            p.setXRange(-self.window_s.value(), 0, padding=0)
            p.enableAutoRange(axis="x", enable=False)
            p.setAutoVisible(y=True)  # vertical auto-scale fits only what is visible in time
            self.plot_items.append(p)
            if first is None:
                first = p
            else:
                p.setXLink(first)
            for ch, name, color in chans:
                self.curves.append((ch, p.plot(pen=pg.mkPen(color, width=1), name=name)))
        if first is not None:
            first.setLabel("bottom", "time", "s")
        self.settings.setValue("plots", [cb.text() for cb in self.plot_checks if cb.isChecked()])

    def reset_view(self):
        """undo zoom/pan: full time window, vertical auto-scale on every plot"""
        for p in self.plot_items:
            p.setXRange(-self.window_s.value(), 0, padding=0)
            p.enableAutoRange(axis="y", enable=True)

    def resize_buffer(self):
        if self.rx:
            self.rx.resize(self.capacity())
        self.build_plots()

    def update_view(self):
        if self.reconnect:
            self.try_reconnect()
        rx = self.rx
        if rx:
            link = self.link
            if link:
                link.drain()  # results and events in arrival order (events -> config.on_event)
            err = rx.error or (self.tx and self.tx.error)
            if err:
                self.say(f"connection lost: {err}; waiting {RECONNECT_S:.0f} s for the port to come back", "#ff6060")
                self.disconnect(reconnect=True)
                return
            if not self.paused and self.glw.isVisible():  # hidden curves need no redraw (it costs the CAN threads GIL time)
                d, s = rx.snapshot()
                if len(s) >= 2:
                    d = np.where(np.isfinite(d), d, np.nan)  # NaN/Inf must not wreck the axes
                    t = (s - s[-1]) * self.frame_dt()
                    for ch, curve in self.curves:
                        curve.setData(t, d[:, ch], connect="finite")
            now = time.monotonic()
            if now - self.last_t >= 0.5:
                fps = (rx.frames - self.last_frames) / (now - self.last_t)
                self.last_frames, self.last_t = rx.frames, now
                rec = "   ● REC" if rx.log else ""
                self.status.setText(
                    f"{self.ser.port} connected   {fps:,.0f} frames/s   frames {rx.frames:,}   "
                    f"dropped {rx.dropped:,}   resyncs {rx.resyncs:,}{rec}"
                    + (f"   link timeouts {self.link.timeouts}" if self.link and self.link.timeouts else ""))

    # ------------------------------------------------------------------ connection
    def set_connected(self, on):
        self.conn_btn.setText("Disconnect" if on else "Connect")
        self.port.setEnabled(not on)
        self.baud.setEnabled(not on)
        self.rec_btn.setEnabled(on)
        if not on:
            self.status.setText("not connected")

    def toggle_connection(self):
        if self.ser or self.reconnect:
            self.disconnect()
            return
        dev = self.port.currentData()
        if not dev:
            self.say("no port selected", "#ff6060")
            return
        try:
            baud = int(self.baud.currentText())
            self.open_port(dev, baud)
        except (ValueError, serial.SerialException) as e:
            self.say(f"cannot open {dev}: {e}", "#ff6060")

    def try_reconnect(self):
        """the USB CDC port goes away while the drive resets (reset node, Reboot): reopen it when it is back"""
        dev, baud, until, next_t = self.reconnect
        now = time.monotonic()
        if now < next_t:
            return
        try:
            self.open_port(dev, baud)
        except serial.SerialException:
            if now >= until:
                self.reconnect = None
                self.set_connected(False)
                self.say(f"{dev} did not come back: not connected", "#ff6060")
            else:
                self.reconnect = (dev, baud, until, now + 0.3)

    def open_port(self, dev, baud):
        """raises serial.SerialException when the port cannot be opened"""
        self.ser = serial.Serial(dev, baud, timeout=0.01)
        self.reconnect = None
        self.settings.setValue("port", dev)
        self.settings.setValue("baud", str(baud))
        self.rx = Receiver(self.ser, self.capacity())
        self.tx = Sender(self.ser)
        self.link = P.Link(self.tx.send_raw)
        self.link.on_event = self.config.on_event
        self.rx.on_message = self.link.on_message
        self.rx.start()
        self.tx.start()
        self.link.start()
        self.last_frames, self.last_t = 0, time.monotonic()
        self.set_connected(True)
        self.say(f"connected to {dev} @ {baud}", "#60c060")
        self.config.on_connect()

    def disconnect(self, reconnect=False):
        """reconnect: keep trying to reopen the same port for RECONNECT_S (the drive is resetting)"""
        dev = self.ser.port if self.ser else None
        baud = self.ser.baudrate if self.ser else None
        if self.rec_btn.isChecked():
            self.rec_btn.setChecked(False)
        if self.link:
            self.link.stop()
        for th in (self.rx, self.tx):
            if th:
                th.running = False
        if self.tx:
            self.tx.ev.set()
        if self.rx:
            self.rx.join(0.5)
        if self.ser:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
        self.ser = self.rx = self.tx = self.link = None
        self.reconnect = None
        if reconnect and dev:
            now = time.monotonic()
            self.reconnect = (dev, baud, now + RECONNECT_S, now + 0.3)
        self.set_connected(False)
        if self.reconnect:
            self.conn_btn.setText("Cancel")
            self.status.setText(f"waiting for {dev}…")
        self.config.on_disconnect()

    def toggle_record(self, on):
        if not self.rx:
            return
        if on:
            path, _ = QtWidgets.QFileDialog.getSaveFileName(
                self, "Record raw frames", time.strftime("run_%Y%m%d_%H%M%S.bin"), "Frames (*.bin)")
            if not path:
                self.rec_btn.blockSignals(True)
                self.rec_btn.setChecked(False)
                self.rec_btn.blockSignals(False)
                return
            self.rx.log = open(path, "wb")
            self.rec_btn.setText("Stop rec")
            self.say(f"recording to {path}", "#60c060")
        else:
            log, self.rx.log = self.rx.log, None
            if log:
                log.close()
                self.say("recording stopped", "#60c060")
            self.rec_btn.setText("Record…")

    def closeEvent(self, e):
        s = self.settings
        s.setValue("window", self.window_s.value())
        s.setValue("loop_hz", self.loop_hz.value())
        s.setValue("cpf", self.cpf.value())
        s.setValue("refresh", self.refresh.value())
        s.setValue("geometry", self.saveGeometry())
        s.setValue("tab", self.tabs.currentIndex())
        s.setValue("top_tab", self.top.currentIndex())
        self.canopen.shutdown()  # disable voltage over CAN, then the PC heartbeat stops
        self.disconnect()
        super().closeEvent(e)


def dark_theme(app):
    app.setStyle("Fusion")
    p = QtGui.QPalette()
    C = QtGui.QPalette.ColorRole
    for role, col in [(C.Window, "#202124"), (C.WindowText, "#e8eaed"), (C.Base, "#17181a"),
                      (C.AlternateBase, "#26272b"), (C.Text, "#e8eaed"), (C.Button, "#2d2e31"),
                      (C.ButtonText, "#e8eaed"), (C.Highlight, "#3d6fd9"), (C.HighlightedText, "#ffffff"),
                      (C.ToolTipBase, "#2d2e31"), (C.ToolTipText, "#e8eaed"), (C.PlaceholderText, "#80868b")]:
        p.setColor(role, QtGui.QColor(col))
    app.setPalette(p)
    app.setStyleSheet("""
        QGroupBox { font-weight: bold; border: 1px solid #3c4043; border-radius: 6px; margin-top: 10px; padding: 8px 6px 6px 6px; }
        QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
        QPushButton { padding: 5px 10px; border-radius: 4px; border: 1px solid #3c4043; background: #2d2e31; }
        QPushButton:hover { background: #3a3b3f; }
        QPushButton:checked { background: #3d6fd9; border-color: #3d6fd9; }
        QPushButton#stop { background: #b3261e; border-color: #b3261e; font-weight: bold; font-size: 15px; }
        QPushButton#stop:hover { background: #d33a2f; }
        QPlainTextEdit, QLineEdit { border: 1px solid #3c4043; border-radius: 4px; }
    """)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", help="connect to this port on start")
    args = ap.parse_args()

    # useOpenGL: GPU drawing via Qt's own OpenGL widget (no PyOpenGL needed); painting is ~97% of the redraw cost
    pg.setConfigOptions(useOpenGL=True, antialias=False, background="#17181a", foreground="#c8c8c8")
    QtCore.QLocale.setDefault(QtCore.QLocale.c())  # '.' as decimal separator in every input, whatever the OS uses
    app = pg.mkQApp("Flux Studio")
    dark_theme(app)
    w = Main(args.port)
    w.show()
    pg.exec()


if __name__ == "__main__":
    main()
