#!/usr/bin/env python3
"""
Telemetry viewer and command console for the STSPIN32G4 FOC firmware.

Frame (72 bytes, little-endian):
    0xA5 0x5A | seq (uint16) | 16 x float32 | sum (uint32)
    sum = rotate-left-by-1-and-add over the first 17 words, seed 0x5A5AA5A5 (frame_sum() in motorCtrl.c)

Usage:
    pip install pyserial numpy pyqtgraph PyQt6        (PyQt5 / PySide6 also work)
    python plotter.py                 pick the port in the window
    python plotter.py COM5            connect on start
    pythonw plotter.py                same, without a console window (Windows)

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

HDR = b"\xA5\x5A"
FRAME = 72
NF = 16  # floats per frame

# One entry per plot: (title, [(float index, label, color), ...], shown by default)
PLOTS = [
    ("Angle (deg)", [(0, "theta", "#e0e0e0")], False),
    ("Bus voltage (V)", [(1, "Vin", "#f0c040")], False),
    ("Phase currents (A)", [(2, "Ia", "#ff5555"), (3, "Ib", "#55dd55"), (4, "Ic", "#55aaff")], False),
    ("Id / Iq (A)", [(5, "Id", "#ff55ff"), (6, "Iq", "#f0f0f0")], True),
    ("Speed (rpm)", [(7, "rpm", "#f0c040"), (8, "ref", "#ff5555")], True),
    ("Position (turns)", [(9, "pos", "#40d0e0"), (10, "ref", "#ff5555")], True),
    ("Observer - encoder (deg elec)", [(11, "err", "#ff9040")], False),
    ("Observer flux / blend", [(12, "|flux|/lambda", "#40d0e0"), (13, "weight", "#f0c040")], False),
    ("Speed sources (rpm)", [(7, "used", "#f0f0f0"), (14, "observer", "#ff9040"), (15, "encoder", "#40d0e0")], False),
]

QUICK = [("Print params", "p"), ("Help", "?")]


def frame_ok(frame):
    s = 0x5A5AA5A5
    for w in struct.unpack_from("<17I", frame):
        s = (((s << 1) | (s >> 31)) + w) & 0xFFFFFFFF
    return s == struct.unpack_from("<I", frame, 68)[0]


def printable(b):
    return bytes(c for c in b if c in (9, 10) or 32 <= c < 127).decode()


# ---------------------------------------------------------------------------------------------- serial
class Receiver(threading.Thread):
    """Reads the port, separates checksummed frames from text, keeps a ring buffer of frames."""

    def __init__(self, ser, capacity):
        super().__init__(daemon=True)
        self.ser = ser
        self.lock = threading.Lock()
        self.text = collections.deque()  # decoded text chunks for the console
        self.running = True
        self.error = None
        self.frames = 0
        self.dropped = 0
        self.resyncs = 0
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

    def run(self):
        buf = bytearray()
        txt = bytearray()
        while self.running:
            try:
                n = self.ser.in_waiting
                chunk = self.ser.read(n if n else 1)
            except (serial.SerialException, OSError) as e:
                self.error = str(e)
                break
            if not chunk:
                continue
            buf += chunk
            while True:
                i = buf.find(HDR)
                if i < 0:
                    keep = 1 if buf[-1:] == HDR[:1] else 0  # header may be split across reads
                    txt += buf[: len(buf) - keep]
                    del buf[: len(buf) - keep]
                    break
                if i:
                    txt += buf[:i]
                    del buf[:i]
                if len(buf) < FRAME:
                    break
                frame = bytes(buf[:FRAME])
                if frame_ok(frame):
                    del buf[:FRAME]
                    self._store(frame)
                else:
                    self.resyncs += 1  # header bytes inside text or a corrupted frame
                    txt += buf[:1]
                    del buf[:1]
            if txt:
                s = printable(txt)
                if s:
                    self.text.append(s)
                txt.clear()
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
            log.write(frame)
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
    """Writes command lines one byte per ms: the firmware's RX queue is small."""

    def __init__(self, ser):
        super().__init__(daemon=True)
        self.ser = ser
        self.q = collections.deque()
        self.ev = threading.Event()
        self.running = True

    def send(self, line):
        self.q.append(line)
        self.ev.set()

    def run(self):
        while self.running:
            self.ev.wait(0.1)
            self.ev.clear()
            while self.q and self.running:
                for ch in (self.q.popleft() + "\n").encode():
                    try:
                        self.ser.write(bytes([ch]))
                    except (serial.SerialException, OSError):
                        return
                    time.sleep(0.001)


# ---------------------------------------------------------------------------------------------- widgets
class CommandLine(QtWidgets.QLineEdit):
    """Line edit with up/down history."""

    submitted = QtCore.Signal(str)

    def __init__(self):
        super().__init__()
        self.hist, self.pos = [], 0
        self.setPlaceholderText("command, e.g.  v 300   a 0.5   s vkp 1   (Enter sends, Up/Down history)")
        self.returnPressed.connect(self._submit)

    def _submit(self):
        t = self.text().strip()
        if not t:
            return
        if not self.hist or self.hist[-1] != t:
            self.hist.append(t)
        self.pos = len(self.hist)
        self.clear()
        self.submitted.emit(t)

    def keyPressEvent(self, e):
        k = e.key()
        if k == QtCore.Qt.Key.Key_Up and self.hist:
            self.pos = max(0, self.pos - 1)
            self.setText(self.hist[self.pos])
        elif k == QtCore.Qt.Key.Key_Down and self.hist:
            self.pos = min(len(self.hist), self.pos + 1)
            self.setText(self.hist[self.pos] if self.pos < len(self.hist) else "")
        else:
            super().keyPressEvent(e)


class Main(QtWidgets.QMainWindow):
    def __init__(self, port=None):
        super().__init__()
        self.setWindowTitle("Driver TCC telemetry")
        self.settings = QtCore.QSettings("driverTCC", "plotter")
        self.ser = self.rx = self.tx = None
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
        f.addRow("Baud", self.baud)
        self.conn_btn = QtWidgets.QPushButton("Connect")
        self.conn_btn.clicked.connect(self.toggle_connection)
        f.addRow(self.conn_btn)
        sl.addWidget(g)

        g = QtWidgets.QGroupBox("Display")
        f = QtWidgets.QFormLayout(g)
        self.window_s = QtWidgets.QDoubleSpinBox(decimals=1, minimum=0.1, maximum=60.0, singleStep=0.5, suffix=" s")
        self.loop_hz = QtWidgets.QDoubleSpinBox(decimals=0, minimum=100, maximum=200000, singleStep=1000, suffix=" Hz")
        self.loop_hz.setToolTip("Control loop rate (CTRL_FS_HZ): sets the time axis")
        self.cpf = QtWidgets.QSpinBox(minimum=1, maximum=1000)
        self.cpf.setToolTip("Control cycles between telemetry frames")
        self.refresh = QtWidgets.QSpinBox(minimum=5, maximum=200, singleStep=5, suffix=" Hz")
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

        g = QtWidgets.QGroupBox("Plots")
        v = QtWidgets.QVBoxLayout(g)
        self.plot_checks = []
        for title, _, _ in PLOTS:
            cb = QtWidgets.QCheckBox(title)
            cb.toggled.connect(self.build_plots)
            v.addWidget(cb)
            self.plot_checks.append(cb)
        sl.addWidget(g)

        g = QtWidgets.QGroupBox("Commands")
        v = QtWidgets.QVBoxLayout(g)
        stop = QtWidgets.QPushButton("STOP  (o)")
        stop.setObjectName("stop")
        stop.setMinimumHeight(44)
        stop.clicked.connect(lambda: self.send("o"))
        v.addWidget(stop)
        row = QtWidgets.QHBoxLayout()
        for label, cmd in QUICK:
            b = QtWidgets.QPushButton(label)
            b.clicked.connect(lambda _=False, c=cmd: self.send(c))
            row.addWidget(b)
        v.addLayout(row)
        sl.addWidget(g)
        sl.addStretch(1)

        # ---- plots + console
        self.glw = pg.GraphicsLayoutWidget()
        self.glw.scene().sigMouseClicked.connect(lambda ev: ev.double() and self.reset_view())
        console = QtWidgets.QWidget()
        cl = QtWidgets.QVBoxLayout(console)
        cl.setContentsMargins(0, 0, 0, 0)
        self.log = QtWidgets.QPlainTextEdit(readOnly=True)
        self.log.setMaximumBlockCount(5000)
        mono = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont)
        self.log.setFont(mono)
        self.cmd = CommandLine()
        self.cmd.setFont(mono)
        self.cmd.submitted.connect(self.send)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.cmd, 1)
        clear = QtWidgets.QPushButton("Clear")
        clear.clicked.connect(self.log.clear)
        row.addWidget(clear)
        cl.addWidget(self.log, 1)
        cl.addLayout(row)

        self.vsplit = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
        self.vsplit.addWidget(self.glw)
        self.vsplit.addWidget(console)
        self.vsplit.setStretchFactor(0, 4)
        self.vsplit.setStretchFactor(1, 1)
        scroll = QtWidgets.QScrollArea(widgetResizable=True)
        scroll.setWidget(side)
        scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        scroll.setMinimumWidth(side.sizeHint().width() + 20)
        hs = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        hs.addWidget(scroll)
        hs.addWidget(self.vsplit)
        hs.setStretchFactor(1, 1)
        self.setCentralWidget(hs)

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
        split = s.value("vsplit")
        if split is not None:
            self.vsplit.restoreState(split)
        for w in (self.window_s, self.loop_hz, self.cpf):
            w.valueChanged.connect(self.resize_buffer)

        self.scan_ports(prefer=port or s.value("port"))
        self.build_plots()
        self.set_connected(False)
        self.cmd.setFocus()

        self.timer = QtCore.QTimer(timerType=QtCore.Qt.TimerType.PreciseTimer)
        self.timer.timeout.connect(self.update_view)
        self.timer.start(int(1000 / self.refresh.value()))
        self.refresh.valueChanged.connect(lambda hz: self.timer.setInterval(int(1000 / hz)))
        if port:
            self.toggle_connection()

    # ------------------------------------------------------------------ helpers
    def frame_dt(self):
        return self.cpf.value() / self.loop_hz.value()

    def capacity(self):
        return max(10, int(self.window_s.value() / self.frame_dt()))

    def scan_ports(self, prefer=None):
        cur = prefer or self.port.currentData()
        self.port.clear()
        for p in sorted(serial.tools.list_ports.comports(), key=lambda p: p.device):
            self.port.addItem(f"{p.device}  {p.description}", p.device)
        i = self.port.findData(cur)
        if i >= 0:
            self.port.setCurrentIndex(i)

    def say(self, text, color=None):
        if color:
            self.log.appendHtml(f'<span style="color:{color}">{text}</span>')
        else:
            self.log.moveCursor(QtGui.QTextCursor.MoveOperation.End)
            self.log.insertPlainText(text)
        self.log.moveCursor(QtGui.QTextCursor.MoveOperation.End)

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
        rx = self.rx
        if rx:
            while rx.text:
                self.say(rx.text.popleft())
            if rx.error:
                self.say(f"connection lost: {rx.error}", "#ff6060")
                self.disconnect()
                return
            if not self.paused:
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
                    f"dropped {rx.dropped:,}   resyncs {rx.resyncs:,}{rec}")

    # ------------------------------------------------------------------ connection
    def set_connected(self, on):
        self.conn_btn.setText("Disconnect" if on else "Connect")
        self.port.setEnabled(not on)
        self.baud.setEnabled(not on)
        self.rec_btn.setEnabled(on)
        if not on:
            self.status.setText("not connected")

    def toggle_connection(self):
        if self.ser:
            self.disconnect()
            return
        dev = self.port.currentData()
        if not dev:
            self.say("no port selected", "#ff6060")
            return
        try:
            baud = int(self.baud.currentText())
            self.ser = serial.Serial(dev, baud, timeout=0.01)
        except (ValueError, serial.SerialException) as e:
            self.say(f"cannot open {dev}: {e}", "#ff6060")
            self.ser = None
            return
        self.settings.setValue("port", dev)
        self.settings.setValue("baud", str(baud))
        self.rx = Receiver(self.ser, self.capacity())
        self.tx = Sender(self.ser)
        self.rx.start()
        self.tx.start()
        self.last_frames, self.last_t = 0, time.monotonic()
        self.set_connected(True)
        self.say(f"connected to {dev} @ {baud}", "#60c060")

    def disconnect(self):
        if self.rec_btn.isChecked():
            self.rec_btn.setChecked(False)
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
        self.ser = self.rx = self.tx = None
        self.set_connected(False)

    def send(self, line):
        if not self.tx:
            self.say("not connected", "#ff6060")
            return
        self.say(f"&gt; {line}", "#7fb2ff")
        self.tx.send(line)

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
        s.setValue("vsplit", self.vsplit.saveState())
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
    app = pg.mkQApp("Driver TCC telemetry")
    dark_theme(app)
    w = Main(args.port)
    w.show()
    pg.exec()


if __name__ == "__main__":
    main()
