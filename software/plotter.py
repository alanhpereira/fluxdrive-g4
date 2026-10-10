#!/usr/bin/env python3
"""
Live plot for the STSPIN32G4 telemetry stream.

Frame (72 bytes, little-endian):
    0xA5 0x5A | seq (uint16) | 17 x float32

Usage:
    pip install pyserial numpy pyqtgraph PyQt5      (PySide6 also works)
    python telemetry_plot.py COM5
    python telemetry_plot.py /dev/ttyACM0 --loop-hz 25000 --window 2 --log run1.bin

Load a --log file later with:
    dt = np.dtype([("sync", "u1", 2), ("seq", "<u2"), ("v", "<f4", 17)])
    frames = np.fromfile("run1.bin", dtype=dt)
"""
import argparse
import struct
import threading

import numpy as np
import pyqtgraph as pg
import serial
from pyqtgraph.Qt import QtCore

HDR = b"\xA5\x5A"
FRAME = 72
NF = 17                  # floats per frame
CYCLES_PER_FRAME = 10    # control cycles between frames

# One entry per plot: (title, [(float index, label, color), ...])
PLOTS = [
    ("Angle", [(0, "theta", "w")]),
    #("Bus voltage", [(1, "Vin", "y")]),
    ("Phase currents", [(2, "Ia", "r"), (3, "Ib", "g"), (4, "Ic", "c")]),
    ("Id / Iq", [(5, "Id", "m"), (6, "Iq", "w")]),
]


class Receiver(threading.Thread):
    """Reads the port, resyncs on the 2-byte header, keeps a ring buffer."""

    def __init__(self, port, baud, capacity, log_path=None):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, baud, timeout=0.01)
        self.cap = capacity
        self.data = np.full((capacity, NF), np.nan, np.float32)
        self.seqs = np.zeros(capacity, np.int64)
        self.head = 0                    # total frames stored
        self.frames = 0
        self.dropped = 0
        self.lock = threading.Lock()
        self.running = True
        self.last_raw = None
        self.useq = 0                    # unwrapped sequence number
        self.log = open(log_path, "wb") if log_path else None

    def run(self):
        buf = bytearray()
        while self.running:
            try:
                n = self.ser.in_waiting
                chunk = self.ser.read(n if n else 1)
            except serial.SerialException:
                break
            if not chunk:
                continue
            buf += chunk
            while True:
                i = buf.find(HDR)
                if i < 0:
                    del buf[:-1]         # keep 1 byte in case the header is split
                    break
                if i:
                    del buf[:i]
                if len(buf) < FRAME + 2:
                    break                # need the next header to confirm this frame
                if buf[FRAME:FRAME + 2] != HDR:
                    del buf[0]           # false sync, rescan
                    continue
                frame = bytes(buf[:FRAME])
                del buf[:FRAME]
                self._store(frame)
        if self.log:
            self.log.close()

    def _store(self, frame):
        seq = struct.unpack_from("<H", frame, 2)[0]
        vals = np.frombuffer(frame, "<f4", NF, 4)

        if self.last_raw is None:
            self.useq = seq
        else:
            d = (seq - self.last_raw) & 0xFFFF
            if d > 0x8000:               # counter went backwards: MCU reset
                d = 1
            elif d > 1:
                self.dropped += d - 1
            self.useq += d
        self.last_raw = seq

        if self.log:
            self.log.write(frame)
        with self.lock:
            k = self.head % self.cap
            self.data[k] = vals
            self.seqs[k] = self.useq
            self.head += 1
            self.frames += 1

    def snapshot(self):
        with self.lock:
            if self.head <= self.cap:
                d = self.data[: self.head].copy()
                s = self.seqs[: self.head].copy()
            else:
                k = self.head % self.cap
                d = np.roll(self.data, -k, axis=0)
                s = np.roll(self.seqs, -k)
            return d, s, self.dropped, self.frames


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--baud", type=int, default=2_000_000)
    ap.add_argument("--loop-hz", type=float, default=25000.0,
                    help="control loop rate (sets the time axis)")
    ap.add_argument("--window", type=float, default=2.0, help="seconds shown")
    ap.add_argument("--log", help="write raw valid frames to this file")
    args = ap.parse_args()

    frame_dt = CYCLES_PER_FRAME / args.loop_hz
    cap = max(10, int(args.window / frame_dt))

    rx = Receiver(args.port, args.baud, cap, args.log)
    rx.start()
    pg.setConfigOptions(useOpenGL=True, antialias=False)
    pg.mkQApp("Telemetry")
    win = pg.GraphicsLayoutWidget(title="Telemetry")
    win.resize(1100, 700)
    win.show()

    plots, curves = [], []
    for row, (title, chans) in enumerate(PLOTS):
        p = win.addPlot(row=row, col=0, title=title)
        p.showGrid(x=True, y=True)
        p.addLegend()
        p.setLabel("bottom", "time", "s")
        p.setClipToView(True)
        p.setDownsampling(auto=True, mode="peak") 
        if plots:
            p.setXLink(plots[0])
        plots.append(p)
        for ch, name, color in chans:
            curves.append((ch, p.plot(pen=pg.mkPen(color, width=1), name=name)))

    status = pg.LabelItem(justify="left")
    win.addItem(status, row=len(PLOTS), col=0)

    def update():
        d, s, dropped, frames = rx.snapshot()
        if len(s) < 2:
            return
        d = np.where(np.isfinite(d), d, np.nan)   # NaN/Inf must not wreck the axes
        t = (s - s[-1]) * frame_dt                # 0 = newest sample
        for ch, curve in curves:
            curve.setData(t, d[:, ch], connect="finite")
        status.setText(f"frames {frames}   dropped {dropped}")

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    try:
        precise = QtCore.Qt.TimerType.PreciseTimer   # Qt6 style
    except AttributeError:
        precise = QtCore.Qt.PreciseTimer             # Qt5 style
    timer.setTimerType(precise)
    timer.start(5)   

    pg.exec()
    rx.running = False


if __name__ == "__main__":
    main()