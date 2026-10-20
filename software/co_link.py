"""
CANopen master side for the drive (docs/design-notes.md, CANopen PC tool), shared by the plotter's CANopen tab,
cotest.py and ad-hoc scripts. No Qt in here.

    CoBus      python-can bus + canopen.Network: raw frame tap, counters, PC heartbeat, software SYNC producer
    DriveNode  one drive: SDO with retry and abort text, NMT, heartbeat watch, EMCY log, PDO samples
    Drive402   CiA 402 on top: controlword commands, statusword decoding, enable walk, PP / HM helpers
    Streamer   CSP / CSV / CST set-points (+ SYNC) every period, with guards and a log of what was sent
    UsbSync    blocking requests over the USB/serial config link (protocol.Link), for tests

Threads: python-can's notifier thread runs the callbacks here; they only append to locked rings and set
fields. A GUI reads them from its own timer. Every bus.send() goes through one lock (pyserial's Win32
write is not safe from two threads: SLCAN gave write timeouts).

    import co_link as C
    bus = C.CoBus(); bus.connect("slcan", "COM8", 1000000)
    d = C.Drive402(bus.add_drive(1)); d.node.nmt("OPERATIONAL"); d.goto_enabled(C.MODE_CSV)
"""
import bisect
import collections
import itertools
import math
import os
import struct
import threading
import time

import can
import canopen
import numpy as np

import objects as O
import protocol as P

HERE = os.path.dirname(os.path.abspath(__file__))
EDS = os.path.join(HERE, "driverTCC.eds")
PC_HB_ID = 0x7F
INC_REV = 1 << 16  # 0x608F default (Q16.16 turns); DriveNode reads the real one

# ---------------------------------------------------------------------------------------------- texts
ABORTS = {
    0x05030000: "toggle bit not alternated", 0x05040000: "SDO protocol timed out",
    0x05040001: "command specifier not valid", 0x05040005: "out of memory",
    0x06010000: "unsupported access", 0x06010001: "read of a write-only object",
    0x06010002: "write of a read-only object", 0x06020000: "object does not exist",
    0x06040041: "object cannot be mapped to a PDO", 0x06040042: "PDO length exceeded",
    0x06040043: "general parameter incompatibility", 0x06040047: "general internal incompatibility",
    0x06060000: "hardware error", 0x06070010: "data type / length mismatch", 0x06070012: "data too long",
    0x06070013: "data too short", 0x06090011: "subindex does not exist", 0x06090030: "value range exceeded",
    0x06090031: "value too high", 0x06090032: "value too low", 0x06090036: "max below min",
    0x060A0023: "resource not available", 0x08000000: "general error",
    0x08000020: "cannot be stored / transferred", 0x08000021: "local control",
    0x08000022: "not in the present device state", 0x08000023: "no object dictionary",
    0x08000024: "no data available",
}

EMCY_CODES = {
    0x0000: "error reset / no error", 0x1000: "generic error", 0x2310: "continuous overcurrent",
    0x3210: "DC link overvoltage", 0x3220: "DC link undervoltage", 0x5000: "device hardware",
    0x5400: "power section (gate driver fault / break)", 0x6000: "device software", 0x6100: "internal software",
    0x6300: "data set", 0x7300: "sensor (sensorless angle lost / polarity)", 0x8100: "communication",
    0x8110: "CAN overrun (objects lost)", 0x8120: "CAN error passive", 0x8130: "heartbeat / life guard: master lost",
    0x8140: "recovered from bus off", 0x8150: "CAN-ID collision", 0x8200: "protocol error",
    0x8210: "PDO not processed (length)", 0x8220: "PDO length exceeded", 0x8240: "unexpected SYNC length",
    0x8250: "RPDO timeout", 0x8400: "velocity (overspeed)", 0x8611: "following error",
    0xFF01: "local STOP over USB",
}

# CANopenNode error status bits (byte 3 of its EMCY): CO_EM_* in CO_Emergency.h, 0x40.. from k_fault in co402.c
EM_BITS = {
    0x00: "no error", 0x01: "CAN bus warning", 0x02: "RX wrong length", 0x03: "RX overflow",
    0x04: "RPDO wrong length", 0x05: "RPDO overflow", 0x06: "CAN RX bus passive", 0x07: "CAN TX bus passive",
    0x08: "wrong NMT command", 0x09: "TIME timeout", 0x12: "CAN TX bus off", 0x13: "CAN RX buffer overflow",
    0x14: "CAN TX buffer overflow", 0x15: "TPDO outside SYNC window", 0x17: "RPDO timeout", 0x18: "SYNC timeout",
    0x19: "SYNC length", 0x1A: "PDO mapping", 0x1B: "heartbeat consumer timeout", 0x1C: "heartbeat remote reset",
    0x20: "EMCY buffer full", 0x22: "MCU reset", 0x27: "auto save failed", 0x28: "wrong error report",
    0x29: "timer overflow", 0x2A: "memory allocation", 0x2B: "generic error", 0x2C: "software error",
    0x2D: "inconsistent OD", 0x2E: "parameter calculation", 0x2F: "non-volatile memory",
    0x40: "overcurrent", 0x41: "DC link overvoltage", 0x42: "DC link undervoltage", 0x43: "bridge fault",
    0x44: "overspeed", 0x45: "sensorless angle", 0x46: "following error", 0x47: "local STOP", 0x48: "master lost",
    0x49: "other fault",
}
ER_BITS = [(0x01, "generic"), (0x02, "current"), (0x04, "voltage"), (0x08, "temperature"), (0x10, "communication"),
           (0x20, "device profile"), (0x80, "manufacturer")]


def abort_text(code):
    return f"0x{code:08X} {ABORTS.get(code, 'abort')}"


def bits_text(v, table):
    return ", ".join(name for b, name in table if v & b) or "-"


# ---------------------------------------------------------------------------------------------- CiA 402
NOT_READY, SOD, RTSO, SO, OE, QSA, FRA, FAULT = range(8)  # same numbers as 0x4042:1 (FSA_STATES)
FSA_NAMES = O.FSA_STATES
MODE_PP, MODE_PV, MODE_HM, MODE_CSP, MODE_CSV, MODE_CST = 1, 3, 6, 8, 9, 10
MODES = O.CIA402_MODES

CW_SHUTDOWN, CW_SWITCH_ON, CW_ENABLE, CW_DISABLE_OP, CW_QUICK_STOP, CW_DISABLE_V = 0x06, 0x07, 0x0F, 0x07, 0x02, 0x00
CW_FAULT_RESET, CW_HALT, CW_NEW_SP, CW_IMMEDIATE, CW_RELATIVE = 0x80, 0x100, 0x10, 0x20, 0x40

SW_BITS_COMMON = {0: "ready to switch on", 1: "switched on", 2: "operation enabled", 3: "fault", 4: "voltage enabled",
                  5: "quick stop (0 = active)", 6: "switch on disabled", 7: "warning", 8: "manufacturer",
                  9: "remote", 10: "target reached", 11: "internal limit", 12: "op mode 12", 13: "op mode 13",
                  14: "manufacturer 14", 15: "manufacturer 15"}
SW_BITS_MODE = {
    MODE_PP: {10: "target reached", 12: "set-point acknowledge", 13: "following error"},
    MODE_PV: {10: "target reached", 12: "speed = 0", 13: "-"},
    MODE_HM: {10: "target reached", 12: "homing attained", 13: "homing error"},
    MODE_CSP: {12: "following target", 13: "following error"},
    MODE_CSV: {12: "following target"},
    MODE_CST: {12: "following target"},
}


def fsa_state(sw):
    if sw & 0x4F == 0x00:
        return NOT_READY
    if sw & 0x4F == 0x40:
        return SOD
    if sw & 0x4F == 0x0F:
        return FRA
    if sw & 0x4F == 0x08:
        return FAULT
    return {0x21: RTSO, 0x23: SO, 0x27: OE, 0x07: QSA}.get(sw & 0x6F, NOT_READY)


def sw_bit_names(mode):
    names = dict(SW_BITS_COMMON)
    names.update(SW_BITS_MODE.get(mode, {}))
    return names


def decode_sw(sw, mode=None):
    """-> (state, [(bit, name, on), ...] for bits 4..13, mode-aware)"""
    names = sw_bit_names(mode)
    return fsa_state(sw), [(b, names[b], bool(sw >> b & 1)) for b in (4, 5, 7, 9, 10, 11, 12, 13)]


class Units:
    """CAN units <-> engineering units (docs/user-manual-driver.md, section 9.4)"""

    def __init__(self, inc_rev=INC_REV, rated_ma=1000):
        self.inc_rev = inc_rev
        self.rated_ma = rated_ma

    def inc_to_turn(self, x):
        return x / self.inc_rev

    def turn_to_inc(self, t):
        return int(round(t * self.inc_rev))

    def inc_to_deg(self, x):
        return x * 360.0 / self.inc_rev

    def ips_to_rpm(self, v):
        return v * 60.0 / self.inc_rev

    def rpm_to_ips(self, rpm):
        return int(round(rpm * self.inc_rev / 60.0))

    def ips2_to_rpms(self, a):
        return a * 60.0 / self.inc_rev

    def rpms_to_ips2(self, a):
        return int(round(a * self.inc_rev / 60.0))

    def pm_to_amp(self, pm):
        return pm * self.rated_ma / 1e6

    def amp_to_pm(self, a):
        return int(round(a * 1e6 / self.rated_ma)) if self.rated_ma else 0


# ---------------------------------------------------------------------------------------------- rings
class Ring:
    """bounded, locked list of tuples; snapshot() returns a copy, since(n) what came after a counter value"""

    def __init__(self, cap):
        self.lock = threading.Lock()
        self.q = collections.deque(maxlen=cap)
        self.total = 0

    def append(self, item):
        with self.lock:
            self.q.append(item)
            self.total += 1

    def snapshot(self):
        with self.lock:
            return list(self.q)

    def tail(self, n):
        """-> the newest n items, oldest first (walks back from the end: touches only those n)"""
        with self.lock:
            out = list(itertools.islice(reversed(self.q), n))
        out.reverse()
        return out

    def since(self, total):
        """-> (items appended after the counter was `total`, new counter)"""
        with self.lock:
            n = min(self.total - total, len(self.q))
            return (list(self.q)[len(self.q) - n:] if n > 0 else []), self.total

    def clear(self):
        with self.lock:
            self.q.clear()


class Series:
    """numpy ring of (t, value) per signal, for plots"""

    def __init__(self, cap=20000):
        self.lock = threading.Lock()
        self.cap = cap
        self.sig = {}  # key -> [t array, v array, head]

    def append(self, key, t, v):
        with self.lock:
            s = self.sig.get(key)
            if s is None:
                s = self.sig[key] = [np.zeros(self.cap), np.zeros(self.cap), 0]
            k = s[2] % self.cap
            s[0][k] = t
            s[1][k] = v
            s[2] += 1

    def get(self, key):
        """-> (t, v) in time order, copies"""
        with self.lock:
            s = self.sig.get(key)
            if s is None:
                return np.zeros(0), np.zeros(0)
            t, v, h = s
            if h <= self.cap:
                return t[:h].copy(), v[:h].copy()
            k = h % self.cap
            return np.roll(t, -k), np.roll(v, -k)

    def tail(self, key, n):
        """-> the last n (t, v), copies, in time order"""
        with self.lock:
            s = self.sig.get(key)
            if s is None:
                return np.zeros(0), np.zeros(0)
            t, v, h = s
            n = min(n, h, self.cap)
            idx = np.arange(h - n, h) % self.cap
            return t[idx], v[idx]

    def since(self, key, head):
        """-> (t, v, new head): what was appended after the counter value `head` (at most one ring)"""
        with self.lock:
            s = self.sig.get(key)
            if s is None:
                return np.zeros(0), np.zeros(0), 0
            t, v, h = s
            n = min(h - head, self.cap)
            if n <= 0:
                return np.zeros(0), np.zeros(0), h
            idx = np.arange(h - n, h) % self.cap
            return t[idx], v[idx], h

    def head(self, key):
        with self.lock:
            s = self.sig.get(key)
            return s[2] if s else 0

    def keys(self):
        with self.lock:
            return list(self.sig)

    def clear(self):
        with self.lock:
            self.sig.clear()


# ---------------------------------------------------------------------------------------------- bus
def frame_bits(dlc, ext=False):
    """approximate bits on the wire, with ~20 % stuffing"""
    return ((67 if ext else 47) + 8 * dlc) * 1.2


class CoBus:
    """The adapter. connect() opens it; everything else needs it open."""

    def __init__(self, trace_cap=50000):
        self.network = None
        self.bus = None
        self.bitrate = 0
        self.trace = Ring(trace_cap)  # (t, cob id, bytes, "rx" | "tx", flags) flags: "E" error frame
        self.send_lock = threading.Lock()
        self.rx_frames = self.tx_frames = self.err_frames = self.tx_errors = 0
        self.bits = 0.0
        self._rate = (time.time(), 0, 0.0)  # for frames/s and load
        self.fps = self.load = 0.0
        self.hb = None
        self.sync = None
        self.drives = {}
        self.connected_at = 0.0

    # ---- connection
    def connect(self, interface="slcan", channel="COM8", bitrate=1000000, **kw):
        self._args = (interface, channel, kw)
        net = canopen.Network()
        net.connect(interface=interface, channel=channel, bitrate=bitrate, **kw)
        self.network, self.bus, self.bitrate = net, net.bus, bitrate
        orig = net.bus.send

        def send(msg, timeout=None):
            with self.send_lock:
                try:
                    orig(msg, timeout)
                except can.CanError:
                    self.tx_errors += 1
                    raise
                self.tx_frames += 1
                self.bits += frame_bits(msg.dlc, msg.is_extended_id)
            self.trace.append((time.time(), msg.arbitration_id, bytes(msg.data), "tx", ""))
        net.bus.send = send  # canopen, python-can's periodic tasks and this module all send through it
        if interface == "slcan":
            _fast_slcan(net.bus)
        net.notifier.add_listener(_Tap(self))
        self.connected_at = time.time()
        self._rate = (self.connected_at, 0, 0.0)
        return self

    @property
    def is_connected(self):
        return self.network is not None

    def disconnect(self):
        """stops the producers, then closes; callers disable the drive first (Drive402.close())"""
        for p in (self.sync, self.hb):
            if p:
                p.stop()
        self.sync = self.hb = None
        net, self.network, self.bus = self.network, None, None
        self.drives.clear()
        if net:
            try:
                net.disconnect()
            except Exception:  # noqa: BLE001 (adapter already gone)
                pass

    def reconnect(self, bitrate=None):
        """close and reopen the adapter (e.g. at another bit rate); the drives are added again (new objects),
        the PC heartbeat restarts if it ran"""
        interface, channel, kw = self._args
        ids = list(self.drives)
        hb = (self.hb.node_id, self.hb.period) if self.hb else None
        self.disconnect()
        time.sleep(0.2)
        self.connect(interface, channel, bitrate or self.bitrate, **kw)
        for i in ids:
            self.add_drive(i)
        if hb:
            self.start_heartbeat(*hb)

    def add_drive(self, node_id=1, eds=EDS):
        d = DriveNode(self, node_id, eds)
        self.drives[node_id] = d
        return d

    def send(self, cob_id, data):
        self.network.send_message(cob_id, bytes(data))

    def nmt_all(self, cmd):
        """broadcast NMT: 'OPERATIONAL', 'PRE-OPERATIONAL', 'STOPPED', 'RESET', 'RESET COMMUNICATION'"""
        self.network.nmt.send_command(NMT_CODES[cmd])

    # ---- counters
    def rates(self):
        """frames/s (both directions) and bus load (0..1), over the time since the last call"""
        now = time.time()
        t0, n0, b0 = self._rate
        dt = now - t0
        if dt >= 0.2:
            n = self.rx_frames + self.tx_frames
            self.fps = (n - n0) / dt
            self.load = (self.bits - b0) / dt / self.bitrate if self.bitrate else 0.0
            self._rate = (now, n, self.bits)
        return self.fps, self.load

    # ---- producers
    def start_heartbeat(self, node_id=PC_HB_ID, period=0.1):
        self.stop_heartbeat()
        self.hb = Heartbeat(self, node_id, period)
        self.hb.start()
        return self.hb

    def stop_heartbeat(self):
        if self.hb:
            self.hb.stop()
        self.hb = None

    def start_sync(self, period=0.005):
        """software SYNC producer (TPDO2/3 are sent on SYNC); Streamer hooks run just before each SYNC"""
        if self.sync and self.sync.is_alive():
            self.sync.period = period
            return self.sync
        self.sync = SyncProducer(self, period)
        self.sync.start()
        return self.sync

    def stop_sync(self):
        if self.sync:
            self.sync.stop()
        self.sync = None


def _fast_slcan(bus):
    """python-can 4.6's SLCAN driver on Windows, measured: it reads the port one byte per read(1) call (~15000
    Win32 reads/s here, each needing the GIL: with the GUI busy, replies came 50-300 ms late), and flushes after
    every frame (pyserial's Win32 flush() sleeps 50 ms whenever the driver still holds TX bytes: 50-100 ms
    gaps in SYNC). This replaces both on the instance: chunked reads, no flush."""
    port = bus.serialPortOrig
    term = (bus._OK[0], bus._ERROR[0])

    def _write(string):
        with can.exceptions.error_check("Could not write to serial device"):
            port.write(string.encode() + bus.LINE_TERMINATOR)

    def _read(timeout):
        deadline = None if timeout is None else time.perf_counter() + timeout
        buf = bus._buffer
        errors = 0
        while True:
            try:
                chunk = port.read(port.in_waiting or 1)  # read(1) waits up to the port timeout (1 ms)
                errors = 0
            except Exception as e:  # noqa: BLE001 (a transient Win32 error must not end reception)
                errors += 1
                bus.read_errors = getattr(bus, "read_errors", 0) + 1
                bus.read_error = repr(e)
                if errors > 20:
                    raise can.CanOperationError(f"Could not read from serial device: {e!r}") from e
                time.sleep(0.005)
                continue
            if chunk:
                buf.extend(chunk)
                out = None
                start = 0
                for i, c in enumerate(buf):
                    if c in term:
                        line = buf[start:i + 1].decode("ascii", "replace")
                        start = i + 1
                        if out is None:
                            out = line
                        else:
                            bus._queue.put_nowait(line)  # _recv_internal takes queued lines first
                if start:
                    del buf[:start]
                if out is not None:
                    return out
            if deadline is not None and time.perf_counter() >= deadline:
                return None

    bus._write = _write
    bus._read = _read


class _Tap(can.Listener):
    def __init__(self, bus):
        self.b = bus

    def on_message_received(self, m):
        b = self.b
        if m.is_error_frame:
            b.err_frames += 1
            b.trace.append((m.timestamp, 0, b"", "rx", "E"))
            return
        b.rx_frames += 1
        b.bits += frame_bits(m.dlc, m.is_extended_id)
        b.trace.append((m.timestamp, m.arbitration_id, bytes(m.data), "rx", ""))

    def on_error(self, exc):
        self.b.err_frames += 1


def busy_wait_until(t_end):
    """sleep until ~2 ms before, then spin (Windows sleep granularity is ~1 ms at best). The spin yields the
    GIL each turn: a pure spin keeps it for the whole switch interval and starves the other threads"""
    left = t_end - time.perf_counter()
    if left > 0.002:
        time.sleep(left - 0.002)
    while time.perf_counter() < t_end:
        time.sleep(0)


class Heartbeat(threading.Thread):
    """the PC's own heartbeat (state 0x05 operational). paused = True simulates a lost master"""

    def __init__(self, bus, node_id, period):
        super().__init__(daemon=True, name="pc-heartbeat")
        self.bus, self.node_id, self.period = bus, node_id, period
        self.running = True
        self.paused = False
        self.sent = 0

    def run(self):
        t = time.perf_counter()
        while self.running:
            if not self.paused and self.bus.network:
                try:
                    self.bus.send(0x700 + self.node_id, b"\x05")
                    self.sent += 1
                except Exception:  # noqa: BLE001 (adapter gone: the GUI notices)
                    pass
            t += self.period
            now = time.perf_counter()
            if t < now:
                t = now
            time.sleep(max(0.0, t - now))

    def stop(self):
        self.running = False


class SyncProducer(threading.Thread):
    """SYNC every period (perf_counter busy-wait for the last ~2 ms). hooks: callables(t_planned) run before
    each SYNC (the Streamer sends its RPDOs there). sent: (planned, actual) perf_counter times for the jitter"""

    def __init__(self, bus, period):
        super().__init__(daemon=True, name="sync")
        self.bus, self.period = bus, period
        self.running = True
        self.hooks = []
        self.sent = Ring(20000)
        self.count = 0
        self.late = 0  # cycles skipped because we were more than a period late

    def run(self):
        t = time.perf_counter()
        while self.running and self.bus.network:
            t += self.period
            busy_wait_until(t)
            now = time.perf_counter()
            if now - t > self.period:
                self.late += 1
                t = now
            for h in list(self.hooks):
                try:
                    h(t)
                except Exception:  # noqa: BLE001 (a hook failing must not stop the SYNC)
                    pass
            try:
                self.bus.network.sync.transmit()
            except Exception:  # noqa: BLE001
                break
            self.sent.append((t, time.perf_counter()))
            self.count += 1

    def stop(self):
        self.running = False
        if self.is_alive() and threading.current_thread() is not self:
            self.join(0.5)

    def jitter(self):
        """-> numpy array of the measured period (s) between consecutive SYNCs"""
        s = self.sent.snapshot()
        if len(s) < 2:
            return np.zeros(0)
        a = np.array([x[1] for x in s])
        return np.diff(a)


# ---------------------------------------------------------------------------------------------- node
class CoError(Exception):
    """SDO failure: .code is the abort code (0 = no reply), str() is readable"""

    def __init__(self, text, code=0, index=0, sub=0):
        super().__init__(text)
        self.code, self.index, self.sub = code, index, sub


class DriveNode:
    """One CANopen node with the drive's EDS"""

    SDO_TIMEOUT = 0.25  # replies take 2-3 ms; SLCAN loses one now and then (more with SYNC traffic): retry soon
    SDO_RETRIES = 3
    HB_DEFAULT_MS = 1000

    def __init__(self, cobus, node_id, eds=EDS):
        self.cobus = cobus
        self.id = node_id
        self.node = cobus.network.add_node(node_id, eds)
        self.node.sdo.RESPONSE_TIMEOUT = self.SDO_TIMEOUT
        self.node.sdo.MAX_RETRIES = 1  # one attempt in canopen; retries here, where we know what is safe
        self.sdo_lock = threading.RLock()
        self.retries = 0
        self.sdo_count = 0
        self.sdo_fail = 0
        self.emcy = Ring(2000)  # (t, code, register, bit, info, text, at_connect)
        self.emcy_count = 0
        self.hb_t = 0.0  # time of the last heartbeat
        self.hb_state = None  # NMT state byte
        self.hb_period = self.HB_DEFAULT_MS / 1000.0
        self.bootups = Ring(100)  # times of boot-up messages
        self.series = Series()
        self.pdo_maps = {}  # ("tpdo"|"rpdo", n) -> [(index, sub, name, bits)]
        self.sw = None  # latest statusword (TPDO1 or SDO)
        self.mode_disp = None
        self.err_code = None
        self.sw_t = 0.0
        self.units = Units()
        self.node.emcy.add_callback(self._on_emcy)
        cobus.network.subscribe(0x700 + node_id, self._on_hb)

    # ---- SDO
    def _var(self, index, sub):
        obj = self.node.sdo[index]
        return obj if sub is None or isinstance(obj, canopen.sdo.SdoVariable) else obj[sub]

    def _transact(self, fn, index, sub, retry):
        with self.sdo_lock:
            q = self.node.sdo.responses
            while not q.empty():  # a reply that came in after its request timed out must not answer this one
                q.get_nowait()
            last = None
            for attempt in range(1 + (self.SDO_RETRIES if retry else 0)):
                self.sdo_count += 1
                try:
                    return fn()
                except canopen.SdoAbortedError as e:
                    self.sdo_fail += 1
                    raise CoError(f"0x{index:04X}:{sub or 0}: {abort_text(e.code)}", e.code, index, sub or 0) from None
                except (canopen.SdoCommunicationError, can.CanError) as e:
                    last = e
                    self.sdo_fail += 1
                    if attempt < (self.SDO_RETRIES if retry else 0):
                        self.retries += 1
            raise CoError(f"0x{index:04X}:{sub or 0}: no reply ({last})", 0, index, sub or 0)

    def read(self, index, sub=0, retry=True):
        """typed from the EDS (int, float, str)"""
        v = self._var(index, sub)
        return self._transact(lambda: v.raw, index, sub, retry)

    def write(self, index, sub, value, retry=True):
        """retry=False for writes that must not happen twice (a controlword edge, a store): on a lost reply it
        reads the object back, and writes again only when the old value is still there. Measured over SLCAN:
        it is the request that gets lost, not the reply"""
        v = self._var(index, sub)

        def w():
            v.raw = value
        if retry:
            return self._transact(w, index, sub, True)
        for attempt in range(1 + self.SDO_RETRIES):
            try:
                return self._transact(w, index, sub, False)
            except CoError as e:
                if e.code or attempt == self.SDO_RETRIES:
                    raise
            try:  # no reply: did it land?
                back = self.read(index, sub)
            except CoError:
                raise CoError(f"0x{index:04X}:{sub or 0}: no reply, and no read-back", 0, index, sub or 0) from None
            self.retries += 1
            if back == value:  # landed. 0x1010/0x1011 read back their capability, never equal: resent (harmless)
                return None
        raise CoError(f"0x{index:04X}:{sub or 0}: no reply", 0, index, sub or 0)

    def upload(self, index, sub=0):
        return self._transact(lambda: self.node.sdo.upload(index, sub), index, sub, True)

    def download(self, index, sub, data, retry=False):
        return self._transact(lambda: self.node.sdo.download(index, sub, bytes(data)), index, sub, retry)

    def read_native(self, key):
        o = O.BY_KEY[key]
        return self.read(o.index, o.sub)

    def write_native(self, key, value):
        o = O.BY_KEY[key]
        return self.write(o.index, o.sub, value)

    def od_entry(self, index, sub=0):
        e = self.node.object_dictionary[index]
        return e if isinstance(e, canopen.objectdictionary.ODVariable) else e[sub]

    # ---- NMT
    def nmt(self, cmd):
        """'OPERATIONAL', 'PRE-OPERATIONAL', 'STOPPED', 'RESET', 'RESET COMMUNICATION'"""
        self.node.nmt.state = cmd

    @property
    def nmt_state(self):
        return {0: "INITIALISING", 4: "STOPPED", 5: "OPERATIONAL", 127: "PRE-OPERATIONAL"}.get(self.hb_state, "?")

    def hb_alive(self):
        return self.hb_t and time.time() - self.hb_t < 1.5 * self.hb_period + 0.1

    def wait_bootup(self, timeout=5.0, since=None):
        since = time.time() if since is None else since
        end = time.time() + timeout
        while time.time() < end:
            for t in self.bootups.snapshot():
                if t >= since:
                    return t
            time.sleep(0.01)
        return None

    def _on_hb(self, can_id, data, ts):
        if not data:
            return
        st = data[0] & 0x7F
        self.hb_t, self.hb_state = ts, st
        if st == 0:
            self.bootups.append(ts)

    # ---- EMCY
    def _on_emcy(self, e):
        bit = e.data[0] if len(e.data) >= 1 else 0
        info = struct.unpack_from("<I", e.data, 1)[0] if len(e.data) >= 5 else 0
        text = EMCY_CODES.get(e.code, EMCY_CODES.get(e.code & 0xFF00, f"0x{e.code:04X}"))
        if e.code and 0x40 <= bit < 0x50 and info:
            text += f" ({P.status_text(info)})"  # drive faults carry the st_t code in the info bytes
        at_connect = e.timestamp - self.cobus.connected_at < 0.5
        self.emcy.append((e.timestamp, e.code, e.register, bit, info, text, at_connect))
        self.emcy_count += 1

    # ---- identity and units
    def refresh_units(self):
        try:
            num, den = self.read(0x608F, 1), self.read(0x608F, 2)
            inc = num // max(den, 1)
        except CoError:
            inc = INC_REV
        try:
            ma = self.read(0x6075)
        except CoError:
            ma = 1000
        self.units = Units(inc, ma)
        try:
            ms = self.read(0x1017)
            self.hb_period = max(0.05, ms / 1000.0) if ms else 1.0
        except CoError:
            pass
        return self.units

    # ---- PDOs
    def pdo_read(self):
        """reads the live mapping of all PDOs (SDO), subscribes the TPDOs; -> {("tpdo", n): [(idx, sub, name, bits)]}"""
        with self.sdo_lock:
            for attempt in range(1 + self.SDO_RETRIES):  # many SDOs, none retried by canopen: retry the lot
                try:
                    self.node.tpdo.read()
                    self.node.rpdo.read()
                    break
                except (canopen.SdoCommunicationError, can.CanError):
                    self.retries += 1
                    if attempt == self.SDO_RETRIES:
                        raise CoError("PDO mapping: no reply") from None
                except canopen.SdoAbortedError as e:
                    raise CoError(f"PDO mapping: {abort_text(e.code)}", e.code) from None
        self.pdo_maps.clear()
        for kind, maps in (("tpdo", self.node.tpdo), ("rpdo", self.node.rpdo)):
            for n, m in maps.map.items():
                self.pdo_maps[(kind, n)] = [(v.index, v.subindex, v.name, v.length) for v in m.map]
                if kind == "tpdo":
                    m.callbacks.clear()
                    m.add_callback(self._on_tpdo)
        return self.pdo_maps

    def pdo_info(self, kind, n):
        m = (self.node.tpdo if kind == "tpdo" else self.node.rpdo).map[n]
        return {"cob_id": m.cob_id, "enabled": m.enabled, "trans_type": m.trans_type,
                "inhibit": getattr(m, "inhibit_time", None), "event": getattr(m, "event_timer", None),
                "map": [(v.index, v.subindex, v.name, v.length) for v in m.map]}

    def _on_tpdo(self, m):
        t = m.timestamp or time.time()
        for v in m.map:
            try:
                raw = v.raw
            except Exception:  # noqa: BLE001 (short PDO)
                continue
            if isinstance(raw, (int, float)):
                self.series.append((v.index, v.subindex), t, raw)
            if v.index == 0x6041:
                self.sw, self.sw_t = raw, t
            elif v.index == 0x6061:
                self.mode_disp = raw
            elif v.index == 0x603F:
                self.err_code = raw

    def pdo_remap(self, kind, n, entries, trans_type=None, event_timer=None, enabled=True):
        """entries [(index, sub)], pre-operational only (the drive refuses otherwise); saves with SDO"""
        maps = self.node.tpdo if kind == "tpdo" else self.node.rpdo
        m = maps.map[n]
        with self.sdo_lock:
            m.read()
            m.clear()
            for idx, sub in entries:
                m.add_variable(idx, sub)
            if trans_type is not None:
                m.trans_type = trans_type
            if event_timer is not None:
                m.event_timer = event_timer
            m.enabled = enabled and bool(entries)
            m.save()
        return self.pdo_read()

    def send_rpdo(self, n, values):
        """values {name or (index, sub): raw}; the mapping must have been read (pdo_read)"""
        m = self.node.rpdo.map[n]
        for k, v in values.items():
            var = next(x for x in m.map if (x.index, x.subindex) == k) if isinstance(k, tuple) else m[k]
            var.raw = v
        m.transmit()

    def sync(self):
        self.cobus.network.sync.transmit()

    # ---- store / restore
    def store(self, sub):
        """0x1010:sub 'save' (1 all, 2 comm, 3 application). Refused while the motor runs"""
        self.write(0x1010, sub, 0x65766173, retry=False)

    def restore(self, sub):
        """0x1011:sub 'load': defaults from the next communication reset (comm) / boot (application)"""
        self.write(0x1011, sub, 0x64616F6C, retry=False)


class Drive402:
    """CiA 402 commands, nothing hidden: each call writes 0x6040 once and returns. The halt bit sticks"""

    def __init__(self, node):
        self.node = node
        self.halted = False
        self.cw = 0

    @property
    def units(self):
        return self.node.units

    def _cw(self, base, extra=0):
        cw = base | extra | (CW_HALT if self.halted else 0)
        self.node.write(0x6040, 0, cw, retry=False)
        self.cw = cw
        return cw

    def shutdown(self):
        return self._cw(CW_SHUTDOWN)

    def switch_on(self):
        return self._cw(CW_SWITCH_ON)

    def enable(self, extra=0):
        return self._cw(CW_ENABLE, extra)

    def disable_op(self):
        return self._cw(CW_DISABLE_OP)

    def quick_stop(self):
        return self._cw(CW_QUICK_STOP)

    def disable_voltage(self):
        return self._cw(CW_DISABLE_V)

    def fault_reset(self):
        """0 -> 1 edge on bit 7, then back to 0 (ready for the next edge)"""
        self._cw(0x00)
        self._cw(0x00, CW_FAULT_RESET)
        return self._cw(0x00)

    def halt(self, on):
        self.halted = bool(on)
        return self._cw(self.cw & ~CW_HALT & 0xFFFF)

    # ---- status
    def sw(self):
        sw = self.node.read(0x6041)
        self.node.sw, self.node.sw_t = sw, time.time()
        return sw

    def state(self):
        return fsa_state(self.sw())

    def mode(self):
        return self.node.read(0x6061)

    def set_mode(self, mode):
        self.node.write(0x6060, 0, mode)

    def refusal(self):
        """-> (st_t, text) of 0x4042:2"""
        st = self.node.read_native("fsa_refused")
        return st, P.status_text(st)

    def fault_text(self):
        code = self.node.read(0x603F)
        st = self.node.read_native("fsa_fault")
        return code, f"0x{code:04X} {EMCY_CODES.get(code, '')}" + (f" ({P.status_text(st)})" if st else "")

    def wait_sw(self, cond, timeout=1.0, poll=0.01):
        """polls 6041 until cond(sw) -> sw, or None on timeout"""
        end = time.time() + timeout
        while True:
            sw = self.sw()
            if cond(sw):
                return sw
            if time.time() >= end:
                return None
            time.sleep(poll)

    def wait_state(self, state, timeout=1.0):
        return self.wait_sw(lambda sw: fsa_state(sw) == state, timeout)

    def goto_enabled(self, mode=None, timeout=1.0):
        """6060 = mode, then 0x06 -> 0x07 -> 0x0F, checking the state after each. Raises CoError with the
        refusal reason (0x4042:2) if the drive does not get there. For CSP it first writes 607A = 6064: the
        drive interpolates from the present position to whatever 607A holds, a stale target included"""
        if mode is not None:
            self.set_mode(mode)
        if (mode if mode is not None else self.node.read(0x6060)) == MODE_CSP:
            self.node.write(0x607A, 0, self.node.read(0x6064))
        for cmd, want in ((self.shutdown, RTSO), (self.switch_on, SO), (self.enable, OE)):
            if fsa_state(self.sw()) == OE and want != OE:
                continue
            cmd()
            if self.wait_state(want, timeout) is None:
                sw = self.node.sw
                if fsa_state(sw) in (FAULT, FRA):
                    raise CoError(f"in fault (sw 0x{sw:04X}): {self.fault_text()[1]}: fault reset first")
                st, why = self.refusal()
                raise CoError(f"stuck in {FSA_NAMES.get(fsa_state(sw), '?')} (sw 0x{sw:04X}) on the way to "
                              f"{FSA_NAMES[want]}: {why}", st)
        return self.node.sw

    def stop_all(self, timeout=2.0, standstill_rpm=None):
        """disable voltage and wait until CAN has let go of the motor (the coast ends below 30 rpm);
        with standstill_rpm, also until |606C| is below it (the shaft coasts on freely after that)"""
        self.halted = False
        self.disable_voltage()
        end = time.time() + timeout
        done = False
        while time.time() < end:
            try:
                if not self.node.read_native("can_owner"):
                    if standstill_rpm is None or abs(self.units.ips_to_rpm(self.node.read(0x606C))) < standstill_rpm:
                        done = True
                        break
            except CoError:
                pass
            time.sleep(0.05)
        return done

    # ---- PP
    def pp_move(self, target_inc, relative=False, immediate=True, timeout=1.0):
        """607A, then the bit 4 / bit 12 handshake: set bit 4, wait ack, clear bit 4, wait ack clear.
        Returns False if the set-point was not acknowledged (buffer full)"""
        self.node.write(0x607A, 0, int(target_inc))
        extra = (CW_RELATIVE if relative else 0) | (CW_IMMEDIATE if immediate else 0)
        self.enable(extra)
        self.enable(extra | CW_NEW_SP)
        ok = self.wait_sw(lambda sw: sw & 0x1000, timeout) is not None
        self.enable(extra)
        self.wait_sw(lambda sw: not sw & 0x1000, timeout)
        return ok

    def wait_reached(self, timeout=10.0):
        return self.wait_sw(lambda sw: sw & 0x0400, timeout, poll=0.02) is not None

    # ---- HM
    def home(self, method=37, offset_inc=0, timeout=5.0):
        """6098 / 607C, mode 6, enable, bit 4 rising; -> (attained, error) bits"""
        self.node.write(0x6098, 0, method)
        self.node.write(0x607C, 0, int(offset_inc))
        self.goto_enabled(MODE_HM)
        self.enable(CW_NEW_SP)
        sw = self.wait_sw(lambda sw: sw & 0x2000 or (sw & 0x1400) == 0x1400, timeout)
        self.enable()
        if sw is None:
            return False, False
        return bool(sw & 0x1000), bool(sw & 0x2000)


# ---------------------------------------------------------------------------------------------- streamer
def gen_step(amp, t_on=0.0, period=0.0):
    """amp after t_on; with a period, a square wave +amp / -amp"""
    def f(t):
        if t < t_on:
            return 0.0
        if period:
            return amp if ((t - t_on) % period) < period / 2 else -amp
        return amp
    return f


def gen_ramp(slope, limit):
    return lambda t: max(-abs(limit), min(abs(limit), slope * t))


def gen_sine(amp, freq, offset=0.0):
    return lambda t: offset + amp * math.sin(2 * math.pi * freq * t)


def gen_chirp(amp, f0, f1, dur):
    k = (f1 - f0) / dur if dur else 0.0

    def f(t):
        t = min(t, dur)
        return amp * math.sin(2 * math.pi * (f0 * t + 0.5 * k * t * t))
    return f


def gen_trapezoid(dist, vmax, acc):
    """position profile 0 -> dist with the speed limited to vmax and the acceleration to acc (all > 0)"""
    d, s = abs(dist), (1 if dist >= 0 else -1)
    vmax, acc = abs(vmax), abs(acc)
    ta = vmax / acc
    if acc * ta * ta > d:  # triangle
        ta = math.sqrt(d / acc)
        vmax = acc * ta
    tc = (d - acc * ta * ta) / vmax if vmax else 0.0

    def f(t):
        if t <= 0:
            return 0.0
        if t < ta:
            x = 0.5 * acc * t * t
        elif t < ta + tc:
            x = 0.5 * acc * ta * ta + vmax * (t - ta)
        elif t < 2 * ta + tc:
            r = 2 * ta + tc - t
            x = d - 0.5 * acc * r * r
        else:
            x = d
        return s * x
    return f


def gen_file(path):
    """CSV of 't,value' rows (header lines that are not numbers are skipped); linear between points, held after"""
    ts, vs = [], []
    with open(path, encoding="utf-8") as f:
        for line in f:
            parts = line.replace(";", ",").split(",")
            try:
                ts.append(float(parts[0]))
                vs.append(float(parts[1]))
            except (ValueError, IndexError):
                continue
    if not ts:
        raise ValueError(f"{path}: no 't,value' rows")

    def f(t):
        i = bisect.bisect_right(ts, t)
        if i == 0:
            return vs[0]
        if i >= len(ts):
            return vs[-1]
        t0, t1 = ts[i - 1], ts[i]
        return vs[i - 1] + (vs[i] - vs[i - 1]) * (t - t0) / (t1 - t0)
    return f


GENERATORS = {"step": gen_step, "ramp": gen_ramp, "sine": gen_sine, "chirp": gen_chirp, "trapezoid": gen_trapezoid,
              "file": gen_file}


class Streamer:
    """Streams CSP / CSV / CST set-points on RPDO2 / RPDO3 from the SYNC producer's thread, one per period
    before the SYNC. The generator gives engineering units relative to the start: turns (CSP, added to the
    position at start), rpm (CSV), ‰ of rated current (CST). Guards: past max |rpm| or max |turns| from
    the start, it disables voltage and stops.

    Needs NMT operational and the drive enabled in the mode (Drive402.goto_enabled); start() does not enable."""

    def __init__(self, drive, mode, gen, period=0.005, max_rpm=None, max_turns=None, duration=None):
        self.d, self.node, self.mode, self.gen = drive, drive.node, mode, gen
        self.period, self.max_rpm, self.max_turns, self.duration = period, max_rpm, max_turns, duration
        self.log = Ring(200000)  # (wall time it was sent, value sent (CAN units), value (eng units))
        self.running = False
        self.tripped = None  # guard text
        self.t0 = 0.0
        self.start_pos = 0
        self.sent = 0
        self.errors = 0
        self._wall0 = 0.0

    def start(self, start_pos=None):
        u = self.d.units
        if self.mode == MODE_CSP:
            self.node.write(0x60C2, 1, max(1, int(round(self.period * 1000))))  # interpolate over one period
            self.node.write(0x60C2, 2, -3)
            self.start_pos = self.node.read(0x6064) if start_pos is None else start_pos
        self._u = u
        self._pos_ref = None
        self.t0 = None
        self._wall0 = time.time() - time.perf_counter()
        self.running = True
        sp = self.node.cobus.start_sync(self.period)
        sp.hooks.append(self._hook)
        return self

    def stop(self):
        self.running = False
        sp = self.node.cobus.sync
        if sp and self._hook in sp.hooks:
            sp.hooks.remove(self._hook)

    def elapsed(self):
        return 0.0 if self.t0 is None else time.perf_counter() - self.t0

    def _guard(self, why):
        self.tripped = why
        self.stop()
        try:
            self.d.disable_voltage()
        except CoError:
            pass

    def _hook(self, t):
        if not self.running:
            return
        if self.t0 is None:
            self.t0 = t
        tr = t - self.t0
        if self.duration is not None and tr > self.duration:
            self.stop()
            return
        u = self._u
        s = self.node.series
        w0 = self.t0 + self._wall0  # guards look only at samples from this stream
        # only the newest sample each period (tail, not get: a copy of the whole ring every 5 ms in this thread,
        # under the lock the notifier thread needs to append)
        if self.max_rpm is not None:
            tt, vv = s.tail((0x606C, 0), 1)
            if len(vv) and tt[-1] >= w0 and abs(u.ips_to_rpm(vv[-1])) > self.max_rpm:
                return self._guard(f"speed {u.ips_to_rpm(vv[-1]):.0f} rpm beyond the {self.max_rpm:.0f} rpm guard")
        if self.max_turns is not None:
            tt, vv = s.tail((0x6064, 0), 1)
            if len(vv) and tt[-1] >= w0:
                if self._pos_ref is None:  # CSP: the start position; else the first sample of this stream
                    self._pos_ref = self.start_pos if self.mode == MODE_CSP else vv[-1]
                if abs(u.inc_to_turn(vv[-1] - self._pos_ref)) > self.max_turns:
                    return self._guard(f"position beyond the ±{self.max_turns:g} turn guard")
        e = self.gen(tr)
        try:
            if self.mode == MODE_CSP:
                raw = (self.start_pos + u.turn_to_inc(e) + 2 ** 31) % 2 ** 32 - 2 ** 31
                self.node.send_rpdo(2, {(0x607A, 0): raw})
            elif self.mode == MODE_CSV:
                raw = u.rpm_to_ips(e)
                self.node.send_rpdo(3, {(0x60FF, 0): raw})
            else:
                raw = max(-32768, min(32767, int(round(e))))
                self.node.send_rpdo(3, {(0x6071, 0): raw})
        except Exception:  # noqa: BLE001
            self.errors += 1
            return
        self.sent += 1
        self.log.append((time.perf_counter() + self._wall0, raw, e))  # when it actually went out (the plan is t)


# ---------------------------------------------------------------------------------------------- USB link
class UsbSync:
    """Blocking requests over a protocol.Link whose drain() someone else calls (the GUI timer), or
    UsbSerial below (its own drain thread). link may be a callable that returns the current Link (or None):
    the GUI replaces its Link when it reopens a port that went away (the drive resetting), and a request
    in flight on the old one is sent again on the new one."""

    def __init__(self, link):
        self._link = link

    @property
    def link(self):
        return self._link() if callable(self._link) else self._link

    def request(self, mtype, payload=b"", timeout=6.0):
        t_end = time.time() + timeout
        while time.time() < t_end:
            link = self.link
            if link is None:  # port being reopened
                time.sleep(0.05)
                continue
            ev = threading.Event()
            box = []

            def cb(res, box=box, ev=ev):
                box.append(res)
                ev.set()
            link.request(mtype, payload, cb, urgent=(mtype == P.STOP))
            while not ev.wait(0.05):
                if self.link is not link or time.time() >= t_end:
                    break
            if box:
                return box[0]
        return P.Result(P.TIMEOUT_ST)

    def read(self, key):
        o = O.BY_KEY[key]
        r = self.request(P.READ, P.p_read(o.index, o.sub))
        if not r.ok:
            raise CoError(f"USB read {key}: {P.status_text(r.status)}", r.status)
        return P.unpack_value(o.type, r.payload)

    def write(self, key, value):
        o = O.BY_KEY[key]
        r = self.request(P.WRITE, P.p_write(o.index, o.sub, o.type, value))
        if not r.ok:
            raise CoError(f"USB write {key}: {P.status_text(r.status)}", r.status)

    def stop(self):
        return self.request(P.STOP).status

    def events(self):
        """-> [(tok, code, vals)] queued since the last call"""
        out = []
        link = self.link
        while link and link.events:
            out.append(link.events.popleft())
        return out


class UsbSerial(UsbSync):
    """Headless config link: the port, a reader thread (Splitter -> Link), the Link thread, a drain thread"""

    def __init__(self, port, baud=2000000):
        import serial  # only needed headless
        self.serial = serial
        self.port, self.baud = port, baud
        self.ser = serial.Serial(port, baud, timeout=0.01)
        self.split = P.Splitter()
        self.wlock = threading.Lock()
        link = P.Link(self._write)
        super().__init__(link)
        self.running = True
        self._rx = threading.Thread(target=self._read_loop, daemon=True)
        self._dr = threading.Thread(target=self._drain_loop, daemon=True)
        self.ser.dtr = True  # the USB CDC only sends while the host holds DTR
        link.start()
        self._rx.start()
        self._dr.start()

    def _write(self, frame):
        """a write into a port that went away (the drive resetting) waits up to 5 s for _reopen() and retries once:
        CAN reports the boot-up ~0.1 s after a reset node, the USB CDC is back 1-2 s later"""
        ser = self.ser
        try:
            with self.wlock:
                ser.write(frame)
            return
        except (self.serial.SerialException, OSError):
            t0 = time.time()
            while self.running and self.ser is ser and time.time() - t0 < 5.0:
                time.sleep(0.05)
            if self.ser is ser:
                raise
        with self.wlock:
            self.ser.write(frame)

    def _reopen(self):
        """the USB CDC port goes away when the drive resets (reset node, REBOOT): wait for it to come back"""
        try:
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass
        while self.running:
            time.sleep(0.2)
            try:
                ser = self.serial.Serial(self.port, self.baud, timeout=0.01)
            except (self.serial.SerialException, OSError):
                continue
            ser.dtr = True
            with self.wlock:
                self.ser = ser
            self.split = P.Splitter()
            return

    def _read_loop(self):
        while self.running:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except (self.serial.SerialException, OSError):
                if self.running:
                    self._reopen()
                continue
            for kind, d in self.split.feed(chunk):
                if kind == "msg":
                    self.link.on_message(*d)

    def _drain_loop(self):
        while self.running:
            self.link.drain()
            time.sleep(0.005)

    def close(self):
        self.running = False
        self.link.stop()
        self._rx.join(0.5)
        try:
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass


# ---------------------------------------------------------------------------------------------- GUI helper
class Worker(threading.Thread):
    """Runs blocking calls (SDO) off the GUI thread, one at a time. submit(fn, done) -> done(result, error)
    is queued for the GUI, which calls drain() from its timer."""

    def __init__(self):
        super().__init__(daemon=True, name="co-worker")
        self.q = collections.deque()
        self.ev = threading.Event()
        self.done = collections.deque()
        self.running = True
        self.busy = False

    def submit(self, fn, done=None, urgent=False):
        (self.q.appendleft if urgent else self.q.append)((fn, done))
        self.ev.set()

    def pending(self):
        return len(self.q) + (1 if self.busy else 0)

    def clear(self):
        self.q.clear()

    def run(self):
        while self.running:
            self.ev.wait(0.1)
            self.ev.clear()
            while self.q and self.running:
                fn, done = self.q.popleft()
                self.busy = True
                try:
                    res, err = fn(), None
                except Exception as e:  # noqa: BLE001 (reported to the GUI)
                    res, err = None, e
                self.busy = False
                if done:
                    self.done.append((done, res, err))

    def drain(self):
        while self.done:
            cb, res, err = self.done.popleft()
            cb(res, err)

    def stop(self):
        self.running = False
        self.ev.set()


# ---------------------------------------------------------------------------------------------- trace decoding
def classify(cob_id, node_id=None):
    """-> (kind, node) for a COB-ID in the pre-defined connection set"""
    if cob_id == 0:
        return "NMT", None
    if cob_id == 0x80:
        return "SYNC", None
    if cob_id == 0x100:
        return "TIME", None
    fn, nid = cob_id & 0x780, cob_id & 0x7F
    kind = {0x080: "EMCY", 0x180: "TPDO1", 0x200: "RPDO1", 0x280: "TPDO2", 0x300: "RPDO2", 0x380: "TPDO3",
            0x400: "RPDO3", 0x480: "TPDO4", 0x500: "RPDO4", 0x580: "SDO resp", 0x600: "SDO req", 0x700: "HB"}.get(fn, "?")
    if kind == "HB" and nid == PC_HB_ID:
        kind = "HB (PC)"
    return kind, nid


NMT_CODES = {"OPERATIONAL": 0x01, "STOPPED": 0x02, "PRE-OPERATIONAL": 0x80, "RESET": 0x81, "RESET COMMUNICATION": 0x82}
NMT_CMDS = {0x01: "start", 0x02: "stop", 0x80: "pre-op", 0x81: "reset node", 0x82: "reset comm"}
HB_STATES = {0x00: "boot-up", 0x04: "stopped", 0x05: "operational", 0x7F: "pre-operational"}


def describe(cob_id, data):
    """short text for a frame in the trace"""
    kind, nid = classify(cob_id)
    d = bytes(data)
    if kind == "NMT" and len(d) >= 2:
        return f"{NMT_CMDS.get(d[0], hex(d[0]))} node {d[1] or 'all'}"
    if kind.startswith("HB") and d:
        return HB_STATES.get(d[0] & 0x7F, hex(d[0]))
    if kind == "EMCY" and len(d) >= 3:
        code = struct.unpack_from("<H", d)[0]
        return f"0x{code:04X} {EMCY_CODES.get(code, '')} reg 0x{d[2]:02X}" + (f" bit 0x{d[3]:02X}" if len(d) > 3 else "")
    if kind.startswith("SDO") and len(d) >= 4:
        cs = d[0] >> 5
        idx, sub = struct.unpack_from("<HB", d, 1)
        what = {"SDO req": {1: "download", 2: "upload", 4: "abort"},
                "SDO resp": {2: "upload reply", 3: "download ok", 4: "abort"}}[kind].get(cs, f"cs{cs}")
        if cs == 4 and len(d) >= 8:
            return f"{what} 0x{idx:04X}:{sub} {abort_text(struct.unpack_from('<I', d, 4)[0])}"
        return f"{what} 0x{idx:04X}:{sub}"
    return ""
