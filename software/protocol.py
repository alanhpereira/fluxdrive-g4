"""
Binary config protocol (docs/design-notes.md, config protocol). Keep in sync with Core/Inc/proto.h.

    A5 5C | type u8 | tok u8 | len u8 | payload[len] | crc u16
    crc = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type..payload, little-endian

Telemetry frames (A5 5A, 72 bytes) share the link; bytes outside frames are console text.
"""
import collections
import math
import struct
import threading

SYNC_MSG = b"\xA5\x5C"
MAX_LEN = 64
PROTO_VERSION = 5

# message types (the reply is type | REPLY)
PING, READ, WRITE, LIST = 0x01, 0x10, 0x11, 0x12
SAVE, CAL, ABORT = 0x20, 0x30, 0x31
RUN_CUR, RUN_VEL, RUN_POS, STOP, HOME = 0x40, 0x41, 0x42, 0x43, 0x44
REBOOT = 0x50  # mode u8: REBOOT_RESET or REBOOT_DFU
REBOOT_RESET, REBOOT_DFU = 0, 1
REPLY, EVENT = 0x80, 0x7F  # EVENT has no REPLY bit: it must never equal (request | REPLY)

# CAL which
CAL_ENC, CAL_R, CAL_LD, CAL_LQ, CAL_TUNE, CAL_KV, CAL_ZERO, CAL_HFI_MAP, CAL_HFI_POL, CAL_CUR_OFF = range(1, 11)
CAL_NAMES = {CAL_ENC: "encoder", CAL_R: "resistance", CAL_LD: "inductance d", CAL_LQ: "inductance q",
             CAL_TUNE: "PI autotune", CAL_KV: "kv", CAL_ZERO: "encoder zero reset", CAL_HFI_MAP: "HFI saliency map",
             CAL_HFI_POL: "HFI polarity", CAL_CUR_OFF: "current-sense offsets"}

# command status (st_t), then CANopen SDO abort codes for object access
STATUS = [
    "ok", "aborted by user", "bridge fault", "bus voltage too low", "overcurrent", "overspeed (1.5 x vlim)",
    "bridge disabled (fault latched?)", "set rated/imax first", "run 'r' first", "measure R and L first",
    "set the current-loop bandwidth (bw) first", "run e, r, l and t first", "run e and t first",
    "set the speed gains (vkp, vki) first", "set the speed limit (vlim) first", "set the position gain (pkp) first",
    "voltage limit hit before reaching the target current (wiring? current sign?)",
    "current change too small (current sense scale/sign?)", "current too low",
    "test current must stay below 0.9 x imax", "recording timeout (is currentLoop running?)",
    "current did not fall to 25% inside the window (L/R too slow)", "L/R shorter than ~3 samples, cannot resolve",
    "rotor did not follow the field (raise current, check load)", "pole pairs not an integer: rotor lost steps",
    "too slow to measure back-EMF (load/friction?)", "spun backwards (encoder offset/direction? rerun e)",
    "too few samples during coast-down", "back-EMF has the wrong sign (encoder offset/direction?)",
    "control loop not running", "busy (a calibration is running)", "bad argument", "unknown message",
    "flash save failed", "not allowed while the motor runs: stop first",
    "set the injection voltage (hv) first", "run the HFI saliency map first (hsal = 0)",
    "no HF current response (raise hv? check hhalf)", "HFI did not lock on an axis (saliency too low? hbw?)",
    "polarity check ended on the wrong side (raise hpi?)",
    "sensorless: HFI stopped tracking for 50 ms (raise hv? check the polarity)",
    "polarity contrast below 5 % (raise hpi)",
    "run the PI autotune (t) first", "set kv first (k, or from the datasheet)",
    "position mode needs the encoder (angle source 0 or 1)",
    "bus voltage above vmax",
    "tripped: press STOP to reset (once the motor has coasted down)",
    "current-sense offsets implausible (rotor turning? sense stage?)",
    "the CANopen master has the motor (STOP takes it back)",
    "CANopen master lost (heartbeat timeout or NMT left operational)",
    "following error (position error above 0x6065 for longer than 0x6066)",
    "encoder lost: no valid angle for 10 ms (stream stalled, CRC failing, sensor not answering)",
]
ST_OK, ST_ABORTED, ST_BUSY = 0, 1, 30
ST_NEED_VGAINS, ST_RUNNING, ST_CAN_ACTIVE, ST_CAN_LOST, ST_FOLLOWING_ERROR = 13, 34, 48, 49, 50
SDO = {
    0x06020000: "object does not exist",
    0x06090011: "subindex does not exist",
    0x06010002: "read-only object",
    0x06070010: "wrong data length",
    0x06090030: "value out of range",
    0x08000022: "not allowed in the present device state (stop the motor first)",
}
TIMEOUT_ST = -1  # local: no reply


def status_text(st):
    if st == TIMEOUT_ST:
        return "no reply (timeout)"
    if 0 <= st < len(STATUS):
        return STATUS[st]
    return SDO.get(st, f"status 0x{st:08X}")


# events: code -> (name, fields, formatter)
def _fmt_l(v):
    axis = "Lq" if v[0] else "Ld"
    tail = "" if v[4] else "  (measured only, not stored)"
    return f"{axis} = {v[1] * 1e6:.1f} uH   (tau {v[2] * 1e6:.1f} us, I0 {v[3]:.3f} A){tail}"


EV_DONE, EV_STOPPED, EV_RPM_LIMITED, EV_CUR_LIMITED, EV_RESUMED = 1, 2, 3, 4, 5
EV_ALIGN, EV_SWEEP, EV_OFFSET_AVG, EV_HOLD_SHAFT, EV_UNLOAD_SHAFT, EV_EEPROM_WAIT, EV_BW_CLAMPED = range(0x10, 0x17)
EV_R, EV_LCURVE, EV_LCURVE_PT, EV_L, EV_PP_BAD, EV_ENC, EV_TUNE, EV_KV, EV_HFI_PT, EV_HFI_FIT, EV_HFI_POL, \
    EV_HFI_START, EV_CUR_OFF = range(0x20, 0x2D)


def hfi_fit(a0, a2, b2):
    """1/L(phi) = a0 + a2 cos 2phi + b2 sin 2phi -> (L min (H), L max (H), deg elec of the L-min axis from d)"""
    m = math.hypot(a2, b2)
    lmax = 1.0 / (a0 - m) if a0 > m else float("inf")
    return 1.0 / (a0 + m), lmax, math.degrees(0.5 * math.atan2(b2, a2))


def _fmt_hfi_fit(v):
    lmin, lmax, ax = hfi_fit(v[1], v[2], v[3])
    verdict = ("comfortable" if abs(v[4]) >= 0.10 else "workable with averaging" if abs(v[4]) >= 0.05
               else "too low for robust HFI")
    return (f"HFI fit @ id {v[0]:.2f} A: L {lmin * 1e6:.2f} .. {lmax * 1e6:.2f} uH, low-L axis {ax:+.1f} deg from d, "
            f"saliency (Lq-Ld)/(Lq+Ld) = {v[4] * 100:+.1f} % ({verdict})" + ("  [stored as hsal]" if v[5] else ""))


def _fmt_hfi_pol(v):
    return (f"HFI - encoder: {v[0]:+.1f} deg after lock, {v[1]:+.1f} deg after polarity "
            f"({'flipped' if v[2] else 'kept'}, contrast {v[3] * 100:+.1f} %)")


EVENTS = {
    EV_DONE: lambda v: f"{CAL_NAMES.get(int(v[1]), '?')}: " + ("done" if int(v[0]) == 0 else f"FAILED: {status_text(int(v[0]))}"),
    EV_STOPPED: lambda v: f"STOPPED: {status_text(int(v[0]))} (id {v[1]:.2f} A, iq {v[2]:.2f} A)",
    EV_RPM_LIMITED: lambda v: f"limited to vlim {v[0]:.0f} rpm",
    EV_CUR_LIMITED: lambda v: f"limited to rated {v[0]:.2f} A",
    EV_RESUMED: lambda v: f"bus back down to {v[0]:.1f} V: resumed after the overvoltage coast ({v[1]:.0f} so far)",
    EV_ALIGN: lambda v: "aligning...",
    EV_SWEEP: lambda v: f"sweeping {v[0]:.0f} electrical revolutions...",
    EV_OFFSET_AVG: lambda v: "averaging the offset over one turn each way...",
    EV_HOLD_SHAFT: lambda v: f"HOLD THE SHAFT FIRMLY: q test starts in {v[0]:.0f} s",
    EV_UNLOAD_SHAFT: lambda v: f"motor will spin freely: unload the shaft. starting in {v[0]:.0f} s",
    EV_EEPROM_WAIT: lambda v: f"saving encoder EEPROM, wait {v[0]:.0f} s...",
    EV_BW_CLAMPED: lambda v: f"bw {v[0]:.0f} Hz above fs/20, clamped to {v[1]:.0f} Hz",
    EV_R: lambda v: f"R = {v[0]:.4f} ohm   ({v[1]:.3f} V @ {v[2]:.3f} A, {v[3]:.3f} V @ {v[4]:.3f} A, voltage offset {v[5]:.3f} V)",
    EV_LCURVE: lambda v: f"secant L along the decay (I0 {v[0]:.2f} A, v_off {v[1]:.3f} V):",
    EV_LCURVE_PT: lambda v: f"  i {v[0]:6.2f} A   L {v[1] * 1e6:6.1f} uH",
    EV_L: _fmt_l,
    EV_PP_BAD: lambda v: f"pole pairs came out {v[0]:.2f}, not an integer: rotor lost steps",
    EV_ENC: lambda v: (f"encoder dir {v[0]:+.0f}, pole pairs {v[1]:.0f} (measured {v[2]:.2f}); single point was "
                       f"{v[3]:+.1f} deg off the average, spread {v[4]:.1f} deg elec"),
    EV_TUNE: lambda v: f"bw {v[0]:.0f} Hz:  d kp {v[1]:.4f} ki {v[2]:.2f}   q kp {v[3]:.4f} ki {v[4]:.2f}",
    EV_KV: lambda v: (f"kv = {v[0]:.1f} rpm/V   (lambda {v[1] * 1e3:.3f} mWb, Kt {v[2]:.4f} Nm/A, offset {v[3]:.3f} V, "
                      f"{v[4]:.0f} samples, peak {v[5]:.0f} rpm)"),
    EV_HFI_PT: lambda v: f"  id {v[0]:5.2f} A  phi {v[1]:5.1f} deg   L {v[2] * 1e6:7.2f} uH   across/along {v[3]:+.3f}",
    EV_HFI_FIT: _fmt_hfi_fit,
    EV_HFI_POL: _fmt_hfi_pol,
    EV_HFI_START: lambda v: (f"polarity contrast {v[0] * 100:+.1f} % "
                             f"({'flipped' if v[1] else 'kept'}), HFI - encoder {v[2]:+.1f} deg"),
    EV_CUR_OFF: lambda v: (f"current-sense offsets {v[0]:+.0f} {v[1]:+.0f} {v[2]:+.0f} counts from 2048 "
                           f"(spread {v[3]:.0f} {v[4]:.0f} {v[5]:.0f})" + ("  [stored]" if v[6] else "  NOT stored")),
}


def event_text(code, vals):
    f = EVENTS.get(code)
    if f is None:
        return f"event 0x{code:04X} {' '.join(f'{x:g}' for x in vals)}"
    try:
        return f(vals)
    except (IndexError, ValueError):
        return f"event 0x{code:04X} (malformed)"


# ---------------------------------------------------------------------------------------------- framing
def crc16(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(mtype, tok, payload=b""):
    if len(payload) > MAX_LEN:
        raise ValueError("payload too long")
    body = bytes((mtype, tok, len(payload))) + payload
    return SYNC_MSG + body + struct.pack("<H", crc16(body))


def message_at(buf):
    """buf starts with A5 5C. Returns (frame length, (type, tok, payload)) for a good frame,
    (0, None) if more bytes are needed, (-1, None) if it is not a valid frame (resync)."""
    if len(buf) < 5:
        return 0, None
    n = buf[4]
    if n > MAX_LEN:
        return -1, None
    total = 7 + n
    if len(buf) < total:
        return 0, None
    body = bytes(buf[2:5 + n])
    if crc16(body) != struct.unpack_from("<H", buf, 5 + n)[0]:
        return -1, None
    return total, (body[0], body[1], body[3:])


TELEM_SYNC = b"\xA5\x5A"
TELEM_LEN = 72


def telem_ok(frame):
    """telemetry checksum: rotate-left-1-and-add over the first 17 words (frame_sum() in motorCtrl.c)"""
    s = 0x5A5AA5A5
    for w in struct.unpack_from("<17I", frame):
        s = (((s << 1) | (s >> 31)) + w) & 0xFFFFFFFF
    return s == struct.unpack_from("<I", frame, 68)[0]


class Splitter:
    """Separates the byte stream into telemetry frames, protocol messages and text."""

    def __init__(self):
        self.buf = bytearray()
        self.resyncs = 0  # a sync pair that did not start a good frame (corruption, or A5 inside text)

    def feed(self, chunk):
        """-> list of ("telem", 72 bytes) | ("msg", (type, tok, payload)) | ("text", bytes), in stream order"""
        out = []
        txt = bytearray()
        buf = self.buf
        buf += chunk

        def flush():
            if txt:
                out.append(("text", bytes(txt)))
                txt.clear()

        while True:
            i = buf.find(0xA5)
            if i < 0:
                txt += buf
                buf.clear()
                break
            if i:
                txt += buf[:i]
                del buf[:i]
            if len(buf) < 2:
                break  # the second sync byte may be in the next read
            if buf[1] == 0x5A:
                if len(buf) < TELEM_LEN:
                    break
                fr = bytes(buf[:TELEM_LEN])
                if telem_ok(fr):
                    flush()
                    out.append(("telem", fr))
                    del buf[:TELEM_LEN]
                    continue
                self.resyncs += 1
            elif buf[1] == 0x5C:
                n, m = message_at(buf)
                if n == 0:
                    break
                if n > 0:
                    flush()
                    out.append(("msg", m))
                    del buf[:n]
                    continue
                self.resyncs += 1
            txt += buf[:1]
            del buf[:1]
        flush()
        return out


# value packing by object type (objdict.h)
T_U8, T_I8, T_U16, T_U32, T_I32, T_F32, T_I64, T_I16 = range(1, 9)
FMT = {T_U8: "<B", T_I8: "<b", T_U16: "<H", T_U32: "<I", T_I32: "<i", T_F32: "<f", T_I64: "<q", T_I16: "<h"}


def pack_value(t, v):
    return struct.pack(FMT[t], float(v) if t == T_F32 else int(round(v)))


def unpack_value(t, b):
    return struct.unpack(FMT[t], b[:struct.calcsize(FMT[t])])[0]


# ---------------------------------------------------------------------------------------------- link
class Result:
    def __init__(self, status, payload=b""):
        self.status = status
        self.payload = payload

    @property
    def ok(self):
        return self.status == ST_OK

    def __repr__(self):
        return f"Result({status_text(self.status)}, {self.payload.hex()})"


# per type: reply timeout (s) and whether a timeout may be retried (no side effects / idempotent)
TIMING = {PING: (0.3, True), READ: (0.3, True), WRITE: (0.3, True), LIST: (0.3, True), SAVE: (1.5, False),
          CAL: (0.5, False), ABORT: (0.5, True), RUN_CUR: (0.5, False), RUN_VEL: (0.5, False),
          RUN_POS: (0.5, False), STOP: (5.0, True), HOME: (5.0, False),
          REBOOT: (0.5, False)}


class Link(threading.Thread):
    """Request/reply over a byte sink, one request in flight, replies matched by token.

    write(bytes) sends a whole frame (the caller serialises it with text lines). The receive side
    calls on_message() for every good A5 5C frame. Results and events are queued for the GUI thread
    in one ordered queue: call drain() from there, which runs the callbacks and on_event(tok, code, vals)
    (without on_event, events collect in .events). An event tagged with the request in flight is held
    until that request's result is queued: a CAL that fails its checks sends ACK and EV_DONE back to back,
    and the EV_DONE must not reach the GUI before the ACK."""

    def __init__(self, write):
        super().__init__(daemon=True)
        self.write = write
        self.q = collections.deque()
        self.ev = threading.Event()
        self.running = True
        self.tok = 0
        self.wait_tok = None
        self.hold_tok = None  # events with this tok wait in held until the request's result is queued
        self.held = []
        self.reply = None
        self.got = threading.Event()
        self.lock = threading.Lock()
        self.done = collections.deque()  # ("res", callback, Result) | ("ev", (tok, code, [floats]), None), in order
        self.on_event = None  # GUI thread: on_event(tok, code, vals), from drain()
        self.events = collections.deque()  # (tok, code, [floats]) drained while on_event is None
        self.timeouts = 0
        self.unmatched = 0

    def _next_tok(self):
        self.tok = self.tok % 255 + 1  # 0 is reserved for unsolicited events
        return self.tok

    # ---- called from the receiver thread
    def on_message(self, mtype, tok, payload):
        if mtype == EVENT:
            if len(payload) >= 3:
                code, n = struct.unpack_from("<HB", payload)
                n = min(n, (len(payload) - 3) // 4)
                ev = (tok, code, list(struct.unpack_from(f"<{n}f", payload, 3)))
                with self.lock:
                    if tok and tok == self.hold_tok:
                        self.held.append(ev)
                    else:
                        self.done.append(("ev", ev, None))
            return
        with self.lock:
            if self.wait_tok is not None and tok == self.wait_tok[0] and mtype == (self.wait_tok[1] | REPLY):
                self.reply = payload
                self.got.set()
                return
        self.unmatched += 1

    # ---- requests (any thread)
    def request(self, mtype, payload=b"", callback=None, timeout=None, urgent=False):
        """queue a request; callback(Result) runs in the GUI thread from drain(). urgent: front of the queue"""
        r = (mtype, payload, callback, timeout)
        if urgent:
            self.q.appendleft(r)
        else:
            self.q.append(r)
        self.ev.set()

    def busy(self):
        return bool(self.q) or self.wait_tok is not None

    def clear_queue(self, keep=()):
        """drop queued (not yet sent) requests, except the given types. Their callbacks still run, with
        ST_ABORTED, so whoever counts replies (read_all, an export, a blocking UsbSync) is not left waiting"""
        dropped, kept = [], []
        while True:  # popleft is atomic; iterating the deque while the Link thread pops it raises RuntimeError
            try:
                r = self.q.popleft()
            except IndexError:
                break
            (kept if r[0] in keep else dropped).append(r)
        self.q.extendleft(reversed(kept))
        for r in dropped:
            self.done.append(("res", r[2], Result(ST_ABORTED)))

    def drain(self):
        while self.done:
            kind, a, b = self.done.popleft()
            if kind == "ev":
                if self.on_event:
                    self.on_event(*a)
                else:
                    self.events.append(a)
            elif a:
                a(b)

    def stop(self):
        self.running = False
        self.ev.set()
        self.got.set()

    def run(self):
        while self.running:
            self.ev.wait(0.1)
            self.ev.clear()
            while self.q and self.running:
                mtype, payload, cb, timeout = self.q.popleft()
                res = self._transact(mtype, payload, timeout)
                with self.lock:  # the result first, then the events it held back
                    self.done.append(("res", cb, res))
                    self.done.extend(("ev", e, None) for e in self.held)
                    self.held.clear()
                    self.hold_tok = None

    def _transact(self, mtype, payload, timeout):
        t, retry = TIMING.get(mtype, (0.5, False))
        t = timeout or t
        for attempt in range(2 if retry else 1):
            tok = self._next_tok()
            with self.lock:
                self.wait_tok = (tok, mtype)
                self.hold_tok = tok
                self.reply = None
                self.got.clear()
            try:
                self.write(encode(mtype, tok, payload))
            except Exception:  # noqa: BLE001 (port gone: report as a timeout)
                with self.lock:
                    self.wait_tok = None  # else busy() stays true and the status poll stops
                break
            ok = self.got.wait(t)
            with self.lock:
                self.wait_tok = None
                reply = self.reply
            if ok and reply is not None:
                return self._parse(mtype, reply)
            self.timeouts += 1
        return Result(TIMEOUT_ST)

    @staticmethod
    def _parse(mtype, reply):
        if mtype in (PING, LIST):
            return Result(ST_OK, reply)
        if len(reply) < 4:
            return Result(TIMEOUT_ST)
        return Result(struct.unpack_from("<I", reply)[0], reply[4:])


# request payload helpers
def p_read(index, sub):
    return struct.pack("<HB", index, sub)


def p_write(index, sub, t, value):
    return struct.pack("<HB", index, sub) + pack_value(t, value)


def p_list(first):
    return struct.pack("<H", first)


def p_cal(which, arg=0.0):
    return struct.pack("<Bf", which, arg)


def p_reboot(dfu=False):
    return bytes([REBOOT_DFU if dfu else REBOOT_RESET])


def p_run_cur(id_, iq):
    return struct.pack("<ff", id_, iq)


def p_run_vel(rpm):
    return struct.pack("<f", rpm)


def p_run_pos(turns):
    """turns (float, or a str for full precision) -> Q32.32"""
    from decimal import Decimal
    q = int((Decimal(str(turns)) * (1 << 32)).to_integral_value())
    return struct.pack("<q", q)


def parse_ping(payload):
    """-> (proto version, table hash, PARAM_VERSION, build id)"""
    return struct.unpack_from("<HIHI", payload)


def parse_list(payload):
    """-> [(index, sub, type, flags), ...]"""
    n = payload[0] if payload else 0
    return [struct.unpack_from("<HBBB", payload, 1 + 5 * i) for i in range(n)]
