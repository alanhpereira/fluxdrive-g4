"""
PC copy of the object table in Core/Src/objdict.c. Keep the same entries in the same order with the same
index, sub, type and flags: both sides hash them (table_hash()) and the GUI warns when they differ.
Everything else here (labels, units, display scale, limits, help) is presentation and only lives here.
"""
from protocol import T_U8, T_I8, T_U16, T_U32, T_I32, T_F32

RO, STORED, PI, NOTRUN = 0x01, 0x02, 0x04, 0x08


class Obj:
    def __init__(self, index, sub, t, flags, key, label, unit="", scale=1.0, lo=None, hi=None, group="", help="",
                 decimals=4):
        self.index, self.sub, self.type, self.flags = index, sub, t, flags
        self.key = key  # the old 's <key>' name, also the JSON key
        self.label = label
        self.unit = unit  # display unit
        self.scale = scale  # display = device value * scale
        self.lo, self.hi = lo, hi  # device units; the firmware checks the same limits
        self.group = group
        self.help = help
        self.decimals = 0 if t != T_F32 else decimals

    @property
    def ro(self):
        return bool(self.flags & RO)

    @property
    def stored(self):
        return bool(self.flags & STORED)

    def to_display(self, v):
        return v * self.scale if self.type == T_F32 else v

    def from_display(self, d):
        return d / self.scale if self.type == T_F32 else int(round(d))

    def fmt(self, v):
        if v is None:
            return ""
        if self.type != T_F32:
            return f"0x{v:02X}" if self.key in ("flags", "st_flags") else str(v)
        d = self.to_display(v)
        return f"{d:.{self.decimals}f}" if abs(d) < 1e7 else f"{d:.4g}"

    def check(self, v):
        """None if v (device units) is acceptable, else a reason"""
        if v != v:
            return "not a number"
        if self.lo is not None and v < self.lo:
            return f"below {self.fmt(self.lo)} {self.unit}".strip()
        if self.hi is not None and v > self.hi:
            return f"above {self.fmt(self.hi)} {self.unit}".strip()
        if self.key == "dir" and v == 0:
            return "must be +1 or -1"
        return None


G_INFO, G_STATUS, G_DIAG = "Device info", "Status", "Diagnostics"
G_MOTOR, G_SENS, G_CUR, G_OUTER, G_LIM, G_OBS = ("Motor", "Sensing (not used by the control path yet)", "Current loop",
                                                 "Speed / position", "Limits", "Observer")
BW_MAX = 25000.0 / 20

OBJECTS = [
    # 0x4000 device info
    Obj(0x4000, 1, T_U16, RO, "proto_ver", "Protocol version", group=G_INFO),
    Obj(0x4000, 2, T_U32, RO, "table_hash", "Object table hash", group=G_INFO),
    Obj(0x4000, 3, T_U16, RO, "param_ver", "PARAM_VERSION", group=G_INFO),
    Obj(0x4000, 4, T_U32, RO, "build_id", "Build id", group=G_INFO),
    # 0x4001 status
    Obj(0x4001, 1, T_U8, RO, "mode", "Mode", group=G_STATUS, help="0 off, 1 calibration, 2 run"),
    Obj(0x4001, 2, T_U8, RO, "loop", "Loop", group=G_STATUS, help="0 current, 1 speed, 2 position"),
    Obj(0x4001, 3, T_U32, RO, "st_flags", "Calibration flags", group=G_STATUS),
    Obj(0x4001, 4, T_U8, RO, "moe", "Bridge enabled (MOE)", group=G_STATUS),
    Obj(0x4001, 5, T_U8, RO, "nfault", "nFAULT high (ok)", group=G_STATUS),
    Obj(0x4001, 6, T_U16, RO, "drv", "Gate driver status", group=G_STATUS, help="0x100 = I2C error"),
    Obj(0x4001, 7, T_F32, RO, "vin", "Bus voltage", "V", group=G_STATUS, decimals=2),
    Obj(0x4001, 8, T_U8, RO, "busy", "Busy", group=G_STATUS, help="CAL_* running, 0 = idle"),
    Obj(0x4001, 9, T_U8, RO, "dirty", "Unsaved changes", group=G_STATUS),
    Obj(0x4001, 10, T_U32, RO, "seq", "Flash save count", group=G_STATUS),
    Obj(0x4001, 11, T_U8, RO, "brk", "TIM1 break latched", group=G_STATUS),
    Obj(0x4001, 12, T_U8, RO, "ready", "Gate driver READY", group=G_STATUS),
    # 0x4002 diagnostics (write 0 clears)
    Obj(0x4002, 1, T_U32, RO, "loop_cyc", "currentLoop() time", "cycles", group=G_DIAG),
    Obj(0x4002, 2, T_U32, 0, "loop_cyc_max", "currentLoop() max", "cycles", lo=0, hi=0, group=G_DIAG),
    Obj(0x4002, 3, T_U32, 0, "enc_samples", "Encoder reads", lo=0, hi=0, group=G_DIAG),
    Obj(0x4002, 4, T_U32, 0, "enc_crc", "Encoder CRC errors", lo=0, hi=0, group=G_DIAG),
    Obj(0x4002, 5, T_U32, 0, "enc_late", "Encoder late", lo=0, hi=0, group=G_DIAG,
        help="read not finished at the ADC sample: lower MT6835_GROUPS if this grows"),
    Obj(0x4002, 6, T_U32, 0, "enc_stalls", "Encoder stalls", lo=0, hi=0, group=G_DIAG),
    Obj(0x4002, 7, T_U8, RO, "enc_status", "Encoder status", group=G_DIAG, help="1 overspeed, 2 weak field, 4 undervoltage"),
    Obj(0x4002, 8, T_U8, RO, "enc_crc_check", "Encoder CRC check on", group=G_DIAG),
    Obj(0x4002, 9, T_U32, 0, "proto_err", "Protocol RX errors", lo=0, hi=0, group=G_DIAG),
    # 0x4010 motor
    Obj(0x4010, 1, T_F32, STORED, "kv", "kv", "rpm/V", lo=0, hi=1e5, group=G_MOTOR, decimals=1,
        help="back-EMF constant, line-to-line peak; measured by the kv calibration. The observer derives lambda from it"),
    Obj(0x4010, 2, T_U8, STORED | NOTRUN, "pp", "Pole pairs", lo=1, hi=64, group=G_MOTOR, help="measured by 'e'"),
    Obj(0x4010, 3, T_I8, STORED | NOTRUN, "dir", "Encoder direction", lo=-1, hi=1, group=G_MOTOR, help="+1 or -1, measured by 'e'"),
    Obj(0x4010, 4, T_U32, STORED | NOTRUN, "offset", "Electrical offset", "2^32=360°", lo=0, hi=2**32 - 1, group=G_MOTOR,
        help="electrical angle offset, measured by 'e'"),
    Obj(0x4010, 5, T_F32, STORED, "rd", "Rd", "mΩ", 1e3, 0, 100, G_MOTOR, "phase resistance, measured by 'r'", 2),
    Obj(0x4010, 6, T_F32, STORED, "rq", "Rq", "mΩ", 1e3, 0, 100, G_MOTOR, "= Rd (surface PM)", 2),
    Obj(0x4010, 7, T_F32, STORED, "ld", "Ld", "µH", 1e6, 0, 1, G_MOTOR, "measured by 'l'", 2),
    Obj(0x4010, 8, T_F32, STORED, "lq", "Lq", "µH", 1e6, 0, 1, G_MOTOR, "measured by 'q' (shaft held), else = Ld", 2),
    Obj(0x4010, 9, T_F32, STORED, "rated", "Rated current", "A", lo=0, hi=50, group=G_MOTOR, decimals=2,
        help="calibration currents are fractions of it; current commands are limited to it"),
    Obj(0x4010, 10, T_F32, STORED, "imax", "Max current", "A", lo=0, hi=50, group=G_MOTOR, decimals=2,
        help="hard trip: the watchdog stops above it"),
    Obj(0x4010, 11, T_U32, STORED | NOTRUN, "flags", "Calibration flags", lo=0, hi=255, group=G_MOTOR,
        help="1 offset, 2 current cal, 4 R, 8 L, 16 PI tuned. Restored by JSON import"),
    # 0x4011 sensing
    Obj(0x4011, 1, T_F32, STORED, "cscale", "Current scale", "A/count", lo=-1e9, hi=1e9, group=G_SENS, decimals=6),
    Obj(0x4011, 2, T_F32, STORED, "vscale", "Bus voltage scale", "V/count", lo=-1e9, hi=1e9, group=G_SENS, decimals=6),
    # 0x4020 current loop
    Obj(0x4020, 1, T_F32, STORED | PI, "kpd", "Kp d", "V/A", lo=0, hi=1e6, group=G_CUR),
    Obj(0x4020, 2, T_F32, STORED | PI, "kid", "Ki d", "V/(A·s)", lo=0, hi=1e6, group=G_CUR, decimals=2),
    Obj(0x4020, 3, T_F32, STORED | PI, "kpq", "Kp q", "V/A", lo=0, hi=1e6, group=G_CUR),
    Obj(0x4020, 4, T_F32, STORED | PI, "kiq", "Ki q", "V/(A·s)", lo=0, hi=1e6, group=G_CUR, decimals=2),
    Obj(0x4020, 5, T_F32, STORED, "bw", "Bandwidth", "Hz", lo=0, hi=BW_MAX, group=G_CUR, decimals=0,
        help="autotune ('t') input, at most fs/20"),
    # 0x4021 speed / position
    Obj(0x4021, 1, T_F32, STORED | PI, "vkp", "Speed Kp", "A/(turn/s)", lo=0, hi=1e4, group=G_OUTER),
    Obj(0x4021, 2, T_F32, STORED | PI, "vki", "Speed Ki", "A/turn", lo=0, hi=1e4, group=G_OUTER),
    Obj(0x4021, 3, T_F32, STORED, "pkp", "Position Kp", "1/s", lo=0, hi=1e4, group=G_OUTER, decimals=2),
    Obj(0x4021, 4, T_F32, STORED, "vlim", "Speed limit", "rpm", 60.0, 0, 1000, G_OUTER,
        "position-mode speed limit and speed-command clamp; overspeed trip at 1.5x", 0),
    Obj(0x4021, 5, T_F32, STORED, "acc", "Acceleration limit", "rpm/s", lo=0, hi=1e7, group=G_OUTER, decimals=0,
        help="speed reference ramp in the speed/position loops; 0 = off"),
    # 0x4022 limits
    Obj(0x4022, 1, T_F32, STORED, "vmin", "Bus min", "V", lo=0, hi=100, group=G_LIM, decimals=1,
        help="calibration and run stop below it"),
    Obj(0x4022, 2, T_F32, STORED, "vmax", "Bus max", "V", lo=0, hi=100, group=G_LIM, decimals=1, help="not used yet"),
    Obj(0x4022, 3, T_F32, STORED, "tmax", "Max temperature", "°C", lo=-40, hi=200, group=G_LIM, decimals=0, help="not used yet"),
    # 0x4030 observer
    Obj(0x4030, 1, T_U8, STORED, "oblend", "Blend observer", lo=0, hi=1, group=G_OBS,
        help="1: blend into angle and speed above olo; 0: run and log it only"),
    Obj(0x4030, 2, T_F32, STORED, "ogain", "Flux correction gain", "rad/s", lo=0, hi=1e6, group=G_OBS, decimals=0),
    Obj(0x4030, 3, T_F32, STORED, "olo", "Blend start", "V", lo=0, hi=100, group=G_OBS, decimals=2,
        help="peak back-EMF where the observer starts to get weight"),
    Obj(0x4030, 4, T_F32, STORED, "ohi", "Blend full", "V", lo=0, hi=100, group=G_OBS, decimals=2),
    Obj(0x4030, 5, T_F32, STORED, "oftol", "Flux tolerance", "", lo=0, hi=1, group=G_OBS, decimals=2,
        help="|flux|/lambda must stay within 1 ± tol"),
    Obj(0x4030, 6, T_F32, STORED, "oerr", "Max observer error", "deg elec", lo=0, hi=180, group=G_OBS, decimals=0),
]

BY_KEY = {o.key: o for o in OBJECTS}
BY_ADDR = {(o.index, o.sub): o for o in OBJECTS}
PARAM_GROUPS = [G_MOTOR, G_CUR, G_OUTER, G_LIM, G_OBS, G_SENS]

# PF_* in params.h
PF_ELEC_OFFSET_VALID, PF_CUR_CAL_VALID, PF_R_VALID, PF_L_VALID, PF_PI_VALID = 1, 2, 4, 8, 16


def table_hash(objs=OBJECTS):
    """FNV-1a 32 over (index lo, index hi, sub, type, flags) of every entry: objdict_init()"""
    h = 2166136261
    for o in objs:
        for b in (o.index & 0xFF, o.index >> 8, o.sub, o.type, o.flags):
            h = ((h ^ b) * 16777619) & 0xFFFFFFFF
    return h
