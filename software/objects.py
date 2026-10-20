"""
PC copy of the object table in Core/Src/objdict.c. Keep the same entries in the same order with the same
index, sub, type and flags: both sides hash them (table_hash()) and the GUI warns when they differ.
Everything else here (labels, units, display scale, limits, help) is presentation and only lives here.
"""
from protocol import T_U8, T_I8, T_U16, T_U32, T_I32, T_F32, T_I16

RO, STORED, PI, NOTRUN = 0x01, 0x02, 0x04, 0x08


class Obj:
    def __init__(self, index, sub, t, flags, key, label, unit="", scale=1.0, lo=None, hi=None, group="", help="",
                 decimals=4, choices=None):
        self.index, self.sub, self.type, self.flags = index, sub, t, flags
        self.key = key  # the old 's <key>' name, also the JSON key
        self.label = label
        self.unit = unit  # display unit
        self.scale = scale  # display = device value * scale
        self.lo, self.hi = lo, hi  # device units; the firmware checks the same limits
        self.group = group
        self.help = help
        self.decimals = 0 if t != T_F32 else decimals
        self.choices = choices  # {device value: name}: shown and edited by name (a drop-down)

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
        if self.choices and v in self.choices:
            return self.choices[v]
        if self.type != T_F32:
            return f"0x{v:02X}" if self.key in ("flags", "st_flags") else str(v)
        d = self.to_display(v)
        return f"{d:.{self.decimals}f}" if abs(d) < 1e7 else f"{d:.4g}"

    def choices_text(self):
        """'0 = encoder, 1 = ...'; just the values when the names are the numbers themselves (dir, can_kbit)"""
        return ", ".join(n if n.lstrip("+") == str(k) else f"{k} = {n}" for k, n in self.choices.items())

    def check(self, v):
        """None if v (device units) is acceptable, else a reason"""
        if v != v:
            return "not a number"
        if self.choices:
            return None if v in self.choices else "must be one of " + self.choices_text()
        if self.lo is not None and v < self.lo:
            return f"below {self.fmt(self.lo)} {self.unit}".strip()
        if self.hi is not None and v > self.hi:
            return f"above {self.fmt(self.hi)} {self.unit}".strip()
        return None


G_INFO, G_STATUS, G_DIAG = "Device info", "Status", "Diagnostics"
G_MOTOR, G_CUR, G_OUTER, G_LIM, G_SENSE, G_OBS, G_HFI, G_CAN = ("Motor", "Current loop", "Speed / position", "Limits",
                                                                "Current sensing", "Observer", "HF injection", "CAN")
CAN_KBIT = (20, 50, 125, 250, 500, 1000)
NMT_STATES = {0: "initializing", 4: "stopped", 5: "operational", 127: "pre-operational"}
FSA_STATES = {0: "not ready", 1: "switch on disabled", 2: "ready to switch on", 3: "switched on", 4: "operation enabled",
              5: "quick stop active", 6: "fault reaction", 7: "fault"}
CIA402_MODES = {0: "-", 1: "PP", 3: "PV", 6: "HM", 8: "CSP", 9: "CSV", 10: "CST"}
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
    Obj(0x4001, 13, T_U8, RO, "trip", "Trip", group=G_STATUS,
        help="0 none, 1 bus overvoltage, 2 overcurrent; STOP resets it once the motor has coasted down"),
    Obj(0x4001, 14, T_U8, RO, "trip_moe", "Trip switched the bridge off", group=G_STATUS),
    Obj(0x4001, 15, T_U8, RO, "coasting", "Coasting to a stop", group=G_STATUS,
        help="a stop holds zero current until the motor is below 30 rpm, then switches off (never short-brakes)"),
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
    Obj(0x4002, 10, T_U32, 0, "tlm_drop", "Telemetry drops", lo=0, hi=0, group=G_DIAG,
        help="frames that found the link queue full (USB host not reading fast enough)"),
    Obj(0x4002, 11, T_U32, 0, "ov_resumes", "Overvoltage resumes", lo=0, hi=0, group=G_DIAG,
        help="overvoltage coasts that ended by themselves once the bus came back down"),
    Obj(0x4002, 12, T_U32, 0, "hall_bad", "Hall bad states", lo=0, hi=0, group=G_DIAG,
        help="invalid Hall states (000/111) and jumps over a sector since boot"),
    Obj(0x4002, 13, T_U32, 0, "hyb_reseeds", "Hall + HFI re-seeds", lo=0, hi=0, group=G_DIAG,
        help="times HFI left the Hall sector and was put back on the Hall angle (angsrc 4); 0 in normal running"),
    # 0x4010 motor
    Obj(0x4010, 1, T_F32, STORED, "kv", "kv", "rpm/V", lo=0, hi=1e5, group=G_MOTOR, decimals=1,
        help="back-EMF constant, line-to-line peak; measured by the kv calibration. The observer derives lambda from it"),
    Obj(0x4010, 2, T_U8, STORED | NOTRUN, "pp", "Pole pairs", lo=1, hi=64, group=G_MOTOR, help="measured by 'e'"),
    Obj(0x4010, 3, T_I8, STORED | NOTRUN, "dir", "Encoder direction", lo=-1, hi=1, group=G_MOTOR, help="+1 or -1, measured by 'e'",
        choices={1: "+1", -1: "-1"}),
    Obj(0x4010, 4, T_U32, STORED | NOTRUN, "offset", "Electrical offset", "2^32=360°", lo=0, hi=2**32 - 1, group=G_MOTOR,
        help="electrical angle offset, measured by 'e'"),
    Obj(0x4010, 5, T_F32, STORED, "rd", "Rd", "mΩ", 1e3, 0, 100, G_MOTOR, "phase resistance, measured by 'r'", 2),
    Obj(0x4010, 6, T_F32, STORED, "rq", "Rq", "mΩ", 1e3, 0, 100, G_MOTOR, "= Rd (surface PM)", 2),
    Obj(0x4010, 7, T_F32, STORED, "ld", "Ld", "µH", 1e6, 0, 1, G_MOTOR, "measured by 'l'", 2),
    Obj(0x4010, 8, T_F32, STORED, "lq", "Lq", "µH", 1e6, 0, 1, G_MOTOR, "measured by 'q' (shaft held), else = Ld", 2),
    Obj(0x4010, 9, T_F32, STORED, "rated", "Rated current", "A", lo=0, hi=50, group=G_MOTOR, decimals=2,
        help="calibration currents are fractions of it; current commands are limited to it"),
    Obj(0x4010, 10, T_F32, STORED, "imax", "Max current", "A", lo=0, hi=50, group=G_MOTOR, decimals=2,
        help="trip after 120 µs above it: zero-current coast; bridge off if the current keeps rising (1.5x, or 5 ms)"),
    Obj(0x4010, 11, T_U32, STORED | NOTRUN, "flags", "Calibration flags", lo=0, hi=255, group=G_MOTOR,
        help="1 offset, 2 current cal, 4 R, 8 L, 16 PI tuned, 32 Hall table. Restored by JSON import"),
    # 0x4011 angle source
    Obj(0x4011, 1, T_U8, STORED | NOTRUN, "angsrc", "Angle source", lo=0, hi=4, group=G_MOTOR,
        help="0 encoder only; 1 encoder, observer blended in above olo (oblend); 2 sensorless: HFI and the "
             "observer, no encoder in the loop (position mode refused, the encoder is only logged); "
             "3 Hall sensors, observer above olo (oblend); 4 Hall + HFI: HFI at low speed, the Halls from "
             "hyblo to hybhi up, observer above olo. 3 and 4 need no encoder; position comes from them. "
             "Needs the calibrations for that source (sensorless: R, L, t, kv, saliency map, polarity; "
             "Hall: Hall table, t; Hall + HFI: also R, L, saliency map)",
        choices={0: "encoder", 1: "encoder + observer", 2: "sensorless", 3: "Hall", 4: "Hall + HFI"}),
    Obj(0x4011, 2, T_U16, STORED | NOTRUN, "hall1", "Hall state 1 begins", "2^16=360°", lo=0, hi=65535,
        group=G_MOTOR, help="electrical angle where Hall state 1 begins going forward, measured by the Hall calibration"),
    Obj(0x4011, 3, T_U16, STORED | NOTRUN, "hall2", "Hall state 2 begins", "2^16=360°", lo=0, hi=65535,
        group=G_MOTOR, help="electrical angle where Hall state 2 begins going forward, measured by the Hall calibration"),
    Obj(0x4011, 4, T_U16, STORED | NOTRUN, "hall3", "Hall state 3 begins", "2^16=360°", lo=0, hi=65535,
        group=G_MOTOR, help="electrical angle where Hall state 3 begins going forward, measured by the Hall calibration"),
    Obj(0x4011, 5, T_U16, STORED | NOTRUN, "hall4", "Hall state 4 begins", "2^16=360°", lo=0, hi=65535,
        group=G_MOTOR, help="electrical angle where Hall state 4 begins going forward, measured by the Hall calibration"),
    Obj(0x4011, 6, T_U16, STORED | NOTRUN, "hall5", "Hall state 5 begins", "2^16=360°", lo=0, hi=65535,
        group=G_MOTOR, help="electrical angle where Hall state 5 begins going forward, measured by the Hall calibration"),
    Obj(0x4011, 7, T_U16, STORED | NOTRUN, "hall6", "Hall state 6 begins", "2^16=360°", lo=0, hi=65535,
        group=G_MOTOR, help="electrical angle where Hall state 6 begins going forward, measured by the Hall calibration"),
    Obj(0x4011, 8, T_U8, RO, "hall_state", "Hall state", group=G_MOTOR,
        help="live: HALL1 = 1, HALL2 = 2, HALL3 = 4; 0 or 7 = no sensor (unplugged, unpowered)"),
    Obj(0x4011, 9, T_F32, STORED, "hyblo", "Hall + HFI handover start", "rpm", lo=0, hi=1e5, group=G_MOTOR, decimals=0,
        help="angsrc 4: below this HFI alone gives the angle; above it the Hall angle fades in"),
    Obj(0x4011, 10, T_F32, STORED, "hybhi", "Hall + HFI handover end", "rpm", lo=0, hi=1e5, group=G_MOTOR, decimals=0,
        help="angsrc 4: from here up the Halls alone (and the observer), the injection pauses"),
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
    Obj(0x4022, 2, T_F32, STORED, "vmax", "Bus max", "V", lo=0, hi=100, group=G_LIM, decimals=1,
        help="trip after 1 ms above it: zero-current coast (no regen, no braking), resumes once the bus is "
             "below vmax - vresume; outside a run STOP resets. 0 = off"),
    Obj(0x4022, 3, T_F32, STORED, "tmax", "Max temperature", "°C", lo=-40, hi=200, group=G_LIM, decimals=0, help="not used yet"),
    Obj(0x4022, 4, T_F32, STORED, "vclamp", "Regen clamp band", "V", lo=0, hi=20, group=G_LIM, decimals=1,
        help="braking current falls linearly to zero from vmax - vclamp to vmax, so decelerations stay within "
             "what the bus can take; motoring is never limited. 0 = off"),
    Obj(0x4022, 5, T_F32, STORED, "vresume", "Overvoltage resume margin", "V", lo=0, hi=20, group=G_LIM, decimals=1,
        help="after an overvoltage trip the run resumes once the bus has been below vmax - vresume for 10 ms"),
    # 0x4023 current sensing
    Obj(0x4023, 1, T_I16, STORED | NOTRUN, "coff_a", "Current offset A", "counts", lo=-300, hi=300, group=G_SENSE,
        help="ADC zero - 2048, stored by the current-offset calibration ('c'); the fallback when the boot "
             "measurement is not trusted"),
    Obj(0x4023, 2, T_I16, STORED | NOTRUN, "coff_b", "Current offset B", "counts", lo=-300, hi=300, group=G_SENSE),
    Obj(0x4023, 3, T_I16, STORED | NOTRUN, "coff_c", "Current offset C", "counts", lo=-300, hi=300, group=G_SENSE),
    Obj(0x4023, 4, T_I16, RO, "coff_a_use", "Offset A in use", "counts", group=G_DIAG),
    Obj(0x4023, 5, T_I16, RO, "coff_b_use", "Offset B in use", "counts", group=G_DIAG),
    Obj(0x4023, 6, T_I16, RO, "coff_c_use", "Offset C in use", "counts", group=G_DIAG),
    Obj(0x4023, 7, T_U8, RO, "coff_src", "Offsets from", group=G_DIAG, help="0 default, 1 stored, 2 measured at boot"),
    # 0x4030 observer
    Obj(0x4030, 1, T_U8, STORED, "oblend", "Blend observer", lo=0, hi=1, group=G_OBS,
        help="1: blend into angle and speed above olo; 0: run and log it only",
        choices={0: "log only", 1: "blend"}),
    Obj(0x4030, 2, T_F32, STORED, "ogain", "Flux correction gain", "rad/s", lo=0, hi=1e6, group=G_OBS, decimals=0),
    Obj(0x4030, 3, T_F32, STORED, "olo", "Blend start", "V", lo=0, hi=100, group=G_OBS, decimals=2,
        help="peak back-EMF where the observer starts to get weight"),
    Obj(0x4030, 4, T_F32, STORED, "ohi", "Blend full", "V", lo=0, hi=100, group=G_OBS, decimals=2),
    Obj(0x4030, 5, T_F32, STORED, "oftol", "Flux tolerance", "", lo=0, hi=1, group=G_OBS, decimals=2,
        help="|flux|/lambda must stay within 1 ± tol"),
    Obj(0x4030, 6, T_F32, STORED, "oerr", "Max observer error", "deg elec", lo=0, hi=180, group=G_OBS, decimals=0),
    # 0x4031 high-frequency injection
    Obj(0x4031, 1, T_U8, STORED | NOTRUN, "hmode", "HFI mode", lo=0, hi=1, group=G_HFI,
        help="0 off; 1 track and log while running (not during calibrations), in the encoder angle sources only. "
             "Sensorless (angsrc 2) always injects. Injection adds hid in speed/position mode",
        choices={0: "off", 1: "track and log"}),
    Obj(0x4031, 2, T_U8, STORED | NOTRUN, "hhalf", "Half wave", "PWM periods", lo=2, hi=8, group=G_HFI,
        help="injection flips every hhalf periods (2 = 6.25 kHz square wave). 1 cannot work: "
             "the samples would sit on the midpoints of the current triangle"),
    Obj(0x4031, 3, T_F32, STORED, "hv", "Injection voltage", "V", lo=0, hi=10, group=G_HFI, decimals=3,
        help="HF current step per PWM period is about hv x 40 us / L"),
    Obj(0x4031, 4, T_F32, STORED, "hbw", "PLL bandwidth", "Hz", lo=0, hi=500, group=G_HFI, decimals=0),
    Obj(0x4031, 5, T_F32, STORED, "hid", "d bias", "A", lo=0, hi=50, group=G_HFI, decimals=2,
        help="d-axis current while HFI runs (saturation saliency); the map stores hsal at this bias"),
    Obj(0x4031, 6, T_F32, STORED, "hsal", "Saliency", "%", 100.0, -1, 1, G_HFI,
        "(Lq-Ld)/(Lq+Ld) at hid, stored by the saliency map; 0 = not measured", 2),
    Obj(0x4031, 7, T_F32, STORED, "hpi", "Polarity pulse current", "A", lo=0, hi=50, group=G_HFI, decimals=2,
        help="aim of the +d/-d pulses; needs to be high enough to saturate the d axis"),
    # 0x4040 CAN settings (comm record in flash pages 60-61), used from the next communication reset
    Obj(0x4040, 1, T_U8, STORED, "can_id", "Node ID", lo=1, hi=127, group=G_CAN,
        help="CANopen node ID. Takes effect at the next communication reset: 'Apply CAN settings', NMT reset, or reboot"),
    Obj(0x4040, 2, T_U16, STORED, "can_kbit", "Bit rate", "kbit/s", lo=20, hi=1000, group=G_CAN,
        help="20, 50, 125, 250, 500 or 1000. Takes effect at the next communication reset",
        choices={k: str(k) for k in CAN_KBIT}),
    Obj(0x4040, 3, T_U8, RO, "can_id_act", "Node ID in use", group=G_DIAG),
    Obj(0x4040, 4, T_U16, RO, "can_kbit_act", "Bit rate in use", "kbit/s", group=G_DIAG),
    Obj(0x4040, 5, T_U8, NOTRUN, "can_apply", "Apply CAN settings", lo=1, hi=1, group=G_DIAG,
        help="write 1: CANopen communication reset with the node ID and bit rate above"),
    # 0x4041 CAN status
    Obj(0x4041, 1, T_U8, RO, "can_nmt", "NMT state", group=G_DIAG),
    Obj(0x4041, 2, T_U16, RO, "can_err", "CAN error state", group=G_DIAG),
    Obj(0x4041, 3, T_U32, 0, "can_rx", "CAN frames received", lo=0, hi=0, group=G_DIAG),
    Obj(0x4041, 4, T_U32, 0, "can_tx", "CAN frames sent", lo=0, hi=0, group=G_DIAG),
    Obj(0x4041, 5, T_U32, 0, "can_lost", "CAN RX overruns", lo=0, hi=0, group=G_DIAG,
        help="frames lost because the RX FIFO was full"),
    # 0x4042 CiA 402 drive profile
    Obj(0x4042, 1, T_U8, RO, "fsa", "CiA 402 state", group=G_DIAG),
    Obj(0x4042, 2, T_U8, RO, "fsa_refused", "Last refusal", group=G_DIAG, help="status code: why switch on / enable was refused"),
    Obj(0x4042, 3, T_U8, RO, "fsa_fault", "Fault", group=G_DIAG, help="status code of the fault in 0x603F"),
    Obj(0x4042, 4, T_U8, RO, "can_owner", "CAN has the motor", group=G_DIAG),
]

BY_KEY = {o.key: o for o in OBJECTS}
BY_ADDR = {(o.index, o.sub): o for o in OBJECTS}
PARAM_GROUPS = [G_MOTOR, G_CUR, G_OUTER, G_LIM, G_SENSE, G_OBS, G_HFI, G_CAN]

# PF_* in params.h
PF_ELEC_OFFSET_VALID, PF_CUR_CAL_VALID, PF_R_VALID, PF_L_VALID, PF_PI_VALID, PF_HALL_VALID = 1, 2, 4, 8, 16, 32


def table_hash(objs=OBJECTS):
    """FNV-1a 32 over (index lo, index hi, sub, type, flags) of every entry: objdict_init()"""
    h = 2166136261
    for o in objs:
        for b in (o.index & 0xFF, o.index >> 8, o.sub, o.type, o.flags):
            h = ((h ^ b) * 16777619) & 0xFFFFFFFF
    return h
