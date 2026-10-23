#!/usr/bin/env python3
"""
Headless command line for the drive: the Config tab's USB commands and the CANopen tab's master commands, without Qt.
One command from the arguments (several separated by ';'), a script (-f, one command per line, # comments), or an
interactive shell when no command is given. 'help' lists the commands.

Usage:
    python software/drivectl.py --usb COM7 status
    python software/drivectl.py --usb COM7 "set vlim 3000; vel 300; sleep 5; stop"
    python software/drivectl.py --can slcan:COM8 co state
    python software/drivectl.py --usb auto --can slcan:COM8 -f bench.txt
    python software/drivectl.py --usb COM7                 interactive shell
    options: --usb-baud 2000000  --bitrate 1000000  --node 1  -k (scripts: go on after a failed command)

USB values are in the Config tab's display units (e.g. hsal in %, choices by name or number); CANopen values are raw
OD values (increments, increments/s, per mille). Ctrl-C during sleep/cal/a wait sends STOP (USB) and disable voltage
(CANopen); during telem/watch/events it only ends that command. Without Ctrl-C the motor keeps doing what it was
told after the tool exits: end scripts with 'stop' / 'co disable'.
Exit code: 0 if every command succeeded, 1 otherwise, 2 if a link could not be opened.
"""
import argparse
import csv
import json
import logging
import os
import shlex
import struct
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import objects as O  # noqa: E402
import protocol as P  # noqa: E402

USB_VID_PID = (0x1209, 0x0001)  # as in fluxstudio.py

CALS = {  # Config tab letters and long names -> CAL which
    "c": P.CAL_CUR_OFF, "offsets": P.CAL_CUR_OFF, "e": P.CAL_ENC, "encoder": P.CAL_ENC, "H": P.CAL_HALL,
    "hall": P.CAL_HALL, "r": P.CAL_R, "l": P.CAL_LD, "ld": P.CAL_LD, "q": P.CAL_LQ, "lq": P.CAL_LQ, "t": P.CAL_TUNE,
    "tune": P.CAL_TUNE, "k": P.CAL_KV, "kv": P.CAL_KV, "z": P.CAL_ZERO, "zero": P.CAL_ZERO, "u": P.CAL_UVW,
    "uvw": P.CAL_UVW, "h": P.CAL_HFI_MAP, "hfimap": P.CAL_HFI_MAP, "p": P.CAL_HFI_POL, "hfipol": P.CAL_HFI_POL,
}
GROUPS = {"status": [O.G_STATUS], "diag": [O.G_DIAG], "info": [O.G_INFO], "params": O.PARAM_GROUPS}
NMT = {"op": "OPERATIONAL", "preop": "PRE-OPERATIONAL", "stop": "STOPPED", "reset": "RESET",
       "comm": "RESET COMMUNICATION"}
TELEM_DT = 10 / 25000.0  # one frame every 10th control cycle (CTRL_FS_HZ)
TELEM_SHOW = ["vin", "id", "iq", "rpm", "rpm_ref", "pos", "pos_ref"]  # columns printed when telem has no file


class Fail(Exception):
    """a command failed: the message is printed, the script stops (unless -k)"""


def err(text):
    print(f"error: {text}", file=sys.stderr, flush=True)


def open_usb(port, baud, skip=()):
    """-> UsbSerial that answers PING. 'auto' tries every port with the drive's VID:PID, except those in skip"""
    import co_link as C
    import serial.tools.list_ports
    ports = [port]
    if port == "auto":
        ports = [p.device for p in serial.tools.list_ports.comports()
                 if (p.vid, p.pid) == USB_VID_PID and p.device not in skip]
        if not ports:
            raise Fail("no drive on USB (VID:PID 1209:0001)")
    why = []
    for dev in ports:
        try:
            usb = C.UsbSerial(dev, baud)
        except Exception as e:  # noqa: BLE001 (busy, gone)
            why.append(f"{dev}: {e}")
            continue
        if usb.request(P.PING, timeout=1.0).status == P.ST_OK:
            if port == "auto":
                print(f"drive on {dev}", file=sys.stderr)
            return usb
        usb.close()
        why.append(f"{dev}: no reply to PING")
    raise Fail("; ".join(why))


def num(text):
    try:
        return int(text, 0)
    except ValueError:
        try:
            return float(text)
        except ValueError:
            raise Fail(f"not a number: {text}") from None


def native(key):
    """a key (vlim) or an address (0x4021:3)"""
    o = O.BY_KEY.get(key)
    if o is None and ":" in key:
        idx, _, sub = key.partition(":")
        try:
            o = O.BY_ADDR.get((int(idx, 16), int(sub, 0)))
        except ValueError:
            pass
    if o is None:
        raise Fail(f"unknown object '{key}' (get all lists them)")
    return o


def parse_native(o, text):
    """display units / choice name -> device value, checked against the limits"""
    if o.choices:
        by_name = {n.lower(): k for k, n in o.choices.items()}
        v = by_name[text.lower()] if text.lower() in by_name else num(text)
    elif o.type == P.T_F32:
        v = float(num(text)) / o.scale
    else:
        v = num(text)
        if not isinstance(v, int):
            raise Fail(f"{o.key} takes an integer")
    why = o.check(v)
    if why:
        raise Fail(f"{o.key} {text}: {why}")
    return v


def show(o, v):
    return f"{o.key:14s} {o.fmt(v)} {o.unit}".rstrip() + f"   ({o.label})"


class Cli:
    def __init__(self, usb=None, bus=None, node_id=1):
        self.usb, self.bus = usb, bus
        self.node = self.drive = None
        if bus:
            import co_link as C
            self.C = C
            self.node = bus.drives.get(node_id) or bus.add_drive(node_id)
            self.drive = C.Drive402(self.node)
        self.emcy_seen = 0
        self.cmds = {
            "help": (self.c_help, "[COMMAND]", "this list"),
            "ping": (self.c_ping, "", "USB: protocol version, table hash, PARAM_VERSION, build id"),
            "get": (self.c_get, "KEY...|all|params|status|diag|info", "USB: read objects"),
            "set": (self.c_set, "KEY VALUE ... | KEY=VALUE ...", "USB: write objects (RAM; 'save' stores them)"),
            "status": (self.c_status, "", "USB: the status objects"),
            "save": (self.c_save, "", "USB: parameters and CANopen record to flash (motor off)"),
            "cal": (self.c_cal, "WHICH [ARG] [--timeout S]",
                    "USB: calibration, waits for its end. WHICH: c e H r l q t k z u h p or "
                    "offsets encoder hall r ld lq tune kv zero uvw hfimap hfipol"),
            "cur": (self.c_cur, "ID IQ", "USB: current mode, A"),
            "vel": (self.c_vel, "RPM", "USB: speed mode"),
            "pos": (self.c_pos, "TURNS", "USB: position mode, turns from home"),
            "stop": (self.c_stop, "", "USB: stop (coasts above 30 rpm)"),
            "home": (self.c_home, "", "USB: the present position becomes 0"),
            "abort": (self.c_abort, "", "USB: abort a calibration"),
            "reboot": (self.c_reboot, "[dfu]", "USB: reset, or into the ST bootloader"),
            "export": (self.c_export, "FILE.json", "USB: stored parameters to JSON (the Config tab's format)"),
            "import": (self.c_import, "FILE.json", "USB: write the parameters from JSON (then 'save')"),
            "telem": (self.c_telem, "[SECONDS] [FILE.csv]", "USB: telemetry to CSV, or a few columns printed (0 = until Ctrl-C)"),
            "watch": (self.c_watch, "KEY... [-p PERIOD]", "USB: print objects every PERIOD s (0.5) until Ctrl-C"),
            "events": (self.c_events, "[SECONDS]", "print events / EMCYs as they come (0 = until Ctrl-C)"),
            "sleep": (self.c_sleep, "SECONDS", "wait, printing events; Ctrl-C stops the motor"),
            "co": (self.c_co, "SUBCOMMAND ...", "CANopen master, 'help co' lists the subcommands"),
        }
        self.co_cmds = {
            "read": (self.co_read, "OBJ...", "SDO read. OBJ: 0x6041, 0x608F:1, a native key (vlim) or an EDS name"),
            "write": (self.co_write, "OBJ VALUE", "SDO write (raw OD value)"),
            "nmt": (self.co_nmt, "[op|preop|stop|reset|comm]", "NMT command, or the state from the heartbeat"),
            "state": (self.co_state, "", "statusword, FSA state, mode, position, speed, fault"),
            "enable": (self.co_enable, "[pp|pv|hm|csp|csv|cst]", "6060, then shutdown -> switch on -> enable"),
            "disable": (self.co_disable, "", "disable voltage, waits until CAN lets go of the motor"),
            "shutdown": (self.co_shutdown, "", "controlword shutdown (0x06)"),
            "qstop": (self.co_qstop, "", "controlword quick stop"),
            "fault-reset": (self.co_fault_reset, "", "fault reset edge"),
            "halt": (self.co_halt, "on|off", "controlword halt bit"),
            "pv": (self.co_pv, "RPM", "profile velocity: enables PV if needed, then 60FF"),
            "pp": (self.co_pp, "TURNS [rel] [--timeout S]", "profile position move, waits for target reached"),
            "home": (self.co_home, "[METHOD] [OFFSET_TURNS]", "homing (37 = here), waits for attained"),
            "store": (self.co_store, "[all|comm|app]", "0x1010 store (default all)"),
            "emcy": (self.co_emcy, "", "the EMCY log since connect"),
        }

    # ------------------------------------------------------------------ plumbing
    def need_usb(self):
        if not self.usb:
            raise Fail("needs the USB link (--usb)")
        return self.usb

    def need_can(self):
        if not self.bus:
            raise Fail("needs the CAN adapter (--can)")
        return self.drive

    def req(self, mtype, payload=b"", what=""):
        r = self.need_usb().request(mtype, payload)
        if not r.ok:
            raise Fail(f"{what or 'request'}: {P.status_text(r.status)}")
        return r

    def pump(self):
        """print the events and EMCYs queued since the last call; -> [(tok, code, vals)]"""
        evs = self.usb.events() if self.usb else []
        for tok, code, vals in evs:
            print(f"  event: {P.event_text(code, vals)}", flush=True)
        if self.node:
            em, self.emcy_seen = self.node.emcy.since(self.emcy_seen)
            for t, code, reg, bit, info, text, at_connect in em:
                print(f"  EMCY 0x{code:04X} reg 0x{reg:02X}: {text}" + (" (at connect)" if at_connect else ""),
                      flush=True)
        return evs

    def wait(self, seconds, until=None):
        """seconds (0 = forever), printing events; until(events) -> True ends it early"""
        end = time.time() + seconds if seconds else float("inf")
        while time.time() < end:
            evs = self.pump()
            if until and until(evs):
                return True
            time.sleep(0.02)
        return False

    def emergency(self):
        """Ctrl-C in a motion command"""
        print("interrupted: stopping", file=sys.stderr, flush=True)
        if self.usb:
            print(f"  USB STOP: {P.status_text(self.usb.stop())}")
        if self.drive:
            try:
                self.drive.disable_voltage()
                print("  CANopen: disable voltage")
            except self.C.CoError as e:
                err(f"CANopen disable voltage: {e}")

    def run(self, line):
        """-> True if the command succeeded"""
        words = shlex.split(line, comments=True, posix=True)
        if not words:
            return True
        name, args = words[0], words[1:]
        if name not in self.cmds:
            err(f"unknown command '{name}' (help)")
            return False
        try:
            self.cmds[name][0](args)
            self.pump()
            return True
        except Fail as e:
            err(str(e))
        except KeyboardInterrupt:
            self.emergency()
        except Exception as e:  # noqa: BLE001 (CoError, a port that went away...)
            err(f"{type(e).__name__}: {e}")
        return False

    # ------------------------------------------------------------------ general
    def c_help(self, a):
        def table(cmds, prefix=""):
            for n, (_, u, h) in cmds.items():
                print(f"  {prefix}{n} {u}".ljust(44) + f" {h}")
        if a and a[0] == "co":
            table(self.co_cmds, "co ")
        elif a and a[0] in self.cmds:
            _, u, h = self.cmds[a[0]]
            print(f"  {a[0]} {u}\n      {h}")
        else:
            table(self.cmds)

    def c_events(self, a):
        try:
            self.wait(float(num(a[0])) if a else 0)
        except KeyboardInterrupt:
            pass

    def c_sleep(self, a):
        if len(a) != 1:
            raise Fail("sleep SECONDS")
        self.wait(float(num(a[0])))

    # ------------------------------------------------------------------ USB
    def c_ping(self, a):
        pv, h, parv, bid = P.parse_ping(self.req(P.PING, what="ping").payload)
        print(f"protocol {pv}, table hash {h:08X}, PARAM_VERSION {parv}, build {bid:08X}")
        if pv != P.PROTO_VERSION:
            print(f"  warning: this tool speaks protocol {P.PROTO_VERSION}")
        if h != O.table_hash():
            print(f"  warning: objects.py hashes to {O.table_hash():08X}: the object tables differ (update the firmware "
                  "or the PC software)")

    def objs(self, names):
        out = []
        for n in names:
            if n == "all":
                out += O.OBJECTS
            elif n in GROUPS:
                out += [o for o in O.OBJECTS if o.group in GROUPS[n] and (n != "params" or o.stored)]
            else:
                out.append(native(n))
        return out

    def read(self, o):
        r = self.req(P.READ, P.p_read(o.index, o.sub), f"read {o.key}")
        return P.unpack_value(o.type, r.payload)

    def c_get(self, a):
        if not a:
            raise Fail("get KEY... (or all, params, status, diag, info)")
        usb, bad = self.need_usb(), 0
        for o in self.objs(a):
            r = usb.request(P.READ, P.p_read(o.index, o.sub))
            if r.ok:
                print(show(o, P.unpack_value(o.type, r.payload)))
            else:
                bad += 1
                print(f"{o.key:14s} -- {P.status_text(r.status)}")
        if bad:
            raise Fail(f"{bad} read(s) failed")

    def c_status(self, a):
        self.c_get(["status"])

    def c_set(self, a):
        pairs = []
        rest = list(a)
        while rest:
            w = rest.pop(0)
            if "=" in w:
                k, _, v = w.partition("=")
            elif rest:
                k, v = w, rest.pop(0)
            else:
                raise Fail(f"set {w}: no value")
            pairs.append((native(k), v))
        if not pairs:
            raise Fail("set KEY VALUE ...")
        for o, text in pairs:
            if o.ro:
                raise Fail(f"{o.key} is read-only")
            v = parse_native(o, text)
            self.req(P.WRITE, P.p_write(o.index, o.sub, o.type, v), f"write {o.key}")
            print(show(o, self.read(o)))

    def c_save(self, a):
        self.req(P.SAVE, what="save")
        print("saved to flash")

    def c_cal(self, a):
        timeout = 600.0
        if "--timeout" in a:
            i = a.index("--timeout")
            timeout = float(num(a[i + 1]))
            a = a[:i] + a[i + 2:]
        if not a or a[0] not in CALS:
            raise Fail("cal WHICH [ARG]: " + " ".join(CALS))
        which = CALS[a[0]]
        arg = float(num(a[1])) if len(a) > 1 else 0.0
        self.need_usb()
        self.pump()
        self.req(P.CAL, P.p_cal(which, arg), f"cal {P.CAL_NAMES[which]}")
        print(f"{P.CAL_NAMES[which]}: started" + (f" (arg {arg:g})" if arg else ""), flush=True)
        done = []

        def end(evs):
            done.extend(v for _, c, v in evs if c == P.EV_DONE)
            return bool(done)
        try:
            ok = self.wait(timeout, end)
        except KeyboardInterrupt:
            print("interrupted: ABORT", file=sys.stderr, flush=True)
            self.usb.request(P.ABORT)
            self.wait(10.0, end)
            raise Fail("calibration aborted") from None
        if not ok:
            self.usb.request(P.ABORT)
            raise Fail(f"no end of the calibration after {timeout:g} s: aborted")
        if int(done[0][0]) != P.ST_OK:
            raise Fail(f"{P.CAL_NAMES[which]}: {P.status_text(int(done[0][0]))}")

    def c_cur(self, a):
        if len(a) != 2:
            raise Fail("cur ID IQ")
        self.req(P.RUN_CUR, P.p_run_cur(float(num(a[0])), float(num(a[1]))), "run current")

    def c_vel(self, a):
        if len(a) != 1:
            raise Fail("vel RPM")
        self.req(P.RUN_VEL, P.p_run_vel(float(num(a[0]))), "run speed")

    def c_pos(self, a):
        if len(a) != 1:
            raise Fail("pos TURNS")
        num(a[0])
        self.req(P.RUN_POS, P.p_run_pos(a[0]), "run position")

    def c_stop(self, a):
        self.req(P.STOP, what="stop")

    def c_home(self, a):
        self.req(P.HOME, what="home")

    def c_abort(self, a):
        self.req(P.ABORT, what="abort")

    def c_reboot(self, a):
        dfu = bool(a) and a[0] == "dfu"
        self.req(P.REBOOT, P.p_reboot(dfu), "reboot")
        print("rebooting into the ST bootloader" if dfu else "rebooting: the port comes back in 1-2 s")

    def c_export(self, a):
        if len(a) != 1:
            raise Fail("export FILE.json")
        pv, h, parv, bid = P.parse_ping(self.req(P.PING, what="ping").payload)
        objs = [o for o in O.OBJECTS if o.stored]
        doc = {"format": "driverTCC-params", "param_version": parv, "build_id": f"{bid:08X}",
               "saved": time.strftime("%Y-%m-%d %H:%M:%S"),
               "values": {o.key: self.read(o) for o in objs},
               "units": {o.key: o.json_unit for o in objs}}
        with open(a[0], "w", encoding="utf-8") as f:
            json.dump(doc, f, indent=1)
        print(f"exported {len(doc['values'])} values to {a[0]}")

    def c_import(self, a):
        if len(a) != 1:
            raise Fail("import FILE.json")
        try:
            with open(a[0], encoding="utf-8") as f:
                doc = json.load(f)
            vals = doc["values"]
        except (OSError, ValueError, KeyError, TypeError) as e:
            raise Fail(f"cannot read {a[0]}: {e}") from None
        n, bad = 0, []
        for k, v in vals.items():
            o = O.BY_KEY.get(k)
            why = ("unknown" if o is None or not o.stored or o.ro else
                   o.unit_changed(doc) or (o.check(v) if isinstance(v, (int, float)) else "not a number"))
            if not why:
                r = self.need_usb().request(P.WRITE, P.p_write(o.index, o.sub, o.type, v))
                why = None if r.ok else P.status_text(r.status)
            if why:
                bad.append(f"{k} ({why})")
            else:
                n += 1
        print(f"wrote {n} value(s) from {a[0]}; 'save' stores them")
        if bad:
            raise Fail("not written: " + ", ".join(bad))

    def c_telem(self, a):
        usb = self.need_usb()
        secs = float(num(a[0])) if a and not a[0].lower().endswith(".csv") else 0.0
        path = next((x for x in a if x.lower().endswith(".csv")), None)
        f = open(path, "w", newline="", encoding="utf-8") if path else None
        w = csv.writer(f) if f else None
        cols = [P.TELEM_NAMES.index(n) for n in TELEM_SHOW]
        state = {"n": 0, "gaps": 0, "seq": None, "last": None, "k": 0}

        def on(fr):
            seq = struct.unpack_from("<H", fr, 2)[0]
            v = struct.unpack_from("<16f", fr, 4)
            if state["seq"] is not None:
                d = (seq - state["seq"]) & 0xFFFF
                state["gaps"] += d != 1
                state["k"] += d  # t from seq: USB delivers the frames in bursts
            state["seq"] = seq
            state["n"] += 1
            state["last"] = v
            if w:
                w.writerow([f"{state['k'] * TELEM_DT:.4f}", seq] + [f"{x:.6g}" for x in v])
        if w:
            w.writerow(["t", "seq"] + P.TELEM_NAMES)
        else:
            print("  ".join(f"{n:>9s}" for n in TELEM_SHOW))
        usb.on_telem = on
        end = time.time() + secs if secs else float("inf")
        t_print = 0.0
        try:
            while time.time() < end:
                self.pump()
                if not w and state["last"] and time.time() - t_print >= 0.1:
                    t_print = time.time()
                    print("  ".join(f"{state['last'][i]:9.3f}" for i in cols), flush=True)
                time.sleep(0.02)
        except KeyboardInterrupt:
            pass
        finally:
            usb.on_telem = None
            if f:
                f.close()
        print(f"{state['n']} frames, {state['gaps']} seq gaps" + (f" -> {path}" if path else ""))
        if not state["n"]:
            raise Fail("no telemetry")

    def c_watch(self, a):
        period = 0.5
        if "-p" in a:
            i = a.index("-p")
            period = float(num(a[i + 1]))
            a = a[:i] + a[i + 2:]
        objs = self.objs(a or ["status"])
        self.need_usb()
        print("  ".join(f"{o.key:>10s}" for o in objs))
        try:
            while True:
                print("  ".join(f"{o.fmt(self.read(o)):>10s}" for o in objs), flush=True)
                self.wait(period)
        except KeyboardInterrupt:
            pass

    # ------------------------------------------------------------------ CANopen
    def c_co(self, a):
        if not a or a[0] not in self.co_cmds:
            raise Fail("co SUBCOMMAND (help co)")
        self.need_can()
        self.co_cmds[a[0]][0](a[1:])

    def co_obj(self, text):
        """-> (index, sub, native Obj or None)"""
        if text in O.BY_KEY:
            o = O.BY_KEY[text]
            return o.index, o.sub, o
        idx, _, sub = text.partition(":")
        try:
            i, s = int(idx, 16), int(sub, 0) if sub else 0
            return i, s, O.BY_ADDR.get((i, s))
        except ValueError:
            pass
        od = self.node.node.object_dictionary
        name, _, subname = text.partition(".")
        try:
            e = od[name]
            if subname:
                e = e[subname]
            elif not isinstance(e, self.C.canopen.objectdictionary.ODVariable):
                e = e[0]
        except KeyError:
            raise Fail(f"unknown object '{text}' (0xIIII[:sub], a native key, or an EDS name)") from None
        return e.index, e.subindex, O.BY_ADDR.get((e.index, e.subindex))

    def co_read(self, a):
        if not a:
            raise Fail("co read OBJ...")
        for t in a:
            i, s, o = self.co_obj(t)
            v = self.node.read(i, s)
            name = self.node.od_entry(i, s).name
            if o:
                print(f"0x{i:04X}:{s}  {show(o, v)}")
            elif isinstance(v, int):
                print(f"0x{i:04X}:{s}  {name} = {v} (0x{v:X})")
            else:
                print(f"0x{i:04X}:{s}  {name} = {v}")

    def co_write(self, a):
        if len(a) != 2:
            raise Fail("co write OBJ VALUE")
        i, s, o = self.co_obj(a[0])
        e = self.node.od_entry(i, s)
        v = num(a[1])
        if e.data_type in (0x08, 0x11):  # REAL32, REAL64
            v = float(v)
        elif not isinstance(v, int):
            raise Fail(f"0x{i:04X}:{s} takes an integer")
        self.node.write(i, s, v)
        self.co_read([f"0x{i:04X}:{s}"])

    def hb_state(self):
        """NMT state from the heartbeat, waiting one period for the first one after connect"""
        n = self.node
        end = time.time() + 1.5 * n.hb_period + 0.1
        while not n.hb_alive() and time.time() < end:
            time.sleep(0.02)
        return n.nmt_state + ("" if n.hb_alive() else " (no heartbeat)")

    def co_nmt(self, a):
        if a:
            if a[0] not in NMT:
                raise Fail("co nmt op|preop|stop|reset|comm")
            self.node.nmt(NMT[a[0]])
            time.sleep(0.2 if a[0] in ("op", "preop", "stop") else 1.0)
        print(f"NMT {self.hb_state()}")

    def co_state(self, a):
        C, d, u = self.C, self.drive, self.node.units
        sw = d.sw()
        mode = self.node.read(0x6061)
        st, bits = C.decode_sw(sw, mode)
        on = ", ".join(n for b, n, v in bits if v and b not in (5,))
        print(f"NMT {self.hb_state()}, {C.FSA_NAMES.get(st, '?')} (sw 0x{sw:04X}{': ' + on if on else ''})")
        print(f"mode {C.MODES.get(mode, mode)}, position {u.inc_to_turn(self.node.read(0x6064)):.4f} turns, "
              f"speed {u.ips_to_rpm(self.node.read(0x606C)):.1f} rpm, CAN has the motor: "
              f"{'yes' if self.node.read_native('can_owner') else 'no'}")
        if st in (C.FAULT, C.FRA):
            print(f"fault {d.fault_text()[1]}")
        st_r, why = d.refusal()
        if st_r:
            print(f"last refusal: {why}")

    def co_mode(self, text):
        m = {n.lower(): k for k, n in self.C.MODES.items() if n != "-"}
        if text.lower() not in m:
            raise Fail("mode: " + " ".join(m))
        return m[text.lower()]

    def co_enable(self, a):
        mode = self.co_mode(a[0]) if a else None
        self.drive.goto_enabled(mode)
        self.co_state([])

    def co_disable(self, a):
        if not self.drive.stop_all(15.0):
            raise Fail("CAN still has the motor after 15 s")
        print("voltage disabled")

    def co_shutdown(self, a):
        self.drive.shutdown()

    def co_qstop(self, a):
        self.drive.quick_stop()

    def co_fault_reset(self, a):
        self.drive.fault_reset()
        self.co_state([])

    def co_halt(self, a):
        if not a or a[0] not in ("on", "off"):
            raise Fail("co halt on|off")
        self.drive.halt(a[0] == "on")

    def ensure(self, mode):
        C, d = self.C, self.drive
        if C.fsa_state(d.sw()) != C.OE or self.node.read(0x6061) != mode:
            d.goto_enabled(mode)

    def co_pv(self, a):
        if len(a) != 1:
            raise Fail("co pv RPM")
        self.ensure(self.C.MODE_PV)
        self.node.write(0x60FF, 0, int(round(self.node.units.rpm_to_ips(float(num(a[0]))))))

    def co_pp(self, a):
        timeout = 30.0
        if "--timeout" in a:
            i = a.index("--timeout")
            timeout = float(num(a[i + 1]))
            a = a[:i] + a[i + 2:]
        if not a:
            raise Fail("co pp TURNS [rel]")
        rel = "rel" in a[1:]
        self.ensure(self.C.MODE_PP)
        if not self.drive.pp_move(int(round(self.node.units.turn_to_inc(float(num(a[0]))))), relative=rel):
            raise Fail("set-point not acknowledged")
        if not self.drive.wait_reached(timeout):
            raise Fail(f"target not reached after {timeout:g} s")
        print(f"reached, position {self.node.units.inc_to_turn(self.node.read(0x6064)):.4f} turns")

    def co_home(self, a):
        method = int(num(a[0])) if a else 37
        off = int(round(self.node.units.turn_to_inc(float(num(a[1]))))) if len(a) > 1 else 0
        attained, error = self.drive.home(method, off)
        if error or not attained:
            raise Fail("homing error" if error else "homing not attained")
        print("homing attained")

    def co_store(self, a):
        sub = {"all": 1, "comm": 2, "app": 3}.get(a[0] if a else "all")
        if sub is None:
            raise Fail("co store [all|comm|app]")
        self.node.store(sub)
        print("stored")

    def co_emcy(self, a):
        em = self.node.emcy.snapshot()
        for t, code, reg, bit, info, text, at_connect in em:
            print(f"{time.strftime('%H:%M:%S', time.localtime(t))}  0x{code:04X} reg 0x{reg:02X}  {text}")
        self.emcy_seen = self.node.emcy.total
        if not em:
            print("no EMCY")


def lines_of(text):
    return [s.strip() for s in text.split(";")]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", nargs=argparse.REMAINDER, help="one command, or several separated by ';'")
    ap.add_argument("--usb", help="config link port (USB CDC or USART1), e.g. COM7, or 'auto'")
    ap.add_argument("--usb-baud", type=int, default=2000000)
    ap.add_argument("--can", help="interface:channel, e.g. slcan:COM8, pcan:PCAN_USBBUS1, socketcan:can0")
    ap.add_argument("--bitrate", type=int, default=1000000)
    ap.add_argument("--node", type=int, default=1)
    ap.add_argument("-f", "--file", help="script: one command per line")
    ap.add_argument("-k", "--keep-going", action="store_true", help="scripts: go on after a failed command")
    a = ap.parse_args()
    # line-buffered: errors (stderr) and output stay in order when piped; UTF-8: units like ohm, deg (cp1252 pipes)
    sys.stdout.reconfigure(line_buffering=True, encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    logging.getLogger("canopen").setLevel(logging.CRITICAL)  # its SDO timeout logs: DriveNode retries and reports

    usb = bus = None
    try:
        if a.usb:
            usb = open_usb(a.usb, a.usb_baud, skip={a.can.partition(":")[2]} if a.can else ())
        if a.can:
            import co_link as C
            iface, _, chan = a.can.partition(":")
            bus = C.CoBus().connect(iface, chan, a.bitrate)
            bus.start_heartbeat()
    except Fail as e:
        err(str(e))
        return 2
    except Exception as e:  # noqa: BLE001 (port busy, no adapter...)
        err(f"cannot open the link: {e}")
        return 2
    cli = Cli(usb, bus, a.node)
    if bus:
        try:
            cli.node.refresh_units()
        except Exception as e:  # noqa: BLE001
            err(f"node {a.node}: {e}")

    ok = True
    try:
        if a.file or a.command:
            if a.file:
                with open(a.file, encoding="utf-8") as f:
                    script = [s for ln in f for s in lines_of(ln)]
            else:
                words = a.command  # one quoted string, or words with ';' as separate words
                script = lines_of(words[0] if len(words) == 1 else " ".join(w if w == ";" else shlex.quote(w) for w in words))
            for ln in script:
                if not ln or ln.startswith("#"):
                    continue
                if a.file:
                    print(f"> {ln}", flush=True)
                if not cli.run(ln):
                    ok = False
                    if not a.keep_going:
                        break
        else:
            print("drivectl: 'help' lists the commands, Ctrl-D / Ctrl-Z Enter or 'quit' ends")
            while True:
                cli.pump()
                try:
                    ln = input("> ")
                except EOFError:
                    break
                except KeyboardInterrupt:
                    print()
                    continue
                if ln.strip() in ("quit", "exit"):
                    break
                for s in lines_of(ln):
                    ok = cli.run(s) and ok
    finally:
        if bus:
            bus.disconnect()
        if usb:
            usb.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
