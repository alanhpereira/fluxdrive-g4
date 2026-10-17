#!/usr/bin/env python3
"""
CANopen test runner (docs/canopen-pc-handoff.md: Test sequences). The tests are plain functions in
software/cotests/*.py, registered with @test; the plotter's CANopen tab runs the same Runner on a thread.

Usage:
    python software/cotest.py --can slcan:COM8 [--usb COM6] [names...]     run all, or the named tests
    python software/cotest.py --list
    options: --bitrate 1000000  --node 1  --usb-baud 2000000  --no-motion (skip the tests that spin the motor)

Results: software/cotest_runs/<date>_<time>.json and .csv (git-ignored).
Every test ends, in a finally, with disable voltage, 6060 back to its start value, the OD values it changed
restored, NMT pre-operational and (with --usb) a STOP over USB plus the loop-time / encoder-late checks.
"""
import argparse
import csv
import importlib
import json
import os
import pkgutil
import sys
import threading
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import co_link as C  # noqa: E402
import protocol as P  # noqa: E402

RUNS = os.path.join(HERE, "cotest_runs")
LOOP_MAX_US = 19.0  # currentLoop() must end before the next encoder read
CPU_MHZ = 170.0

REGISTRY = []  # Test, in definition order
sys.modules.setdefault("cotest", sys.modules[__name__])  # run as a script, cotests/* still register here


class Test:
    def __init__(self, fn, name, usb, motion, doc):
        self.fn, self.name, self.usb, self.motion, self.doc = fn, name, usb, motion, doc

    def __repr__(self):
        return f"Test({self.name})"


def test(name=None, usb=False, motion=False):
    """usb: needs the config link (skipped without it); motion: spins the motor"""
    def deco(fn):
        doc = (fn.__doc__ or "").strip().splitlines()
        REGISTRY.append(Test(fn, name or fn.__name__.removeprefix("t_"), usb, motion, doc[0] if doc else ""))
        return fn
    return deco


def discover():
    """imports software/cotests/*.py once; -> list of Test"""
    if not REGISTRY:
        import cotests
        for m in sorted(pkgutil.iter_modules(cotests.__path__), key=lambda m: m.name):
            if not m.name.startswith("_"):
                importlib.import_module(f"cotests.{m.name}")
    return list(REGISTRY)


class Skip(Exception):
    pass


class Cancelled(Exception):
    pass


class Ctx:
    """what a test gets: .bus .node .drive .units .usb (UsbSync or None), check(), log(), note() and helpers"""

    def __init__(self, bus, node_id, usb, log, cancel):
        self.bus, self.node_id, self.usb, self._log, self.cancel = bus, node_id, usb, log, cancel
        self.node = bus.drives.get(node_id) or bus.add_drive(node_id)
        self.drive = C.Drive402(self.node)
        self.checks = []
        self.notes = {}
        self._restore = []  # (index, sub, value), restored in reverse
        self._usb_restore = []  # (key, value)

    @property
    def units(self):
        return self.node.units

    # ---- reporting
    def log(self, text):
        self._log(f"    {text}")

    def check(self, name, cond, detail=""):
        ok = bool(cond)
        self.checks.append((name, ok, str(detail)))
        self._log(f"    {'PASS' if ok else 'FAIL'}  {name}" + (f"  ({detail})" if detail != "" else ""))
        return ok

    def note(self, key, value):
        self.notes[key] = value
        self._log(f"    note  {key} = {value}")

    def need_usb(self):
        if not self.usb:
            raise Skip("needs the USB/serial link (--usb)")

    # ---- timing
    def sleep(self, s):
        end = time.time() + s
        while time.time() < end:
            if self.cancel.is_set():
                raise Cancelled()
            time.sleep(min(0.02, max(0.0, end - time.time())))

    def wait(self, cond, timeout, poll=0.01):
        """-> True once cond() is true, False on timeout"""
        end = time.time() + timeout
        while True:
            if cond():
                return True
            if time.time() >= end:
                return False
            self.sleep(poll)

    # ---- OD changes that are put back afterwards
    def set(self, index, sub, value):
        """write, remembering the value before (once per object)"""
        if not any(r[:2] == (index, sub) for r in self._restore):
            self._restore.append((index, sub, self.node.read(index, sub)))
        self.node.write(index, sub, value)

    def usb_set(self, key, value):
        self.need_usb()
        if not any(r[0] == key for r in self._usb_restore):
            self._usb_restore.append((key, self.usb.read(key)))
        self.usb.write(key, value)

    # ---- bus helpers
    def operational(self, sync_period=0.005):
        """NMT operational, the PDO maps read, SYNC running (TPDO2/3 come on SYNC)"""
        self.node.nmt("OPERATIONAL")
        self.sleep(0.05)
        self.node.pdo_read()
        self.bus.start_sync(sync_period)
        self.sleep(0.05)

    def rpm(self):
        """latest 606C from TPDO2, rpm"""
        t, v = self.node.series.get((0x606C, 0))
        return self.units.ips_to_rpm(v[-1]) if len(v) else float("nan")

    def pos(self):
        """latest 6064 from TPDO2, increments"""
        t, v = self.node.series.get((0x6064, 0))
        return int(v[-1]) if len(v) else None

    def signal(self, index, sub=0, since=None):
        """(t, v) of a TPDO signal, optionally only after the wall time `since`"""
        t, v = self.node.series.get((index, sub))
        if since is not None:
            k = t >= since
            t, v = t[k], v[k]
        return t, v

    def emcys(self, since):
        return [e for e in self.node.emcy.snapshot() if e[0] >= since]

    def reconnect(self, bitrate=None, node_id=None):
        """reopen the adapter (other bit rate) and/or talk to another node ID"""
        b = self.bus
        if bitrate and bitrate != b.bitrate:
            b.reconnect(bitrate)
        self.node_id = node_id or self.node_id
        self.node = b.drives.get(self.node_id) or b.add_drive(self.node_id)
        self.drive = C.Drive402(self.node)
        self.node.refresh_units()

    # ---- the start and end of every test
    def clear_fault(self):
        """wait for a fault reaction to end, then fault reset"""
        d = self.drive
        d.wait_sw(lambda sw: C.fsa_state(sw) != C.FRA, 10.0, poll=0.05)
        if C.fsa_state(d.sw()) == C.FAULT:
            d.fault_reset()
            d.wait_state(C.SOD, 1.0)

    def prepare(self):
        """no fault, motor at standstill (a free coast from a few hundred rpm takes seconds)"""
        self.clear_fault()
        if C.fsa_state(self.drive.sw()) == C.FAULT:
            raise C.CoError(f"drive in fault: {self.drive.fault_text()[1]}")
        u = self.node.units
        if not self.wait(lambda: abs(u.ips_to_rpm(self.node.read(0x606C))) < 5.0, 15.0, poll=0.1):
            raise C.CoError(f"motor still turning at {u.ips_to_rpm(self.node.read(0x606C)):.0f} rpm")

    def cleanup(self, mode0):
        errs = []
        self.bus.stop_sync()

        def tryit(what, fn):
            try:
                fn()
            except Exception as e:  # noqa: BLE001
                errs.append(f"{what}: {e}")
        tryit("disable voltage", lambda: self.drive.stop_all(15.0, standstill_rpm=5.0))
        if self.usb:
            tryit("USB STOP", self.usb.stop)
        tryit("fault reset", self.clear_fault)
        for key, v in reversed(self._usb_restore):
            tryit(f"USB restore {key}", lambda k=key, v=v: self.usb.write(k, v))
        for idx, sub, v in reversed(self._restore):
            tryit(f"restore 0x{idx:04X}:{sub}", lambda i=idx, s=sub, v=v: self.node.write(i, s, v))
        if mode0 is not None:
            tryit("6060", lambda: self.node.write(0x6060, 0, mode0))
        tryit("NMT pre-operational", lambda: self.node.nmt("PRE-OPERATIONAL"))
        return errs


class Runner:
    """runs tests one after the other on the caller's thread; log(text) gets every line"""

    def __init__(self, bus, node_id=1, usb=None, log=print, cancel=None, motion=True):
        self.bus, self.node_id, self.usb, self.log = bus, node_id, usb, log
        self.cancel = cancel or threading.Event()
        self.motion = motion
        self.results = []
        self.path = None

    def run(self, tests):
        start = time.strftime("%Y%m%d_%H%M%S")
        if not self.bus.hb:
            self.bus.start_heartbeat()
        for t in tests:
            if self.cancel.is_set():
                break
            self.results.append(self.run_one(t))
        self.path = self.save(start)
        n_pass = sum(r["result"] == "pass" for r in self.results)
        n_fail = sum(r["result"] in ("fail", "error") for r in self.results)
        n_skip = sum(r["result"] == "skip" for r in self.results)
        self.log(f"== {n_pass} passed, {n_fail} failed, {n_skip} skipped   -> {self.path}.json")
        return self.results

    def run_one(self, t):
        self.log(f"-- {t.name}: {t.doc}")
        r = {"name": t.name, "doc": t.doc, "result": "pass", "error": "", "checks": [], "notes": {}, "t0": time.time()}
        if t.motion and not self.motion:
            r["result"] = "skip"
            r["error"] = "motion tests disabled"
            self.log("    skip  (motion tests disabled)")
            return r
        ctx = Ctx(self.bus, self.node_id, self.usb, self.log, self.cancel)
        mode0 = None
        try:
            if t.usb:
                ctx.need_usb()
            ctx.node.refresh_units()
            ctx.prepare()
            ctx.node.series.clear()  # no samples from the test before
            mode0 = ctx.node.read(0x6060)
            if self.usb:
                for k in ("loop_cyc_max", "enc_late"):
                    self.usb.write(k, 0)
            t.fn(ctx)
        except Skip as e:
            r["result"], r["error"] = "skip", str(e)
            self.log(f"    skip  {e}")
        except Cancelled:
            r["result"], r["error"] = "error", "cancelled"
            self.log("    cancelled")
        except C.CoError as e:
            r["result"], r["error"] = "error", str(e)
            self.log(f"    ERROR {e}")
        except Exception as e:  # noqa: BLE001
            r["result"], r["error"] = "error", f"{type(e).__name__}: {e}"
            self.log(f"    ERROR {type(e).__name__}: {e}")
            self.log("    " + traceback.format_exc().strip().replace("\n", "\n    "))
        finally:
            errs = ctx.cleanup(mode0)
            for e in errs:
                self.log(f"    cleanup: {e}")
            if self.usb and r["result"] != "skip":
                try:
                    mx = self.usb.read("loop_cyc_max") / CPU_MHZ
                    ctx.check("loop time max", mx <= LOOP_MAX_US, f"{mx:.1f} us")
                    late = self.usb.read("enc_late")
                    ctx.check("encoder late", late == 0, late)
                except C.CoError as e:
                    self.log(f"    diagnostics: {e}")
        if r["result"] == "error":
            r["trace"] = self.dump_trace(t.name)
        r["checks"] = ctx.checks
        r["notes"] = ctx.notes
        r["duration"] = round(time.time() - r["t0"], 2)
        if r["result"] == "pass" and not all(ok for _, ok, _ in ctx.checks):
            r["result"] = "fail"
        self.log(f"   {r['result'].upper()}  {t.name}  ({r['duration']:.1f} s)")
        return r

    def dump_trace(self, name, seconds=3.0):
        """the last seconds of bus traffic, as a CSV next to the results; -> path"""
        os.makedirs(RUNS, exist_ok=True)
        path = os.path.join(RUNS, time.strftime("%Y%m%d_%H%M%S_") + name + "_trace.csv")
        tr = self.bus.trace.snapshot()
        if not tr:
            return ""
        t_end = tr[-1][0]
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t", "dir", "cob_id", "kind", "data", "decoded"])
            for t, cid, data, d, fl in tr:
                if t >= t_end - seconds:
                    w.writerow([f"{t - t_end:.4f}", d, f"0x{cid:03X}", C.classify(cid)[0] if not fl else "error",
                                data.hex(" "), C.describe(cid, data)])
        self.log(f"    trace: {path}")
        return path

    def save(self, start):
        os.makedirs(RUNS, exist_ok=True)
        base = os.path.join(RUNS, start)
        doc = {"started": start, "can": {"bitrate": self.bus.bitrate, "node": self.node_id}, "usb": bool(self.usb),
               "results": self.results}
        with open(base + ".json", "w", encoding="utf-8") as f:
            json.dump(doc, f, indent=1, default=str)
        with open(base + ".csv", "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["test", "result", "check", "ok", "detail"])
            for r in self.results:
                w.writerow([r["name"], r["result"], "", "", r["error"]])
                for name, ok, detail in r["checks"]:
                    w.writerow([r["name"], "", name, int(ok), detail])
        return base


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("names", nargs="*", help="tests to run (default: all)")
    ap.add_argument("--can", default="slcan:COM8", help="interface:channel, e.g. slcan:COM8, pcan:PCAN_USBBUS1, socketcan:can0")
    ap.add_argument("--bitrate", type=int, default=1000000)
    ap.add_argument("--node", type=int, default=1)
    ap.add_argument("--usb", help="config link port (USB CDC or USART1), e.g. COM6")
    ap.add_argument("--usb-baud", type=int, default=2000000)
    ap.add_argument("--no-motion", action="store_true", help="skip the tests that spin the motor")
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args()

    tests = discover()
    if a.list:
        for t in tests:
            print(f"{t.name:18s} {'usb ' if t.usb else '    '}{'motion ' if t.motion else '       '}{t.doc}")
        return 0
    if a.names:
        by = {t.name: t for t in tests}
        bad = [n for n in a.names if n not in by]
        if bad:
            print("unknown test(s):", ", ".join(bad), "(--list)")
            return 2
        tests = [by[n] for n in a.names]

    iface, _, chan = a.can.partition(":")
    bus = C.CoBus().connect(iface, chan, a.bitrate)
    usb = C.UsbSerial(a.usb, a.usb_baud) if a.usb else None
    if usb and usb.request(P.PING).status != P.ST_OK:
        print(f"no reply from the config link on {a.usb}")
        return 2
    runner = Runner(bus, a.node, usb, motion=not a.no_motion)
    try:
        runner.run(tests)
    except KeyboardInterrupt:
        runner.cancel.set()
        print("interrupted")
    finally:
        try:
            C.Drive402(bus.drives.get(a.node) or bus.add_drive(a.node)).disable_voltage()
        except Exception:  # noqa: BLE001
            pass
        bus.disconnect()  # disable voltage first, then the heartbeat stops
        if usb:
            usb.close()
    return 0 if all(r["result"] in ("pass", "skip") for r in runner.results) else 1


if __name__ == "__main__":
    sys.exit(main())
