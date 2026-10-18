"""CiA 402 modes and stops: these spin the motor (keep the shaft free)"""
import time

import numpy as np

import co_link as C
import protocol as P
from cotest import test


def csv_run(ctx, rpm, settle=1.5):
    """CSV enabled, 60FF = rpm over SDO, wait; -> mean rpm over the last half of `settle`"""
    ctx.node.write(0x60FF, 0, ctx.units.rpm_to_ips(rpm))
    ctx.sleep(settle / 2)
    since = time.time()
    ctx.sleep(settle / 2)
    t, v = ctx.signal(0x606C, since=since)
    return ctx.units.ips_to_rpm(v.mean()) if len(v) else float("nan")


def wait_slow(ctx, rpm=5.0, timeout=3.0):
    """-> seconds until |speed| < rpm, or None"""
    t0 = time.time()
    return time.time() - t0 if ctx.wait(lambda: abs(ctx.rpm()) < rpm, timeout) else None


def wait_fault(ctx, timeout):
    """fault reaction active or fault (polled over SDO)"""
    return ctx.drive.wait_sw(lambda sw: C.fsa_state(sw) in (C.FRA, C.FAULT), timeout)


def first_emcy(ctx, code, since, timeout=1.0):
    """-> time of the first EMCY `code` after `since` (the drive's own reaction time), or None"""
    ctx.wait(lambda: any(e[1] == code for e in ctx.emcys(since)), timeout)
    return next((e[0] for e in ctx.emcys(since) if e[1] == code), None)


@test("csv", motion=True)
def t_csv(ctx):
    """CSV +60 / -60 rpm: follows, statusword bit 12"""
    n, d = ctx.node, ctx.drive
    ctx.operational()
    n.write(0x60FF, 0, 0)
    d.goto_enabled(C.MODE_CSV)
    for rpm in (60.0, -60.0):
        got = csv_run(ctx, rpm)
        ctx.check(f"speed {rpm:+.0f} rpm", abs(got - rpm) < 10, f"{got:+.1f} rpm")
        sw = d.sw()
        ctx.check(f"  bit 12 following ({rpm:+.0f})", sw & 0x1000, f"0x{sw:04X}")
    csv_run(ctx, 0.0, 1.0)


@test("stops_at_speed", motion=True)
def t_stops(ctx):
    """disable operation and quick stop at 60 rpm: ramp, coast, end in switched on / switch on disabled"""
    n, d = ctx.node, ctx.drive
    ctx.operational()
    u = ctx.units
    dec, qdec = u.ips2_to_rpms(n.read(0x6084)), u.ips2_to_rpms(n.read(0x6085))
    for name, cmd, want, rate in (("disable operation", d.disable_op, C.SO, dec),
                                  ("quick stop", d.quick_stop, C.SOD, qdec)):
        n.write(0x60FF, 0, 0)
        d.goto_enabled(C.MODE_CSV)
        csv_run(ctx, 60.0)
        ctx.check(f"{name}: running at 60 rpm first", abs(ctx.rpm() - 60) < 15, f"{ctx.rpm():+.1f} rpm")
        cmd()
        t = wait_slow(ctx, 5.0, 3.0)
        ramp = f"ramp at {rate:.0f} rpm/s: {60 / rate:.2f} s" if rate else "no ramp (rate 0)"
        ctx.check(f"{name}: below 5 rpm", t is not None, f"{t:.2f} s ({ramp})" if t is not None else "still turning")
        sw = d.wait_state(want, 2.0)
        ctx.check(f"{name}: ends in {C.FSA_NAMES[want]}", sw is not None, f"0x{d.node.sw:04X}")
        ctx.check(f"{name}: CAN released the motor", ctx.wait(lambda: not n.read_native("can_owner"), 2.0))
        d.disable_voltage()
        ctx.sleep(0.2)


@test("local_stop", usb=True, motion=True)
def t_local_stop(ctx):
    """USB STOP while CAN runs: fault 0xFF01 + EMCY; fault reset after the coast"""
    n, d = ctx.node, ctx.drive
    ctx.operational()
    n.write(0x60FF, 0, 0)
    d.goto_enabled(C.MODE_CSV)
    csv_run(ctx, 60.0)
    r = ctx.usb.request(P.RUN_VEL, P.p_run_vel(0.0))
    ctx.check("USB RUN refused while CAN has the motor", r.status == P.ST_CAN_ACTIVE, P.status_text(r.status))
    since = time.time()
    ctx.usb.stop()
    ctx.check("fault within 0.5 s", wait_fault(ctx, 0.5) is not None, f"0x{d.node.sw:04X}")
    ctx.check("603F = 0xFF01", n.read(0x603F) == 0xFF01, f"0x{n.read(0x603F):04X}")
    ctx.sleep(0.1)
    ctx.check("EMCY 0xFF01", any(e[1] == 0xFF01 for e in ctx.emcys(since)), [hex(e[1]) for e in ctx.emcys(since)])
    wait_slow(ctx, 5.0, 3.0)
    ctx.sleep(0.3)
    d.fault_reset()
    sw = d.wait_state(C.SOD, 1.0)
    ctx.check("fault reset -> switch on disabled", sw is not None, f"0x{d.node.sw:04X}")


@test("master_loss", motion=True)
def t_master_loss(ctx):
    """PC heartbeat watched by 0x1016 (300 ms), then stopped: fault 0x8130 within ~0.3 s"""
    n, d = ctx.node, ctx.drive
    hb = ctx.bus.hb or ctx.bus.start_heartbeat()
    ctx.set(0x1016, 1, (hb.node_id << 16) | 300)
    ctx.operational()
    n.write(0x60FF, 0, 0)
    d.goto_enabled(C.MODE_CSV)
    ctx.sleep(0.5)
    ctx.check("enabled with the heartbeat watched", C.fsa_state(d.sw()) == C.OE, f"0x{d.node.sw:04X}")
    since = time.time()
    hb.paused = True
    try:
        t = first_emcy(ctx, 0x8130, since, 1.5)
        ctx.check("EMCY 0x8130 after the heartbeat stops", t is not None)
        if t is not None:
            dt = t - since
            ctx.check("  within 0x1016 (300 ms) + one PC heartbeat period", dt < 0.3 + hb.period + 0.05, f"{dt:.3f} s")
        ctx.check("fault", wait_fault(ctx, 1.5) is not None, f"0x{d.node.sw:04X}")
        ctx.check("603F = 0x8130", n.read(0x603F) == 0x8130, f"0x{n.read(0x603F):04X}")
        ctx.note("NMT state after the loss", n.nmt_state)
    finally:
        hb.paused = False
    ctx.sleep(0.3)
    d.fault_reset()
    ctx.check("fault reset", d.wait_state(C.SOD, 1.0) is not None, f"0x{d.node.sw:04X}")


@test("cst", motion=True)
def t_cst(ctx):
    """CST +-20 per mille pulses, 0.2 s each (Streamer, 5 ms), speed guard at 4 turn/s: the current follows"""
    n, d = ctx.node, ctx.drive
    ctx.operational()
    n.write(0x6071, 0, 0)
    d.goto_enabled(C.MODE_CST)
    s = C.Streamer(d, C.MODE_CST, C.gen_step(20.0, 0.2, 0.4), period=0.005, max_rpm=240.0, duration=1.0).start()
    t0 = time.time()
    ctx.wait(lambda: not s.running, 3.0)
    n.write(0x6071, 0, 0)
    ctx.note("guard", s.tripped or "not reached")
    tt, tq = ctx.signal(0x6077, since=t0)
    sl = s.log.snapshot()
    if len(tt) < 20 or not sl:
        ctx.check("torque samples", False, len(tt))
        return
    ts = np.array([x[0] for x in sl])
    cmd = np.array([x[1] for x in sl])
    want = np.interp(tt - 0.01, ts, cmd)  # what was asked 10 ms before (2 periods)
    on = np.abs(want) >= 19
    if on.any():
        err = np.abs(tq[on] - want[on])
        ctx.check("6077 follows the pulses (median error < 5 per mille)", np.median(err) < 5,
                  f"median {np.median(err):.1f}, sign ok {np.mean(np.sign(tq[on]) == np.sign(want[on])) * 100:.0f} %")
    ctx.check("streamer: no send errors", s.errors == 0, s.errors)
    peak = np.abs(ctx.units.ips_to_rpm(ctx.signal(0x606C, since=t0)[1])).max()
    ctx.check("speed stayed inside the guard (or the guard acted)", peak < 240 or s.tripped, f"peak {peak:.0f} rpm")


@test("csp", usb=False, motion=True)
def t_csp(ctx):
    """CSP sine 0.5 turn, 0.5 Hz, 5 ms RPDO stream + SYNC: tracking error"""
    n, d = ctx.node, ctx.drive
    ctx.operational()
    d.goto_enabled(C.MODE_CSP)  # writes 607A = 6064 first
    p0 = n.read(0x6064)
    s = C.Streamer(d, C.MODE_CSP, C.gen_sine(0.5, 0.5), period=0.005, max_rpm=200.0, max_turns=0.8,
                   duration=4.0).start(start_pos=p0)
    t0 = time.time()
    ctx.wait(lambda: not s.running, 6.0)
    ctx.check("no guard", not s.tripped, s.tripped or "")
    ctx.check("still enabled", C.fsa_state(d.sw()) == C.OE, f"0x{d.node.sw:04X}")
    tt, pp = ctx.signal(0x6064, since=t0 + 1.0)
    sl = s.log.snapshot()
    if len(tt) < 50 or len(sl) < 50:
        ctx.check("samples", False, f"{len(tt)} TPDO2, {len(sl)} sent")
        return
    ts = np.array([x[0] for x in sl])
    tgt = np.array([x[1] for x in sl], float)
    k = tt <= ts[-1]
    tt, pp = tt[k], pp[k]
    err_now = np.abs(pp - np.interp(tt, ts, tgt)) * 360 / ctx.units.inc_rev
    err_lag = np.abs(pp - np.interp(tt - s.period, ts, tgt)) * 360 / ctx.units.inc_rev
    ctx.note("tracking error deg (max / rms), vs the target at the same time", f"{err_now.max():.1f} / "
             f"{np.sqrt(np.mean(err_now ** 2)):.1f}")
    ctx.note("tracking error deg (max), vs the target one period before", f"{err_lag.max():.1f}")
    ctx.check("tracking within 8 deg (bench: 3-5 deg at 94 rpm)", min(err_now.max(), err_lag.max()) < 8.0,
              f"{min(err_now.max(), err_lag.max()):.1f} deg")
    j = ctx.bus.sync.jitter() * 1000 if ctx.bus.sync else np.zeros(0)
    if len(j):
        ctx.note("PC SYNC period ms (mean / max)", f"{j.mean():.2f} / {j.max():.2f}")
    # back to the start smoothly
    back = C.Streamer(d, C.MODE_CSP, C.gen_trapezoid(ctx.units.inc_to_turn(p0 - n.read(0x6064)), 1.0, 4.0),
                      period=0.005, duration=2.0).start()
    ctx.wait(lambda: not back.running, 3.0)


@test("following_error", motion=True)
def t_following_error(ctx):
    """6065 = 200 inc, 6066 = 5 ms, 1/4-turn CSP step: fault 0x8611 + EMCY, reset OK"""
    n, d = ctx.node, ctx.drive
    ctx.operational()
    ctx.set(0x6065, 0, 200)
    ctx.set(0x6066, 0, 5)
    d.goto_enabled(C.MODE_CSP)
    p0 = n.read(0x6064)
    since = time.time()
    s = C.Streamer(d, C.MODE_CSP, C.gen_step(0.25, 0.05), period=0.005, max_rpm=600.0, max_turns=0.5,
                   duration=0.5).start(start_pos=p0)
    sw = wait_fault(ctx, 1.0)
    s.stop()
    ctx.check("fault", sw is not None, f"0x{d.node.sw:04X}")
    ctx.check("603F = 0x8611", n.read(0x603F) == 0x8611, f"0x{n.read(0x603F):04X}")
    ctx.sleep(0.1)
    ctx.check("EMCY 0x8611", any(e[1] == 0x8611 for e in ctx.emcys(since)))
    wait_slow(ctx, 5.0, 3.0)
    ctx.sleep(0.3)
    d.fault_reset()
    ctx.check("fault reset", d.wait_state(C.SOD, 1.0) is not None, f"0x{d.node.sw:04X}")


@test("pv", motion=True)
def t_pv(ctx):
    """PV 0 -> +60 -> -60 rpm at 240 rpm/s; target reached; halt -> bit 10 at standstill; release"""
    n, d = ctx.node, ctx.drive
    u = ctx.units
    ctx.operational()
    ctx.set(0x6083, 0, u.rpms_to_ips2(240))
    ctx.set(0x6084, 0, u.rpms_to_ips2(240))
    n.write(0x60FF, 0, 0)
    d.goto_enabled(C.MODE_PV)
    for rpm, ramp in ((60.0, 0.25), (-60.0, 0.5)):
        t0 = time.time()
        n.write(0x60FF, 0, u.rpm_to_ips(rpm))
        ctx.sleep(0.05)
        ok = d.wait_reached(2.0)
        dt = time.time() - t0
        ctx.check(f"target reached at {rpm:+.0f} rpm", ok, f"{dt:.2f} s (ramp alone {ramp:.2f} s)")
        ctx.sleep(0.5)
        got = ctx.rpm()
        ctx.check(f"  speed {rpm:+.0f}", abs(got - rpm) < 15, f"{got:+.1f} rpm")
    d.halt(True)
    ok = ctx.wait(lambda: abs(ctx.rpm()) < 5, 2.0)
    ctx.sleep(0.2)
    sw = d.sw()
    ctx.check("halt: stops", ok, f"{ctx.rpm():+.1f} rpm")
    ctx.check("halt: bit 10 at standstill", sw & 0x400, f"0x{sw:04X}")
    d.halt(False)
    ctx.sleep(0.8)
    ctx.check("release: back to -60 rpm", abs(ctx.rpm() + 60) < 15, f"{ctx.rpm():+.1f} rpm")
    n.write(0x60FF, 0, 0)
    ctx.sleep(0.5)


@test("hm", motion=True)
def t_hm(ctx):
    """HM 37 with 607C = 1/4 turn: 6064 = -1/4 turn, attained"""
    n, d = ctx.node, ctx.drive
    ctx.operational()
    q = ctx.units.inc_rev // 4
    ctx.set(0x6098, 0, 37)
    ctx.set(0x607C, 0, q)
    attained, err = d.home(37, q)
    sw = d.node.sw
    ctx.check("homing attained (bits 10 + 12)", attained and sw & 0x1400 == 0x1400, f"0x{sw:04X}")
    ctx.check("no homing error (bit 13)", not err)
    ctx.sleep(0.1)
    pos = n.read(0x6064)
    ctx.check("6064 = -607C", abs(pos + q) < 200, f"{pos} inc")
    d.disable_voltage()
    ctx.sleep(0.2)
    d.home(37, 0)  # leave zero here again


@test("pp", motion=True)
def t_pp(ctx):
    """PP: +1 turn abs, -0.5 rel, change immediately, a buffered pair, halt mid-move"""
    n, d = ctx.node, ctx.drive
    u = ctx.units
    ctx.operational()
    ctx.set(0x6081, 0, u.rpm_to_ips(60))
    ctx.set(0x6083, 0, u.rpms_to_ips2(240))
    ctx.set(0x6084, 0, u.rpms_to_ips2(240))
    win = n.read(0x6067)
    d.goto_enabled(C.MODE_PP)
    p0 = n.read(0x6064)
    T = u.inc_rev

    def at(target, name, timeout=8.0):
        """demand 6062 ends exactly on the target; 6064 is inside 6067 when bit 10 comes on (TPDO1 time)"""
        t_cmd = time.time()
        ok = d.wait_reached(timeout)
        dem = n.read(0x6062)
        ctx.check(name + ": demand on the target", ok and abs(dem - target) <= 1, f"{dem - target:+d} inc")
        ts, sw = ctx.signal(0x6041, since=t_cmd)
        tp, pp = ctx.signal(0x6064, since=t_cmd - 0.1)
        on = ts[(sw.astype(int) & 0x400) != 0]
        if ok and len(on) and len(tp):
            e = (np.interp(on[0], tp, pp) - target) * 360 / T
            ctx.check(name + ": actual inside 6067 at target reached", abs(e) <= win * 360 / T * 1.5,
                      f"{e:+.2f} deg, window {win * 360 / T:.2f} deg")
        ctx.sleep(0.5)
        ctx.note(name + ": settled error deg", f"{(n.read(0x6064) - target) * 360 / T:+.2f}")

    ctx.check("set-point acknowledged", d.pp_move(p0 + T, relative=False, immediate=True))
    at(p0 + T, "+1 turn absolute")
    d.pp_move(-T // 2, relative=True, immediate=True)
    at(p0 + T // 2, "-0.5 turn relative")
    # change immediately: head for +2 turns, replace with +0.25 after 0.4 s
    d.pp_move(p0 + 2 * T, immediate=True)
    ctx.sleep(0.4)
    d.pp_move(p0 + T // 4, immediate=True)
    at(p0 + T // 4, "change immediately to +0.25 turn")
    # buffered pair: two relative +0.5 set-points while the first runs (relative to the last target)
    last = p0 + T // 4
    a = d.pp_move(T // 2, relative=True, immediate=False)
    b = d.pp_move(T // 2, relative=True, immediate=False)
    ctx.check("both buffered set-points acknowledged", a and b)
    at(last + T, "buffered pair: exactly 1.0 turn on")
    # halt mid-move
    here = n.read(0x6064)
    d.pp_move(here - T, immediate=True)
    ctx.sleep(0.4)
    d.halt(True)
    ctx.wait(lambda: abs(ctx.rpm()) < 3, 2.0)
    ctx.sleep(0.3)
    h1 = n.read(0x6064)
    ctx.sleep(0.3)
    h2 = n.read(0x6064)
    ctx.check("halt holds mid-move", abs(h2 - h1) < T // 200 and abs(h2 - (here - T)) > win, f"{(h2 - here) / T:+.3f} turn")
    d.halt(False)
    at(here - T, "release: continues to the target")
