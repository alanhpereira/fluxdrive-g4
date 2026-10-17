"""CiA 301 and FSA tests: nothing here spins the motor"""
import statistics
import time

import co_link as C
import protocol as P
from cotest import test


def expect_abort(ctx, name, fn, codes):
    """fn() must fail with one of the abort codes"""
    try:
        fn()
    except C.CoError as e:
        return ctx.check(name, e.code in codes, C.abort_text(e.code))
    return ctx.check(name, False, "accepted")


@test("identity")
def t_identity(ctx):
    """reset node: boot-up, 0x1018 identity, heartbeat period"""
    n = ctx.node
    t0 = time.time()
    n.nmt("RESET")
    ctx.check("boot-up 0x700+ID 00", n.wait_bootup(8.0, since=t0) is not None)
    ctx.sleep(0.3)
    ctx.check("vendor 0x1018:1 = 0", n.read(0x1018, 1) == 0)
    ctx.check("product 0x1018:2 = 1", n.read(0x1018, 2) == 1)
    pv, parv = n.read_native("proto_ver"), n.read_native("param_ver")
    rev = n.read(0x1018, 3)
    ctx.check("revision = PROTO_VERSION<<16 | PARAM_VERSION", rev == (pv << 16 | parv), f"0x{rev:08X}")
    ctx.check("protocol version matches protocol.py", pv == P.PROTO_VERSION, pv)
    ctx.note("serial 0x1018:4", f"0x{n.read(0x1018, 4):08X}")
    ctx.note("device name 0x1008", n.read(0x1008))  # a string: segmented or expedited SDO
    hb_ms = n.read(0x1017)
    t_hb = []
    since = time.time()
    ctx.wait(lambda: len([e for e in ctx.bus.trace.snapshot() if e[1] == 0x700 + n.id and e[0] >= since]) >= 4,
             4 * hb_ms / 1000 + 1)
    t_hb = [e[0] for e in ctx.bus.trace.snapshot() if e[1] == 0x700 + n.id and e[0] >= since]
    if len(t_hb) >= 2:
        per = statistics.mean(b - a for a, b in zip(t_hb, t_hb[1:])) * 1000
        ctx.check("heartbeat period = 0x1017", abs(per - hb_ms) < 0.1 * hb_ms + 5, f"{per:.0f} ms vs {hb_ms} ms")
    else:
        ctx.check("heartbeat seen", False)


@test("sdo_errors")
def t_sdo_errors(ctx):
    """abort codes: read-only, out of range, wrong size, unknown object / subindex"""
    n = ctx.node
    expect_abort(ctx, "write RO 6041", lambda: n.download(0x6041, 0, b"\x00\x00"), {0x06010002})
    expect_abort(ctx, "6060 = 2 (unsupported mode)", lambda: n.write(0x6060, 0, 2), {0x06090030})
    expect_abort(ctx, "6098 = 1 (homing method)", lambda: n.write(0x6098, 0, 1), {0x06090030})
    expect_abort(ctx, "node ID 0x4040:1 = 0", lambda: n.download(0x4040, 1, b"\x00"), {0x06090030, 0x06090032})
    expect_abort(ctx, "node ID 0x4040:1 = 200", lambda: n.download(0x4040, 1, b"\xC8"), {0x06090030, 0x06090031})
    expect_abort(ctx, "6040 with 3 bytes", lambda: n.download(0x6040, 0, b"\x00\x00\x00"), {0x06070010, 0x06070012})
    expect_abort(ctx, "6040 with 1 byte", lambda: n.download(0x6040, 0, b"\x00"), {0x06070010, 0x06070013})
    expect_abort(ctx, "unknown object 0x2000", lambda: n.upload(0x2000, 0), {0x06020000})
    expect_abort(ctx, "unknown subindex 0x1018:9", lambda: n.upload(0x1018, 9), {0x06090011})
    expect_abort(ctx, "store with a bad signature", lambda: n.download(0x1010, 1, b"abcd"), {0x08000020})


@test("node_id_bitrate", usb=True)
def t_node_id_bitrate(ctx):
    """node ID and bit rate over USB 0x4040 + apply; the node answers at the new ones; then back"""
    id0, kb0 = ctx.usb.read("can_id"), ctx.usb.read("can_kbit")
    id1, kb1 = (5 if id0 != 5 else 6), (500 if kb0 != 500 else 250)
    try:
        ctx.usb.write("can_id", id1)
        ctx.usb.write("can_kbit", kb1)
        ctx.usb.write("can_apply", 1)
        ctx.sleep(0.3)
        ctx.reconnect(bitrate=kb1 * 1000, node_id=id1)
        ctx.check(f"answers as node {id1} @ {kb1} kbit/s", ctx.node.read(0x1000) is not None)
        ctx.check("0x4040:3 node ID in use", ctx.node.read_native("can_id_act") == id1)
        ctx.check("0x4040:4 bit rate in use", ctx.node.read_native("can_kbit_act") == kb1)
    finally:
        ctx.usb.write("can_id", id0)
        ctx.usb.write("can_kbit", kb0)
        ctx.usb.write("can_apply", 1)
        ctx.sleep(0.3)
        ctx.reconnect(bitrate=kb0 * 1000, node_id=id0)
    ctx.check(f"back at node {id0} @ {kb0} kbit/s", ctx.node.read_native("can_id_act") == id0)


# OD_PERSIST_APP in CANopen/od/OD.h: what 0x1010:3 / 0x1011:3 store and restore
PERSIST_APP = (0x6007, 0x605A, 0x605B, 0x605C, 0x605D, 0x605E, 0x6065, 0x6066, 0x6067, 0x6068, 0x606D, 0x606E, 0x606F,
               0x6070, 0x6072, 0x607C, 0x607D, 0x607F, 0x6081, 0x6083, 0x6084, 0x6085, 0x6086, 0x6098, 0x6099, 0x609A,
               0x60B2, 0x60C2)


def app_record(n):
    """{(index, sub): value} of the writable entries in the application record"""
    out = {}
    for idx in PERSIST_APP:
        e = n.node.object_dictionary[idx]
        subs = [e] if hasattr(e, "access_type") else [v for k, v in e.subindices.items() if k]
        for v in subs:
            if "w" in v.access_type:
                out[(idx, v.subindex)] = n.read(idx, v.subindex)
    return out


@test("store_restore")
def t_store_restore(ctx):
    """0x1010:3 survives a reset node; 0x1011:3 goes back to the EDS default; refused while running"""
    n, d = ctx.node, ctx.drive
    default = n.od_entry(0x6065).default
    saved = app_record(n)  # 0x1011:3 resets all of it: put it back afterwards
    v1 = default + 12345
    try:
        n.write(0x6065, 0, v1)
        n.store(3)
        ctx.sleep(0.5)  # the main loop saves with the bridge off
        t0 = time.time()
        n.nmt("RESET")
        ctx.check("boot-up after the store", n.wait_bootup(8.0, since=t0) is not None)
        ctx.sleep(0.3)
        ctx.check("stored 6065 survives the reset", n.read(0x6065) == v1, n.read(0x6065))
        n.restore(3)
        t0 = time.time()
        n.nmt("RESET")
        n.wait_bootup(8.0, since=t0)
        ctx.sleep(0.3)
        ctx.check("restore: 6065 back to the EDS default", n.read(0x6065) == default, n.read(0x6065))
        # refused while running (CSV at 0 rpm)
        n.nmt("OPERATIONAL")
        n.write(0x60FF, 0, 0)
        d.goto_enabled(C.MODE_CSV)
        try:
            n.store(3)
            ctx.check("store refused while enabled", False, "accepted")
        except C.CoError as e:
            ctx.check("store refused while enabled", e.code in (0x08000022, 0x08000020), C.abort_text(e.code))
        d.stop_all()
    finally:
        d.disable_voltage()
        now = app_record(n)
        diff = {k: v for k, v in saved.items() if now.get(k) != v}
        for (idx, sub), v in diff.items():
            n.write(idx, sub, v)
        if diff:  # leave the stored record as it was
            ctx.wait(lambda: not n.read_native("can_owner"), 2.0)
            n.store(3)
            ctx.sleep(0.5)
            ctx.log(f"put back {len(diff)} application value(s) and stored them")


@test("fsa_walk")
def t_fsa_walk(ctx):
    """0x06, 0x07, 0x0F, 0x07, 0x06, 0x00 in CSV at 0 rpm: statusword at each step"""
    n, d = ctx.node, ctx.drive
    n.nmt("OPERATIONAL")
    n.write(0x60FF, 0, 0)
    d.set_mode(C.MODE_CSV)
    for cmd, want in ((0x06, C.RTSO), (0x07, C.SO), (0x0F, C.OE), (0x07, C.SO), (0x06, C.RTSO), (0x00, C.SOD)):
        d._cw(cmd)
        sw = d.wait_state(want, 1.0) or d.node.sw
        ctx.check(f"0x{cmd:02X} -> {C.FSA_NAMES[want]}", C.fsa_state(sw) == want, f"sw 0x{sw:04X}")
        if want != C.SOD:
            ctx.check(f"  voltage enabled + remote (0x{cmd:02X})", sw & 0x0210 == 0x0210, f"0x{sw:04X}")


@test("refusal", usb=True)
def t_refusal(ctx):
    """switch on is refused with the speed gains zeroed, and while USB runs the motor"""
    n, d = ctx.node, ctx.drive
    n.nmt("OPERATIONAL")
    n.write(0x60FF, 0, 0)
    d.set_mode(C.MODE_CSV)
    ctx.usb_set("vkp", 0.0)
    d.shutdown()
    d.wait_state(C.RTSO)
    d.switch_on()
    ctx.sleep(0.1)
    sw = d.sw()
    ctx.check("gains zeroed: stays ready to switch on", C.fsa_state(sw) == C.RTSO, f"0x{sw:04X}")
    ctx.check("warning bit 7", sw & 0x80, f"0x{sw:04X}")
    st, why = d.refusal()
    ctx.check("0x4042:2 = speed gains", st == P.ST_NEED_VGAINS, why)
    d.disable_voltage()
    ctx.usb.write("vkp", ctx._usb_restore[0][1])
    # USB has the motor (speed loop at 0 rpm)
    r = ctx.usb.request(P.RUN_VEL, P.p_run_vel(0.0))
    ctx.check("USB run at 0 rpm accepted", r.ok, P.status_text(r.status))
    ctx.sleep(0.1)
    sw = d.sw()
    ctx.check("remote bit 9 = 0 while USB runs", not sw & 0x200, f"0x{sw:04X}")
    d.shutdown()
    ctx.sleep(0.05)
    d.switch_on()
    ctx.sleep(0.1)
    sw = d.sw()
    ctx.check("switch on refused while USB runs", C.fsa_state(sw) != C.SO and C.fsa_state(sw) != C.OE, f"0x{sw:04X}")
    st, why = d.refusal()
    ctx.check("0x4042:2 = running", st == P.ST_RUNNING, why)
    ctx.usb.stop()


@test("timing")
def t_timing(ctx):
    """SDO latency (n = 500) and the TPDO2 period with SYNC at 5 ms"""
    n = ctx.node
    r0 = n.retries
    lat = []
    for _ in range(500):
        t = time.perf_counter()
        n.read(0x6041)
        lat.append((time.perf_counter() - t) * 1000)
    lat.sort()
    ctx.note("SDO latency ms (median / p99 / max)", f"{lat[250]:.2f} / {lat[494]:.2f} / {lat[-1]:.1f}")
    ctx.note("SDO lost replies (retried)", n.retries - r0)
    ctx.check("SDO median latency < 5 ms", lat[250] < 5.0, f"{lat[250]:.2f} ms")
    ctx.check("lost replies < 1 %", n.retries - r0 < 5, n.retries - r0)
    ctx.operational(0.005)
    since = time.time()
    ctx.sleep(2.0)
    t, _ = ctx.signal(0x6064, since=since)
    sp = ctx.bus.sync
    jit = sp.jitter() * 1000 if sp else []
    ctx.bus.stop_sync()
    if len(t) > 10:
        dt = (t[1:] - t[:-1]) * 1000
        ctx.note("TPDO2 period ms (mean / max)", f"{dt.mean():.2f} / {dt.max():.1f}")
        ctx.check("TPDO2 every SYNC (5 ms)", abs(dt.mean() - 5.0) < 0.5, f"{dt.mean():.2f} ms, {len(t)} frames")
    else:
        ctx.check("TPDO2 received", False, len(t))
    if len(jit):
        ctx.note("PC SYNC period ms (mean / min / max)", f"{jit.mean():.3f} / {jit.min():.2f} / {jit.max():.2f}")
