#!/usr/bin/env python3
"""
Checks that the PC side matches the firmware sources (no hardware, no compiler needed):
    Core/Src/objdict.c k_tab   <->  objects.py OBJECTS   (order, index, sub, type, flags; and the hash)
    Core/Inc/proto.h           <->  protocol.py          (message types, CAL ids, status and event codes)
    software/driverTCC.eds     <->  objects.py OBJECTS   (the native objects 0x4000-0x40FF over CANopen: type, access)

Usage:
    python software/check_sync.py
"""
import configparser
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import objects as O  # noqa: E402
import protocol as P  # noqa: E402

errors = []


def src(path):
    with open(os.path.join(ROOT, path), encoding="utf-8") as f:
        return re.sub(r"/\*.*?\*/", "", f.read(), flags=re.S)  # drop comments


def enum_values(body):
    """'A = 1, B, C = 0x10, D' -> {name: value}"""
    out, v = {}, -1
    for item in body.split(","):
        item = item.strip()
        if not item:
            continue
        name, _, val = item.partition("=")
        v = int(val.strip(), 0) if val.strip() else v + 1
        out[name.strip()] = v
    return out


# ---------------------------------------------------------------- object table
c = src("Core/Src/objdict.c")
types = {"T_U8": 1, "T_I8": 2, "T_U16": 3, "T_U32": 4, "T_I32": 5, "T_F32": 6, "T_I64": 7, "T_I16": 8}
flags = {"OF_RO": O.RO, "OF_STORED": O.STORED, "OF_PI": O.PI, "OF_NOTRUN": O.NOTRUN, "0": 0}
fw = []
for idx, sub, t, fl in re.findall(r"\{\s*(0x[0-9A-Fa-f]+),\s*(\d+),\s*(T_\w+),\s*([\w |]+?),\s*F\(", c):
    fw.append((int(idx, 16), int(sub), types[t], sum(flags[x.strip()] for x in fl.split("|"))))
py = [(o.index, o.sub, o.type, o.flags) for o in O.OBJECTS]
if fw != py:
    for i in range(max(len(fw), len(py))):
        a = fw[i] if i < len(fw) else None
        b = py[i] if i < len(py) else None
        if a != b:
            errors.append(f"object #{i}: objdict.c {a} != objects.py {b}")
            break
    errors.append(f"object table differs ({len(fw)} entries in objdict.c, {len(py)} in objects.py)")
print(f"object table: {len(fw)} entries, hash {O.table_hash():08X}")

# ---------------------------------------------------------------- proto.h
h = src("Core/Inc/proto.h")
defs = {k: int(v, 0) for k, v in re.findall(r"#define\s+(MSG_\w+)\s+(0x[0-9A-Fa-f]+|\d+)", h)}
for name, val in defs.items():
    pyname = name[4:]
    if getattr(P, pyname, None) != val:
        errors.append(f"{name} = 0x{val:02X} in proto.h, protocol.{pyname} = {getattr(P, pyname, None)}")
requests = {k: v for k, v in defs.items() if k not in ("MSG_REPLY", "MSG_EVENT")}
for k, v in requests.items():
    if v >= defs["MSG_EVENT"] or (v | defs["MSG_REPLY"]) == defs["MSG_EVENT"]:
        errors.append(f"{k} = 0x{v:02X}: requests must stay below MSG_EVENT, or a reply looks like an event")
m = re.search(r"#define\s+PROTO_VERSION\s+(\d+)", h)
if int(m.group(1)) != P.PROTO_VERSION:
    errors.append(f"PROTO_VERSION {m.group(1)} != protocol.PROTO_VERSION {P.PROTO_VERSION}")

enums = [enum_values(b) for b in re.findall(r"enum\s*\{(.*?)\}", h, flags=re.S)]
st = next(e for e in enums if "ST_OK" in e)
if st["ST_N"] != len(P.STATUS):
    errors.append(f"proto.h has {st['ST_N']} status codes, protocol.STATUS {len(P.STATUS)}")
for name in ("ST_OK", "ST_ABORTED", "ST_BUSY"):
    if st[name] != getattr(P, name):
        errors.append(f"{name} = {st[name]} in proto.h, {getattr(P, name)} in protocol.py")
for e in enums:
    for name, val in e.items():
        if name.startswith(("EV_", "CAL_")) and name != "CAL_N" and getattr(P, name, None) != val:
            errors.append(f"{name} = {val} in proto.h, protocol.{name} = {getattr(P, name, None)}")
        if name.startswith("EV_") and val not in P.EVENTS:
            errors.append(f"{name} has no formatter in protocol.EVENTS")
for code in re.findall(r"#define\s+SDO_\w+\s+(0x[0-9A-Fa-f]+)u", h):
    if int(code, 16) not in P.SDO:
        errors.append(f"SDO code {code} has no text in protocol.SDO")

# CANopen: the native objects in the exported EDS (driverTCC.xdd -> CANopenEditor) must match objects.py
EDS = os.path.join(ROOT, "software", "driverTCC.eds")
EDS_TYPE = {P.T_U8: 0x0005, P.T_I8: 0x0002, P.T_U16: 0x0006, P.T_U32: 0x0007, P.T_I32: 0x0004, P.T_F32: 0x0008,
            P.T_I16: 0x0003}
eds = configparser.ConfigParser(strict=False, interpolation=None)
eds.read(EDS, encoding="utf-8")
native = {o.index for o in O.OBJECTS}
for o in O.OBJECTS:
    sec = f"{o.index:04X}sub{o.sub:X}"
    if not eds.has_section(sec):
        errors.append(f"0x{o.index:04X}:{o.sub} {o.key} is not in driverTCC.eds (add it to driverTCC.xdd, re-export)")
        continue
    t = int(eds.get(sec, "DataType", fallback="0"), 0)
    if t != EDS_TYPE[o.type]:
        errors.append(f"0x{o.index:04X}:{o.sub} {o.key}: EDS DataType 0x{t:04X}, objects.py 0x{EDS_TYPE[o.type]:04X}")
    acc = eds.get(sec, "AccessType", fallback="").lower()
    if (acc in ("ro", "const")) != o.ro:
        errors.append(f"0x{o.index:04X}:{o.sub} {o.key}: EDS AccessType {acc}, objects.py {'RO' if o.ro else 'writable'}")
for sec in eds.sections():
    m = re.fullmatch(r"([0-9A-F]{4})sub([0-9A-F]+)", sec, flags=re.I)
    if m and int(m.group(1), 16) in native and int(m.group(2), 16) and (int(m.group(1), 16), int(m.group(2), 16)) not in O.BY_ADDR:
        errors.append(f"driverTCC.eds has {sec}, unknown to objects.py")
xdd, odh = os.path.join(ROOT, "driverTCC.xdd"), os.path.join(ROOT, "CANopen", "od", "OD.h")
if os.path.getmtime(xdd) > os.path.getmtime(odh) + 2 or os.path.getmtime(xdd) > os.path.getmtime(EDS) + 2:
    print("note: driverTCC.xdd is newer than OD.h / driverTCC.eds: re-export both from CANopenEditor if it changed")

if errors:
    print("\n".join("MISMATCH: " + e for e in errors))
    sys.exit(1)
print("in sync")
