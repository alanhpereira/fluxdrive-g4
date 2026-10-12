#!/usr/bin/env python3
"""
Checks that the PC side matches the firmware sources (no hardware, no compiler needed):
    Core/Src/objdict.c k_tab   <->  objects.py OBJECTS   (order, index, sub, type, flags; and the hash)
    Core/Inc/proto.h           <->  protocol.py          (message types, CAL ids, status and event codes)

Usage:
    python software/check_sync.py
"""
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
types = {"T_U8": 1, "T_I8": 2, "T_U16": 3, "T_U32": 4, "T_I32": 5, "T_F32": 6, "T_I64": 7}
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

if errors:
    print("\n".join("MISMATCH: " + e for e in errors))
    sys.exit(1)
print("in sync")
