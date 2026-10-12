#!/usr/bin/env python3
"""
Flash usage by component, from the linker map of the last build (build/Driver_TCC.map).

The map is shared by debug and release builds: it describes whichever was linked last.

Usage:
    python software/flash_report.py [path/to/Driver_TCC.map] [-n 25]
"""
import argparse
import collections
import re

FLASH_LO, FLASH_HI = 0x08000000, 0x08020000

# groups worth tracking while trimming; a section is counted in the first group that matches
GROUPS = [
    ("USB CDC stack (unused)", lambda sec, obj: re.search(r"usbd_|_pcd|ll_usb|usb_device", obj)),
    ("printf %f (_printf_float, dtoa)", lambda sec, obj: "libc_nano" in obj and re.search(r"_printf_float|_dtoa|cvt|exponent|frexp", sec)),
    ("strtod/strtof + bignum helpers", lambda sec, obj: "libc_nano" in obj and re.search(
        r"strto|gethex|hexnan|_match|mprec|Balloc|Bfree|lshift|multadd|ulp|b2d|d2b|ratio|hi0bits|lo0bits|s2b|pow5mult|i2b|multiply|mcmp|mdiff|any_on|copybits", sec)),
    ("rest of libc_nano", lambda sec, obj: "libc_nano" in obj),
    ("libgcc (soft double, 64-bit div)", lambda sec, obj: "libgcc" in obj),
    ("libm", lambda sec, obj: "libm.a" in obj),
    ("HAL/LL drivers", lambda sec, obj: re.search(r"stm32g4xx_(hal|ll)", obj)),
    ("CubeMX init (Core/Src/*.c generated)", lambda sec, obj: re.search(r"/(adc|dac|dma|fdcan|gpio|i2c|opamp|spi|tim|usart|cordic|main|stm32g4xx_it|stm32g4xx_hal_msp|system_stm32g4xx|startup_\w+)\.o$", obj)),
    ("application", lambda sec, obj: True),
]


def parse(path):
    lines = open(path, encoding="utf-8", errors="replace").read().splitlines()
    start = next(k for k, l in enumerate(lines) if l.startswith("Linker script and memory map"))
    rows, pend = [], None
    for l in lines[start:]:
        m = re.match(r"^ (\.[\w.$]+)\s*$", l)
        if m:
            pend = m.group(1)  # long section name: address/size/object follow on the next line
            continue
        m = re.match(r"^ (\.[\w.$]+)\s+(0x[0-9a-f]+)\s+(0x[0-9a-f]+)\s+(\S+)", l)
        if m:
            sec, addr, size, obj = m.groups()
        else:
            m = re.match(r"^\s+(0x[0-9a-f]+)\s+(0x[0-9a-f]+)\s+(\S+)", l)
            if not (m and pend):
                pend = None
                continue
            addr, size, obj = m.groups()
            sec = pend
        pend = None
        addr, size = int(addr, 16), int(size, 16)
        if FLASH_LO <= addr < FLASH_HI and size:
            rows.append((size, sec, obj.replace("\\", "/")))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("map", nargs="?", default="build/Driver_TCC.map")
    ap.add_argument("-n", type=int, default=25, help="how many objects / sections to list")
    a = ap.parse_args()
    rows = parse(a.map)
    total = sum(r[0] for r in rows)
    print(f"{a.map}: {total} bytes in flash sections (.data init image not included)\n")

    groups = collections.Counter()
    for size, sec, obj in rows:
        for name, f in GROUPS:
            if f(sec, obj):
                groups[name] += size
                break
    print("by component:")
    for name, _ in GROUPS:
        print(f"  {groups[name]:7d}  {name}")

    objs = collections.Counter()
    for size, sec, obj in rows:
        lib = re.match(r".*/(lib\w+\.a)", obj)
        objs[lib.group(1) if lib else obj.split("/")[-1]] += size
    print("\nby object:")
    for k, v in objs.most_common(a.n):
        print(f"  {v:7d}  {k}")

    print("\nlargest sections:")
    for size, sec, obj in sorted(rows, reverse=True)[: a.n]:
        print(f"  {size:7d}  {sec:48s} {obj.split('/')[-1]}")


if __name__ == "__main__":
    main()
