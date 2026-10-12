# Flash optimization handoff

**Goal:** free enough flash for a CANopen slave stack on FDCAN1, without slowing the 25 kHz control ISR:
- CiA 301 (NMT, SDO, PDO, EMCY, heartbeat)
- CiA 305 LSS (node-ID and bit rate set over the bus)
- CiA 402 drive profile (FSA state machine, controlword/statusword, PP/PV/HM/CSP modes)

The USB CDC stack **stays**: configuration is planned to move from USART1 to USB.

Updated 2026-10-12 (branch `dev`), after steps 1, 3 and 4 below.

## Where things stand

| Build | Before | Now | Free in the 120 KB region |
|---|---|---|---|
| Debug (`-Og`, "Build STM") | 90.5 KB | 71.2 KB | 48.8 KB |
| Release `-Os` (ISR path `-O2`), "Build STM release" | 81.7 KB | 62.7 KB | 57.3 KB |

RAM: 18.7 KB of 32 KB (was 19.1 KB).

**Flash layout** (STM32G431VB, 128 KB, 2 KB pages):

| Pages | Range | Use |
|---|---|---|
| 0–59 | 0x08000000–0x0801DFFF | Program (`FLASH` LENGTH = 120K in `STM32G431XX_FLASH.ld`) |
| 60–61 | 0x0801E000–0x0801EFFF | Reserved for CANopen storage (LSS node-ID/bit rate, OD persistence) |
| 62–63 | 0x0801F000–0x0801FFFF | Motor parameters, A/B slots (`params.c`) |

**CANopen budget (estimate, not measured):**
- **CANopenNode v4, CiA 301 slave plus LSS slave:** roughly 15–25 KB at `-Os`. That covers NMT, heartbeat, SDO server, EMCY, a few PDOs, and an object dictionary sized for this drive.
- **FDCAN HAL:** 2–3 KB more once it is actually used.
- **CiA 402 layer:** the FSA, mode switching, and the 0x6040/0x6041/0x6060/0x607A/0x60FF… objects mapped onto `g_mode`/`g_loop`/refs. Another ~4–8 KB plus its OD entries.
- **Total:** about 25–35 KB. That fits in either build today. Measure the chosen stack with an empty application before committing.
- **RAM:** about 2–4 KB more.

## Done

### 1. `-Os` release build with an `-O2` ISR path
- **Pragmas:** `motorCtrl.c` and `curSnsCtrl.c` start with `#pragma GCC optimize("O2")`, placed before the includes so the `pi_ctrl.h`/`cordicMgr.h` inlines are covered too. `mt6835.c` has the same pragma from `MT6835_DecodeAngle` to the end: the trigger, `MT6835_Sample()` and the IRQ handler. This costs 0.8 KB against a plain `-Os` build, and it applies to the debug build too.
- **Still at the build's level:** HAL's `HAL_ADC_IRQHandler` and the UART/DMA HAL calls used for telemetry, because `Drivers/` can't be edited. If the loop time gets tight in release, that is the place to look (or replace those calls with LL).
- **VS Code:** the tasks "Build STM release" and "Flash STM release" (`DEBUG=0 "OPTIMIZATION=-Os -g"`; `-g` adds no flash) and the launch config "Attach STM32 (release)" are new. "Build STM"/"Flash STM"/"Debug STM32" still use the debug build.
- **Stale objects:** `make` does not rebuild when only `OPTIMIZATION` changes. Delete `build/release/` after changing flags.

### 3. No `strtof`/`strtod`
- **Parsers:** `parse_f()` and `parse_q32()` in `calib.c` handle `[+-]digits[.digits][e[+-]digits]`. `a <turns>` parses straight to Q32.32 with no double math (exact to 1e-9 turn, |turns| < 2e9).
- **Behavior change:** commands with a missing or non-numeric argument (`v`, `a`, `g`, `i`, `s`) now fail with a usage message instead of reading 0. `l`/`q` with no argument still mean "default current, store".
- **`cal_kv()`:** the regression uses float running means (Welford) instead of `double` sums.
- **`strtok`:** in `set_param()` it was replaced by a manual split, because `strtok` calls `assert`, which linked `fprintf` and newlib's stdio/`malloc` (~4.5 KB).

### 4. No newlib `printf`
- **Formatter:** `say()` uses `fmt_v()` in `calib.c`, a subset `vsnprintf` with `%f` done in float. `__asm__(".global _printf_float")` is gone from `main.c`.
- **Checked on the host:** compiled with the host gcc and compared with the host libc's `vsnprintf`/`strtod` for every format string used in `calib.c` and about 30 parser inputs, including garbage, exponents and range limits.
- **Left:** libgcc 2.5 KB. That is float→double promotion for varargs (`__aeabi_f2d`/`d2f`, which share one object with the double add) plus 64-bit division, and it's not worth more effort.

## Not done / still available

### 2. Remove USB CDC: **skipped on purpose**
USB is planned for configuration. When that work starts, the console in `calib.c` can run over CDC unchanged: `say()` only needs a different transmit call, and `cmd_poll()` a different byte source. Then USART1 can carry telemetry only.

### 5. If more is needed later
- **LTO (`-flto`), roughly 5–7 KB:** STM32Make passes `-Wa,-a,-ad,-alms=...` listing flags per file, which LTO drops with a warning; remove them for LTO builds. Check that the weak HAL callbacks and `TIM1_UP_TIM16_IRQHandler` in `mt6835.c` still link to the right definitions. Check how `#pragma GCC optimize` interacts with LTO, and re-measure loop timing.
- **HAL UART → LL, ≈ 3 KB:** USART1 only does DMA TX plus byte RX. If USB takes over config, UART RX goes away too.
- **Shorter strings in `calib.c`, ≈ 1–3 KB:** do this last; the messages are useful at the bench.

## Must be verified on hardware (not done yet)
- **Loop timing:** `f` in both builds. `currentLoop` max must stay under ~19 µs, and encoder `late` must stay 0.
- **Commands:** `p`, then `s` with fractional, negative and exponent values (`s ld 2.2e-5`) and garbage (`s kv abc` must be rejected). Then `v 1000`, `v -500`, `a 2.5`, `a -0.25`, `g 0 1`, `i 0.5`, `l 8`, `q 8`.
- **`k`:** run kv and compare the result with the value saved before this change (same motor), because the regression arithmetic changed.

## Constraints to keep
- **CubeMX:** `Driver TCC.ioc` is the source of truth. Generated files only change inside `USER CODE` blocks (see CLAUDE.md).
- **Timing:** `currentLoop()` must finish within ~19 µs of the ADC trigger.
- **Parameter pages:** pages 62–63 stay out of the program region, and pages 60–61 stay free for CANopen.
- **Telemetry:** the frame layout and checksum must stay in sync with `software/plotter.py`.
- **No newlib text I/O:** see the CLAUDE.md gotcha. A stray `snprintf` or `strtof` brings back up to ~19 KB.

## How to measure
```
make -f STM32Make.make -j DEBUG=0 "OPTIMIZATION=-Os -g"   # release build -> build/release/, map -> build/Driver_TCC.map
python software/flash_report.py                         # component / object / section breakdown
arm-none-eabi-nm --size-sort -S build/release/Driver_TCC.elf | tail -40
```
The map file is shared: it describes whichever build linked last.
