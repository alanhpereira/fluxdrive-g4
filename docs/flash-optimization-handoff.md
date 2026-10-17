# Flash optimization handoff

**Goal:** free enough flash for a CANopen slave stack on FDCAN1, without slowing the 25 kHz control ISR:
- CiA 301 (NMT, SDO, PDO, EMCY, heartbeat)
- CiA 305 LSS (node-ID and bit rate set over the bus)
- CiA 402 drive profile (FSA state machine, controlword/statusword, PP/PV/HM/CSP modes)

The USB CDC stack **stays**: configuration is planned to move from USART1 to USB.

Updated 2026-10-12 (branch `dev`), after steps 1, 3, 4 and 6 below.

> **Update 2026-10-17, CANopen in (branch `Feat/CANOpen`, `docs/canopen-handoff.md`):** measured, not estimated. Release `-Os` 96.5 KB (23.5 KB free), debug 105.8 KB (14.2 KB free), with all CANopen sources at `-Os` in both builds through a pragma in `CO_driver_target.h`. CANopenNode (no LSS) plus the full CiA 402 layer (all six modes) and the FDCAN HAL added about 30 KB, the top of the estimate below. RAM: 19.9 of 22 KB used including the 4 KB stack reserve (about 2.1 KB free). CCM: release 9.2 KB, debug 9.4 KB. Pages 60–61 now hold the CANopen record (`co_store.c`). Tables below are the 2026-10-12 state.

## Where things stand

| Build | Before | Now | Free in the 120 KB region |
|---|---|---|---|
| Debug (`-Og`, "Build STM") | 90.5 KB | 71.4 KB | 48.6 KB |
| Release `-Os` (ISR path `-O2`), "Build STM release" | 81.7 KB | 63.1 KB | 56.9 KB |

"Now" is `.text` + `.data` from `arm-none-eabi-size` (decimal KB). The CRC table moving to flash cost +0.2 KB; the CCM moves below cost about +0.1 KB.

**RAM** (static `.data` + `.bss`, plus the stack reserve in the linker script):

| | Before step 6 | Now |
|---|---|---|
| Static `.data` + `.bss` | 13.2 KB | 10.6 KB |
| Reserved heap + stack | 6.1 KB (2 KB heap, 4 KB stack) | 4.1 KB (0 heap, 4 KB stack) |
| Free SRAM for new static data | 3.2 KB | 7.9 KB |

**CCM SRAM** (ISR code, 10 KB): release 8160 of 10240 B (2080 B free). Debug 8400 B (1840 B free). Before the CCM step: release 9712 B, debug 9944 B (296 B free).

**Flash layout** (STM32G431VB, 128 KB, 2 KB pages):

| Pages | Range | Use |
|---|---|---|
| 0–59 | 0x08000000–0x0801DFFF | Program (`FLASH` LENGTH = 120K in `STM32G431XX_FLASH.ld`) |
| 60–61 | 0x0801E000–0x0801EFFF | CANopen record, A/B slots (`co_store.c`): node ID, bit rate, OD persistence |
| 62–63 | 0x0801F000–0x0801FFFF | Motor parameters, A/B slots (`params.c`) |

**CANopen budget (estimate, not measured):**
- **CANopenNode v4, CiA 301 slave plus LSS slave:** roughly 15–25 KB at `-Os`. That covers NMT, heartbeat, SDO server, EMCY, a few PDOs, and an object dictionary sized for this drive.
- **FDCAN HAL:** 2–3 KB more once it is actually used.
- **CiA 402 layer:** the FSA, mode switching, and the 0x6040/0x6041/0x6060/0x607A/0x60FF… objects mapped onto `g_mode`/`g_loop`/refs. Another ~4–8 KB plus its OD entries.
- **Total:** about 25–35 KB. That fits in either build today. Measure the chosen stack with an empty application before committing.
- **RAM:** about 2–4 KB more. 7.9 KB is free now (see section 6), so that fits. The real CAN usage is still unmeasured.

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
> **Superseded 2026-10-12:** the text console, with `say()`/`fmt_v()`/`parse_*`, was removed (phase 3 of `docs/config-gui-handoff.md`); the firmware no longer formats text. `EV()` passes float arrays, so the `__aeabi_f2d` part of libgcc is gone too (libgcc 1.45 KB). Release `-Os`: 56 268 B.

- **Formatter:** `say()` uses `fmt_v()` in `calib.c`, a subset `vsnprintf` with `%f` done in float. `__asm__(".global _printf_float")` is gone from `main.c`.
- **Checked on the host:** compiled with the host gcc and compared with the host libc's `vsnprintf`/`strtod` for every format string used in `calib.c` and about 30 parser inputs, including garbage, exponents and range limits.
- **Left:** libgcc 2.5 KB. That is float→double promotion for varargs (`__aeabi_f2d`/`d2f`, which share one object with the double add) plus 64-bit division, and it's not worth more effort.

### 6. RAM and CCM: duplicate buffers and runtime tables
- **Ld/Lq decay recorder (`calib.c`):** `g_rec` (2400 B, 600 floats) was copied into a static `buf` of the same size before the fit. `print_flux_curve()` and `decay_tau()` now take `const volatile float*` and read `g_rec` directly. The ISR stops writing `g_rec` at state 3, before the fit runs, so this is safe.
- **CRC-8 table (`mt6835.c`):** the 256-byte table was built in RAM at the first burst start. It is now a `const` table in flash, and the init code and its call are gone. Net flash +216 B. The ISR reads it from flash (up to 3 lookups per sample), so check the loop time.
- **Heap reserve (`STM32G431XX_FLASH.ld`):** `_Min_Heap_Size` 0x800 → 0. Nothing calls `malloc`: USB uses static buffers, and `_sbrk` is only reached through `malloc`. If something later needs `malloc` (for example a CANopen stack), it gets NULL, so restore a heap then. `_Min_Stack_Size` stays at 0x1000 as a guard: a static-data overflow fails at link time.
- **Stack use:** the largest frames are about 150–220 B (`params_save` 224 B, `proto_poll` 184 B, `currentLoop` 152 B). The deepest chain is around 1 KB by estimate. Nothing has been measured at run time.

**CCM reduction (applied):** the ISR path is still `-O2`. Cold code in it moved to flash:
- `send_telemetry()` (`motorCtrl.c`): the frame packing and `HAL_UART_Transmit_DMA` call, once every 10 cycles. The UART/DMA helpers (`HAL_UART_Transmit_DMA`, `HAL_DMA_Start_IT`, `DMA_SetConfig`) left the `.ccmram` list. `HAL_GetTick` stays, because `SVPWM` calls it every cycle.
- `outer_loops()`: `noinline`, so its body (every 10 cycles) runs from flash. Its cheap path still runs every cycle from flash, so check the loop time.
- `pol_step()` (`hfi.c`): `noinline`, only during the polarity pulses.

Effect: CCM −1552 B in release (9712 → 8160), −1544 B in debug (9944 → 8400). The change is only a move and a call, so no algorithm changed, but the ISR path now makes a few calls into flash: check the max loop time before relying on it.

**Measured, not applied** (each one changes timing or behavior):
| Change | CCM (debug) | Note |
|---|---|---|
| Drop `#pragma GCC optimize("O2")` from `motorCtrl.c` (`currentLoop` at `-Og`) | −2832 B | Biggest, but the loop gets slower. |
| Drop the O2 pragma from `hfi.c` | −800 B | `hfi_step` at `-Og`. |
| `currentLoop` at `-Os` (release) | −1264 B | Measured earlier; applies to release only. |
| `HAL_ADC_IRQHandler` replaced by direct JDR reads (`stm32g4xx_it.c`) | ~−600 B | Must keep the overrun and error handling. |

Not yet measured: LTO (see section 5), and running `hfi_step` faster. `hfi_step` is 2428 B at `-Os` and is the largest CCM item after `currentLoop`.

## Not done / still available

### 2. Remove USB CDC: **skipped on purpose**
USB is planned for configuration. When that work starts, the binary protocol in `proto.c` runs over CDC (phase 4 of `docs/config-gui-handoff.md`). Then USART1 can carry telemetry only.

### 5. If more is needed later
- **LTO (`-flto`), roughly 5–7 KB:** STM32Make passes `-Wa,-a,-ad,-alms=...` listing flags per file, which LTO drops with a warning; remove them for LTO builds. Check that the weak HAL callbacks and `TIM1_UP_TIM16_IRQHandler` in `mt6835.c` still link to the right definitions. Check how `#pragma GCC optimize` interacts with LTO, and re-measure loop timing.
- **HAL UART → LL, ≈ 3 KB:** USART1 only does DMA TX plus byte RX. If USB takes over config, UART RX goes away too.

## Must be verified on hardware (not done yet)
- **Loop timing:** Config tab diagnostics in both builds. `currentLoop` max must stay under ~19 µs, and encoder `late` must stay 0.
- **CCM moves (section 6):** `currentLoop` max time and the encoder `late` counter, since the telemetry and outer-loop calls now run from flash.
- **CRC table in flash (section 6):** `currentLoop` max time and the encoder `late`/CRC counters in diagnostics, against the build before this step.
- **Commands:** now through the Config tab: parameter writes (including out-of-range values, which must be refused), speed ±, position ±, current, Ld/Lq at an explicit test current.
- **kv calibration:** run kv and compare the result with the value saved before this change (same motor), because the regression arithmetic changed.

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
arm-none-eabi-size -A build/release/Driver_TCC.elf | grep -E 'ccmram|\.bss|\.data'   # CCM and static RAM
```
The map file is shared: it describes whichever build linked last.
