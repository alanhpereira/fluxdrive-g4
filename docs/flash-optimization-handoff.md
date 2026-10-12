# Flash optimization handoff

**Goal:** free enough flash for a CANopen slave stack on FDCAN1, without slowing the 25 kHz control ISR.

Measured on 2026-10-12, firmware at the observer/params v4 state (branch `dev`).

## Where things stand

| Build | Flash used | Free in the 120 KB region |
|---|---|---|
| Debug (`-Og`, what "Build STM" produces) | 90.5 KB | 29.5 KB |
| Release `-Os` | 81.7 KB | 38.3 KB |
| Release `-O2` | 88.3 KB | 31.7 KB |
| Release `-Os -flto` | 74.4 KB | 45.6 KB |

**Flash layout** (STM32G431VB, 128 KB, 2 KB pages):

| Pages | Range | Use |
|---|---|---|
| 0–59 | 0x08000000–0x0801DFFF | Program (`FLASH` LENGTH = 120K in `STM32G431XX_FLASH.ld`) |
| 60–61 | 0x0801E000–0x0801EFFF | Free, kept in reserve (e.g. CANopen storage) |
| 62–63 | 0x0801F000–0x0801FFFF | Motor parameters, A/B slots (`params.c`) |

**CANopen budget (estimate, not measured):**
- **CANopenNode v4, minimal slave** (NMT + heartbeat, SDO server, EMCY, a few PDOs, an object dictionary sized for this drive): roughly 15–25 KB at `-Os`. Plus 2–3 KB more of `stm32g4xx_hal_fdcan` once it is actually used (only 572 bytes are linked today).
- **RAM:** about 2–4 KB more. RAM is at 19.1 KB of 32 KB today.
- **Before committing to a target,** build the chosen stack with an empty application and measure it.

With the debug build as it is, ~29 KB free is borderline. Steps 1–3 below give comfortable margin. Steps 4–5 are optional.

## Where the flash goes (`-Os` build)

From `python software/flash_report.py` (it reads `build/Driver_TCC.map`):

| Component | Bytes | Notes |
|---|---|---|
| Application | 23.6 K | `calib.o` alone is 16.4 K: 7.8 K code + 8.5 K strings (help text, messages) |
| HAL/LL drivers | 22.9 K | UART 4.1 K, TIM 3.0 K, ADC 4.7 K, PCD 3.4 K, I2C 1.7 K, SPI 1.5 K… |
| USB CDC stack | 9.8 K | **Initialized but never used** (`MX_USB_Device_Init()`, stock template) |
| `strtod`/`strtof` + bignum helpers | 7.9 K | From 7 call sites in `calib.c`, including one `strtod` for `a <turns>` |
| Rest of libc_nano | 6.3 K | `vfprintf`, `memcpy` and similar; mostly needed |
| CubeMX init code | 4.9 K | Needed |
| `printf` `%f` (`_printf_float`, `_dtoa_r`) | 4.5 K | Pulled in by `__asm__(".global _printf_float")` in `main.c` and 26 `%f` formats in `calib.c` |
| libgcc | 4.1 K | Soft-float **double** arithmetic (`strtod`, `double` in `run_pos()`/`cal_kv()`), `__udivmoddi4` (int64 division) |
| libm | 0.9 K | `logf` (inductance curve), `lroundf` |

`strtod` and `_dtoa_r` share the bignum helpers (`Balloc`, `mprec`, …). Those helpers, about 3 K of the 7.9 K, only go away once **both** `strtof`/`strtod` and `%f` are gone.

## Work items, in suggested order

### 1. Build with `-Os`, keep the ISR path fast. Saves ≈ 9 KB, low effort
- **Gotcha:** `make -f STM32Make.make` with the default `DEBUG=1` **ignores `OPTIMIZATION`** and always builds `-Og`. A release build needs `DEBUG=0 OPTIMIZATION=-Os`; it goes into `build/release/`.
- **Keep the hot path at `-O2`:** wrap `motorCtrl.c`, `curSnsCtrl.c` and the burst/DMA part of `mt6835.c` in `#pragma GCC optimize("O2")`, or mark the hot functions `__attribute__((optimize("O2")))`. The control loop budget is tight. `currentLoop()` measured 13 µs typical and 18.7 µs max at `-Og`, against about 19 µs available before the next counter bottom.
- **Decide what gets flashed:** `make flash` programs `$(RELEASE_DIRECTORY)/$(TARGET).elf`, so `make -f STM32Make.make -j DEBUG=0 OPTIMIZATION=-Os flash` flashes the release build. The VS Code tasks and the cortex-debug launch configs point at `build/debug/`. Decide which build is the everyday one and update `.vscode/tasks.json` / `launch.json` to match. Keep a debug configuration for stepping.
- **Verify:** use `f` for loop time and max (must stay under ~19 µs) and the encoder `late` counter (must stay 0). Then `r`, `v 1000`, `v 6000` with telemetry.

### 2. Remove the unused USB CDC device. Saves ≈ 10 KB plus RAM, low effort, CubeMX change
- **In `Driver TCC.ioc`:** disable the USB_DEVICE middleware and the USB peripheral (PA11/PA12, `USB_LP_IRQn`, `RCC.CK48CLockSelection`), then regenerate. **Don't** hand-delete `USB_Device/` or `MX_USB_Device_Init()`: CubeMX would put them back.
- **After regenerating, check:**
  - `TIM1_UP_TIM16_IRQn` is still disabled in the .ioc (see the CLAUDE.md gotcha).
  - `FLASH` LENGTH in the linker script is still 120K.
  - All `USER CODE` blocks survived.
- **Confirm nothing uses USB first:** grep for `CDC_Transmit` and `usbd`.
- **Verify:** it builds, `p` and telemetry work over USART1, and the RAM figure drops.

### 3. Replace `strtof`/`strtod` with a small decimal parser. Saves ≈ 5–7 KB, moderate effort
- **What to parse:** command arguments only need `[-]digits[.digits]`. The callers are `run_vel`, `run_pos`, `run_start` (`g`/`i`), `set_param`, and the `l`/`q` argument in `dispatch`.
- **`a <turns>`:** today it parses to `double` to keep Q32.32 resolution (`calib.c`, `run_pos`). Parse straight into `int64_t` Q32.32 instead (integer part `<< 32`, plus the fraction scaled by 2³²). That also removes the soft-double code from libgcc.
- **Check for other `double` math:** `cal_kv()` uses `double` accumulators for its regression, and the libgcc soft-double code stays linked if anything still uses `double`. Check with `arm-none-eabi-nm --size-sort` for `__aeabi_d*` after the change.
- **Verify:** `s`, `v`, `a`, `g`, `i`, `l 8` and `q 8` with positive, negative and fractional values, plus garbage input (it should be rejected or read as 0, as today).

### 4. Drop `printf` `%f`. Saves ≈ 4.5 KB, plus ≈ 3 KB of shared bignum code if step 3 is done; moderate effort, touches many strings
- **Replace the formatting:** write a small fixed-point formatter, for example `fmt_f(char* buf, float v, int decimals)`, or print scaled integers. Then remove `__asm__(".global _printf_float")` from `main.c` (it sits in a USER CODE block).
- **There are 26 `%f` sites in `calib.c`.** `print_params()` and the calibration results are the bulk.
- **Watch the protocol:** the plotter parses binary frames, not text, so text formatting changes don't affect it. But keep `p` output readable, because people read it.

### 5. Optional, if more is needed
- **LTO (`-flto`), ≈ 7 KB more:** STM32Make passes `-Wa,-a,-ad,-alms=...` listing flags per file, which LTO drops with a warning; remove them for LTO builds. Check that the weak HAL callbacks (`HAL_ADCEx_InjectedConvCpltCallback` etc.) and `TIM1_UP_TIM16_IRQHandler` in `mt6835.c` still link to the right definitions, and re-measure loop timing.
- **HAL UART → LL, ≈ 3 KB:** USART1 only does DMA TX of frames and text, plus RX polling (`HAL_UART_Transmit_DMA`, `huart1.gState`). Only worth it if needed.
- **Shorter strings in `calib.c`, ≈ 1–3 KB:** the help text and messages are 8.5 KB. Do this last; the messages are useful at the bench.

## Constraints to keep
- **CubeMX:** `Driver TCC.ioc` is the source of truth. Generated files only change inside `USER CODE` blocks (see CLAUDE.md).
- **Timing:** `currentLoop()` must finish within ~19 µs of the ADC trigger. Any optimization-level change needs a `f` check on hardware.
- **Parameter pages:** pages 62–63 must stay out of the program region, and the parameter layout must not change as a side effect (`PARAM_VERSION` 4).
- **Telemetry:** the frame layout and checksum must stay in sync with `software/plotter.py`.

## How to measure
```
make -f STM32Make.make -j DEBUG=0 OPTIMIZATION=-Os     # release build -> build/release/, map -> build/Driver_TCC.map
python software/flash_report.py                         # component / object / section breakdown
arm-none-eabi-nm --size-sort -S build/release/Driver_TCC.elf | tail -40
```
The map file is shared: it describes whichever build linked last.
