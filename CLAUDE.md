# CLAUDE.md

This file guides Claude Code (claude.ai/code) when it works in this repository.

## What this is

This is the firmware for a graduation-thesis BLDC/PMSM motor driver. It runs field-oriented control (FOC) on an STM32G431VB (STSPIN32G4-class part, Cortex-M4F, 170 MHz, 128 KB flash, 32 KB RAM). The hardware it drives:
- A 3-phase bridge on TIM1, plus a gate driver on I2C3 at address `0x47`. The driver's nFAULT pin is PE15, which also feeds the TIM1 break input.
- Three low-side shunts (2 mΩ) read through the internal OPAMPs (gain 16). ADC1 and ADC2 sample them as injected conversions, triggered by TIM1 CC4.
- An MT6835 21-bit magnetic encoder on SPI1 (10.6 MHz), with nCS on PA15. It stays in burst mode (CS held low), and a TIM1-triggered DMA read of a few angle groups runs once per PWM period.
- USART1 at 2 Mbaud. It carries both the text command console and the binary telemetry.

`software/plotter.py` is the PC-side GUI (PyQt + pyqtgraph). It shows the telemetry and sends commands.

## Build / flash / debug

The project is driven by the **STM32 for VSCode** extension. `STM32Make.make` is generated from `STM32-for-VSCode.config.yaml` plus the CubeMX `Makefile`. Toolchain paths are in `.stm32env`: arm-none-eabi-gcc 13.2 and OpenOCD 0.12 under `C:/DEV/TOOLS/...`.

- Build: VS Code task "Build STM", or `make -f STM32Make.make -j`. Output goes to `build/debug/Driver_TCC.elf`. That debug build is always `-Og`: `OPTIMIZATION` is only used with `DEBUG=0` (e.g. `make -f STM32Make.make -j DEBUG=0 OPTIMIZATION=-Os`), which builds into `build/release/`; `make ... DEBUG=0 ... flash` flashes that one. Flash budget and trimming plan: `docs/flash-optimization-handoff.md`, measured with `python software/flash_report.py`.
- Flash: task "Flash STM", or `make -f STM32Make.make flash`. This uses ST-Link through OpenOCD (`openocd.cfg`).
- Erase: `make -f STM32Make.make erase`. This also wipes the stored motor parameters.
- Debug: the cortex-debug launch configs "Debug STM32" and "Attach STM32" in `.vscode/launch.json`, with live watch enabled.
- Plotter: `pip install -r software/requirements.txt`, then `python software/plotter.py [COMx]`.

There are no unit tests. Changes are verified on hardware through the serial console and telemetry.

## CubeMX rules (important)

`Driver TCC.ioc` is the source of truth for the peripheral setup. Everything below is CubeMX-generated, and CubeMX overwrites it when the project is regenerated:
- `Core/Src/{adc,dac,dma,fdcan,gpio,i2c,opamp,spi,tim,usart,cordic}.c`
- `stm32g4xx_hal_msp.c`, `stm32g4xx_it.c`
- `USB_Device/`, `Drivers/`, `Middlewares/`, `Makefile`, the linker script, and the startup file

Code you add to those files **must** sit between `/* USER CODE BEGIN x */` and `/* USER CODE END x */` markers, or CubeMX will delete it. `main.c` follows the same rule. All the hand-written logic is in its own files, listed below. USB CDC is initialized but unused (stock template).

The flash length in the linker script is set to 120K on purpose. Pages 60 and 61 are kept free (reserved, e.g. for CANopen storage), and the last two 2 KB pages (62 and 63) hold the parameters. If CubeMX regenerates `STM32G431XX_FLASH.ld`, set FLASH back to 120K (never above 124K).

Formatting is set by `.clang-format`: Google base style, **tabs**, width 4, no column limit, and `else` on a new line after `}`.

## Architecture

### Hand-written modules (`Core/Src` + `Core/Inc`)
| File | Role |
|---|---|
| `motorCtrl.c/h` | `currentLoop()`, the FOC ISR body: Clarke/Park, current PI, SVPWM, PLL position/speed estimate, outer speed/position loops, step recorder, telemetry frames. `initMotor()` brings everything up. |
| `curSnsCtrl.c/h` | ADC/OPAMP/DAC setup. `HAL_ADCEx_InjectedConvCpltCallback` waits for both ADC1 and ADC2 to finish, scales the currents and the bus voltage, then **calls `currentLoop()`**. |
| `bridgeCtrl.c/h` | TIM1 PWM start/stop, `setDuty()`/`setSamplePoint()` (CCR1-4), and the DAC2 short-circuit reference. `checkAndClearBridgeFault()` reads the gate-driver fault register over I2C and re-arms MOE after a break. |
| `mt6835.c/h` | Encoder driver: register R/W, zero, EEPROM. Burst mode: `MT6835_StartBurst()` sends one burst command and leaves CS low. The sensor then shifts out 4-byte groups (angle H/M/L+status, CRC) whenever the SPI clocks. `TIM1_UP_TIM16_IRQHandler` (defined here) starts a register-level DMA read of `MT6835_GROUPS` groups at the TIM1 counter bottom, sized to end just before the ADC sample. `MT6835_Sample()`, called first in `currentLoop()`, takes the last complete group, checks its CRC, updates `hMT6835.angle` (21-bit) and the multi-turn `turns` counter, and returns the sample age. |
| `calib.c/h` | Serial command console (`cmd_init`/`cmd_poll`), calibration routines, run modes, safety watchdog. Owns the shared ISR↔main globals (`g_mode`, `g_loop`, refs, PI states). |
| `params.c/h` | `motor_params_t g_params`, stored in flash. Two A/B slots with seq number + CRC32. Versioned (v4); a copy with any other version is ignored and the defaults load. |
| `pi_ctrl.h` | Inline PI with conditional-integration anti-windup. `pi_current_step()` shares the voltage circle between d and q, with d taking priority. |
| `cordicMgr.h` | Hardware CORDIC sin/cos (`sincos_q31`, Q32 angle in, floats out). |

### Execution model
- **ISR context (25 kHz):** TIM1 is center-aligned with ARR 3399, giving 25 kHz PWM. CC4 (`setSamplePoint`) triggers the injected ADC conversions. The ADC callback calls `currentLoop()`. The outer speed/position loops run every `OUTER_DIV` = 10 cycles, at 2.5 kHz. A telemetry frame goes out every 10 cycles over UART DMA.
- **Main loop:** `cmd_poll()` (command parsing plus `run_watchdog()`), then `checkAndClearBridgeFault()`. Calibration routines block in main with `HAL_Delay` and drive the ISR through volatile globals.
- Encoder timing: the TIM1 update interrupt at the counter bottom starts the read. 6 groups take 18.1 µs and end about 1 µs before the ADC samples (at 19.6 µs), so the last group is ≈4 µs old at the current sample. The exact age comes from the TIM1 count recorded at the trigger. `currentLoop()` must finish before the next bottom (≈19 µs after the ADC), or the read starts late. `f` prints the loop time and the encoder late/stall/CRC counters; if `late` grows, lower `MT6835_GROUPS`. `currentLoop()` advances the angle with the PLL speed for the Park transform and further, by `OUT_DELAY_S`, for the inverse Park (`ENC_EXTRA_S`, `ANGLE_COMP` in `motorCtrl.c`). The PLL compares at the true sample time.
- Flux observer (`obs_update()`/`obs_weight()` in `motorCtrl.c`): a nonlinear flux observer runs every cycle in the αβ frame, using R and L from params and λ derived from `kv`. Above `obs_lo_v` of back-EMF its angle is blended into the commutation angle, reaching full weight at `obs_hi_v`, but only when `obs_blend` is 1 and the observer has passed its flux check and agreed with the encoder for `OBS_LOCK_N` cycles. Once it is locked, only a sustained failure drops it (leaky counters `OBS_FLUX_DROP_N`/`OBS_ENC_DROP_N`), or MOE going off. The speed/position loops slew their speed reference at `acc_lim` (rpm/s), which keeps transient current, and with it the observer's load-dependent flux error, small. A second PLL (`obs_pll()`) on the observer angle gives an encoder-independent speed, blended into `s_vel` (what the speed loop, `g_vel` and the output advance use) with the same weight. Position always comes from the encoder. Telemetry `v[11..15]` carries the observer−encoder error (deg elec), |flux|/λ, the blend weight, observer rpm and encoder rpm. `atan2_q32()` switches the CORDIC to phase mode and back, so call it only from the ISR.
- All IRQs (ADC, TIM1 update, UART, TIM1 break) are at **priority 0**, so they cannot preempt each other. `hMT6835.turns`/`angle` are only written inside `currentLoop()`, so it reads them directly. From thread context, use `MT6835_Position()` instead.
- `CTRL_FS_HZ` in `calib.h` must match the real ISR rate. PI gains, the PLL, and the inductance decay timing all depend on it.

### Control modes
- `g_mode`:
  - `CTRL_OFF`: zero volts, bridge still switching, integrators reset.
  - `CTRL_CAL`: open-loop `g_vd_cmd`/`g_vq_cmd` at the forced angle `g_theta_cmd`.
  - `CTRL_RUN`: closed-loop current control.
- In `CTRL_RUN`, `g_loop` picks what drives `g_iq_ref`:
  - `LOOP_CURRENT`: set by the command.
  - `LOOP_VEL`: speed PI.
  - `LOOP_POS`: position P → speed PI. Switching in is bumpless.
- Main hands values to the ISR through these rules:
  - Write the reference first, then switch the mode/loop.
  - The 64-bit position ref goes through `g_pos_ref_new` + the `g_pos_ref_req` handshake, because 64-bit writes are not atomic on the M4.
  - Homing uses `g_home_req`.

### Units / fixed-point conventions
- Angles are Q32: `2^32` = 360°. That covers the electrical angle, `elec_offset`, and `g_theta_cmd`.
- The encoder gives 21 bits per turn: `angle << 11` is a mechanical Q32 angle.
- Position is `pos_q32_t`, an int64 Q32.32 in turns. `g_pos` (float) is for display only.
- Speed is in turn/s internally. Commands and telemetry use rpm.
- `params` are in SI units (Ω, H, A, V). The observer/ramp settings (`oblend`, `olo`, `ohi`, `ogain`, `oftol`, `oerr`, `acc`; fields `obs_*`, `acc_lim`, added in v4) are params too. `ki` is stored as continuous-time; `pi_apply()` turns it into `ki_ts`. Call `pi_apply()` after any gain change.

### Serial protocol (USART1)
- **PC → MCU:** single-letter commands, one per line, ending in LF. Type `?` for help. The commands:
  - Calibration: `e` encoder dir/pole pairs/offset (averaged over one turn each way), `r` R, `l` Ld, `q` Lq (hold the shaft). `l <A>`/`q <A>` measure at that current and only print (secant L curve from the decay); without an argument they store, `t` PI autotune from R, L and `bw`, `k` kv (the motor spins).
  - Run modes: `g id iq`, `i iq`, `v rpm`, `a turns`.
  - Other: `h` home, `o` off, `p` print params, `s key val` set param, `w` save to flash, `z` reset the encoder zero, `x` abort.
  - Prerequisites are enforced through the `PF_*` flags in `g_params.flags`. The usual order is `e` → `r` → `l` → `t`. `v`/`a` also need `vkp`/`vki`/`vlim` set.
- **MCU → PC:** a 72-byte binary `Frame`: `A5 5A`, `seq u16`, 16 floats, then a `sum u32` computed with rotate-left-1-and-add from seed `0x5A5AA5A5`. While a command prints text, `begin()` clears `g_telem_en` so text and frames never mix. Commands call `finish()` afterwards, which re-enables telemetry.
- **Keep in sync:** if you change the frame layout, `frame_sum()`, or what `telemetry.v[i]` holds, update `PLOTS`/`frame_ok()` in `software/plotter.py` too.

### Parameters in flash
- `params_save()` refuses to run while MOE is set, because the page erase stalls the CPU about 20 ms. `w` therefore disables the bridge, saves, and re-inits it.
- When changing `motor_params_t`:
  - Keep `crc` last and the size a multiple of 8. Give new fields a default in `k_defaults`.
  - Bump `PARAM_VERSION`. There is no migration: after a layout change the stored copy is ignored, the defaults load, and everything must be recalibrated and saved with `w`.

## Gotchas
- Keep the TIM1 update interrupt (`TIM1_UP_TIM16_IRQn`) **disabled in the .ioc**. `mt6835.c` enables it at runtime and owns `TIM1_UP_TIM16_IRQHandler`. If CubeMX generates a second handler, the link fails. The SPI1 DMA channels are reprogrammed by register on every read; leave them in NORMAL mode in the .ioc.
- Current and voltage scaling come from the hard-coded constants and offsets in `curSnsCtrl.c` (`CS_SHUNT`, `CS_GAIN`, `cur_offset[]`). The stored `cur_scale`, `cur_off`, `vin_scale`, `vbus_max` and `temp_max` params are **not used** by the control path yet.
- `SVPWM()` writes nothing for `FAULT_CLEAR_MIN_TIME` ms after a fault is cleared.
- Leaving `CTRL_RUN` while the motor spins applies a zero vector, which short-brakes it. `run_stop()` and `kv_coast()` exist to slow down first. Keep that behavior.
- `__asm__(".global _printf_float")` in `main.c` enables `%f` in newlib-nano `printf`. `-fno-math-errno` makes `sqrtf` compile to a single `vsqrt` instruction.
