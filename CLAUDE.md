# CLAUDE.md

This file guides Claude Code (claude.ai/code) when it works in this repository.

## What this is

Firmware, PC software and hardware of a graduation-thesis BLDC/PMSM motor driver. It runs field-oriented control (FOC) on an STM32G431VB (inside the STSPIN32G4, Cortex-M4F, 170 MHz, 128 KB flash, 32 KB RAM). The hardware:
- A 3-phase bridge on TIM1, plus a gate driver on I2C3 at address `0x47`. The driver's nFAULT pin is PE15, which also feeds the TIM1 break input.
- Three low-side shunts (2 mΩ) read through the internal OPAMPs (gain 16). ADC1 and ADC2 sample them as injected conversions, triggered by TIM1 CC4.
- An MT6835 21-bit magnetic encoder on SPI1 (10.6 MHz), nCS on PA15. It stays in burst mode (CS held low); a TIM1-triggered DMA read of a few angle groups runs once per PWM period.
- USB CDC (full-speed virtual COM port, ST VID/PID 0483:5740) carries the binary telemetry and the binary config protocol (no text console). USART1 at 2 Mbaud is the build-time fallback: `PROTO_LINK_USB 0` in `link.h`.
- CANopen (CiA 301/402) on FDCAN1 carries normal operation.

`software/plotter.py` is the PC GUI (PyQt + pyqtgraph): telemetry, parameters, calibration, and a CANopen master tab.

## Documentation
- `docs/user-manual-driver.md`: using the driver (pin-outs, commissioning, modes, protections, CANopen tables, parameter reference).
- `docs/user-manual-software.md`: using the PC tools.
- `docs/design-notes.md`: design decisions, bench measurements and open items (config protocol, CANopen drive and PC tool, HFI, sensorless tuning, flash budget).
- Hardware: KiCad project in `hardware/`, schematic PDF `hardware/Driver TCC schematic.pdf`, datasheets in `hardware/datasheets/`.

Keep the manuals in step when a change affects what a user sees (parameters, status codes, GUI controls, pin use).

## Build / flash / debug

The project is driven by the **STM32 for VSCode** extension. `STM32Make.make` is generated from `STM32-for-VSCode.config.yaml` plus the CubeMX `Makefile`. Toolchain paths are in `.stm32env`: arm-none-eabi-gcc 13.2 and OpenOCD 0.12 under `C:/DEV/TOOLS/...`.

- Build: VS Code task "Build STM", or `make -f STM32Make.make -j` → `build/debug/Driver_TCC.elf`, always `-Og`. `OPTIMIZATION` is only used with `DEBUG=0` (`make -f STM32Make.make -j DEBUG=0 OPTIMIZATION=-Os`) → `build/release/`; tasks "Build STM release" / "Flash STM release" (`-Os -g`) and the launch config "Attach STM32 (release)" go with it. `make` does not rebuild when only `OPTIMIZATION` changes: delete `build/release/` when you switch flags. Flash use per module: `python software/flash_report.py`.
- Flash: task "Flash STM", or `make -f STM32Make.make flash` (ST-Link through OpenOCD, `openocd.cfg`). Erase: `make -f STM32Make.make erase`, which also wipes the stored motor parameters and CANopen record.
- Debug: cortex-debug launch configs "Debug STM32" and "Attach STM32" in `.vscode/launch.json`, live watch enabled.
- Plotter: `pip install -r software/requirements.txt`, then `python software/plotter.py [COMx]`. Numeric inputs use the C locale (`.` decimals) on purpose; keep it that way for new inputs.
- CANopen tests: `python software/cotest.py --can slcan:COM8 [--usb COM7] [names…]` (`--list`, `--no-motion`), the same runner as the CANopen tab's Tests; results in `software/cotest_runs/` (git-ignored). Core: `software/co_link.py` (no Qt). It patches python-can's SLCAN reader/writer and sets a 0.5 ms GIL switch interval in the GUI so the software SYNC stays on time (`docs/design-notes.md`, CANopen PC tool).
- `python software/check_sync.py` checks the PC tables against the firmware sources and the EDS.
- New hand-written `Core/Src/*.c` files must be in `C_SOURCES` in `STM32Make.make`. The extension adds them when it regenerates the file; for command-line builds, add them by hand. Sources and include paths under `CANopen/` are listed in `STM32-for-VSCode.config.yaml` (`sourceFiles`, `includeDirectories`), so a regenerate keeps them.
- CANopen object dictionary: `driverTCC.xdd` (repo root) is the CANopenEditor project. `CANopen/od/OD.c`/`OD.h` and `software/driverTCC.eds` are **exported** from it by the user in CANopenEditor; never write or hand-edit them. After an XDD change, stop and let the user export, then build.

There are no unit tests. Changes are verified on hardware through the plotter (Config tab and telemetry) and `cotest.py`.

## CubeMX rules (important)

`Driver TCC.ioc` is the source of truth for the peripheral setup. Everything below is CubeMX-generated, and CubeMX overwrites it when the project is regenerated:
- `Core/Src/{adc,dac,dma,fdcan,gpio,i2c,opamp,spi,tim,usart,cordic}.c`
- `stm32g4xx_hal_msp.c`, `stm32g4xx_it.c`
- `USB_Device/`, `Drivers/`, `Middlewares/`, `Makefile`, the linker script, and the startup file

Code you add to those files **must** sit between `/* USER CODE BEGIN x */` and `/* USER CODE END x */` markers, or CubeMX deletes it. `main.c` follows the same rule. All hand-written logic is in its own files (below).
- USB CDC hooks are in USER CODE sections of `usbd_cdc_if.c` (receive, control/DTR, TX complete), `stm32g4xx_it.c` (`USB_LP_IRQn 0`: `link_usb_pump()`) and `usbd_conf.c` (`USB_MspInit 1`: USB IRQ priority 1 and CRS trimming HSI48 to the SOF). The .ioc sets USB_LP to priority 1 and `APP_RX/TX_DATA_SIZE` to 64.
- The .ioc has FDCAN1 at 1 Mbit/s (prescaler 10, Seg1 13, Seg2 3, SJW 3) with AutoRetransmission on; the port re-times it at runtime for other bit rates. The FDCAN1 and TIM7 interrupts are **not** in the .ioc (see Gotchas).
- Reserved for future features, set up in the .ioc but unused by the firmware: TIM4 as an ABI encoder interface (J8), the Hall inputs PC0–PC2 as EXTI0–2 (J13, empty handlers), the DO1–DO3 pins (J10/J11), the LED on PC13 and the temperature inputs (PC3 motor, PC4 board NTC). Leave them configured. Note that the Hall lines have no pull-ups on the board or in the .ioc and their EXTIs sit at priority 0: with J13 empty, a floating pin can fire them. Add pulls (or drop the EXTIs until Hall support exists) if the loop time shows unexplained jitter.

The flash length in the linker script is 120K on purpose. Pages 60 and 61 hold the CANopen record (`co_store.c`), and pages 62 and 63 hold the parameters. If CubeMX regenerates `STM32G431XX_FLASH.ld`, set FLASH back to 120K (never above 124K).

The 25 kHz ISR path runs from CCM SRAM. The linker script splits RAM into `RAM` (22K at 0x20000000) and `CCMRAM` (10K at 0x10000000). A `.ccmram` output section placed **before** `.text` (the first matching rule wins) collects `currentLoop`, `SVPWM`, `hfi_*`, `outer_loops`, the ADC IRQ (`ADC1_2_IRQHandler`, `cs_adc_irq`), `MT6835_Sample`, `TIM1_UP_TIM16_IRQHandler`, `link_tx_ready`/`link_defer` and `HAL_GetTick`. The telemetry fill (`send_telemetry`, every 10th cycle, never an outer-loop cycle) stays in flash to leave room. `main()` copies the section from flash in `USER CODE BEGIN 1`, before `HAL_Init`. From flash the loop is too slow (max 23.9 µs with HFI). A CubeMX regenerate drops all of this, so put it back. New per-cycle functions belong in that list; `--print-memory-usage` shows the CCMRAM fill (≈9.6 of 10 KB).

Formatting is set by `.clang-format`: Google base style, **tabs**, width 4, no column limit, and `else` on a new line after `}`.

## Architecture

### Hand-written modules (`Core/Src` + `Core/Inc`)
| File | Role |
|---|---|
| `motorCtrl.c/h` | `currentLoop()`, the FOC ISR body: Clarke/Park, current PI, SVPWM, PLL position/speed estimate, flux observer, outer speed/position loops, trips, step recorder, telemetry frames. `initMotor()` brings everything up. |
| `curSnsCtrl.c/h` | ADC/OPAMP/DAC setup. `cs_adc_irq()` is the whole ADC1_2 interrupt (called from its USER CODE section, which returns before the HAL handlers): one interrupt per cycle at ADC2's end of sequence (JEOS; `initCS()` sets the IER bits by register). It reads both sequences, scales the currents and the bus voltage, then **calls `currentLoop()`**, ~2.0–2.5 µs after the CC4 trigger. |
| `bridgeCtrl.c/h` | TIM1 PWM start/stop, `setDuty()`/`setSamplePoint()` (CCR1-4), and the DAC short-circuit reference. `checkAndClearBridgeFault()` reads the gate-driver fault register over I2C and re-arms MOE after a break. |
| `mt6835.c/h` | Encoder driver: register R/W, zero, EEPROM. Burst mode: `MT6835_StartBurst()` sends one burst command and leaves CS low; the sensor then shifts out 4-byte groups (angle H/M/L+status, CRC) whenever the SPI clocks. `TIM1_UP_TIM16_IRQHandler` (defined here) starts a register-level DMA read of `MT6835_GROUPS` groups at the TIM1 counter bottom, sized to end just before the ADC sample. `MT6835_Sample()`, called first in `currentLoop()`, takes the last complete group, checks its CRC, updates `hMT6835.angle` (21-bit) and the multi-turn `turns` counter, and returns the sample age. |
| `calib.c/h` | Command entry (`cmd_message()`: SAVE, CAL, RUN_*, STOP, HOME), calibration routines, run modes, safety watchdog. `cmd_poll()` = `run_watchdog()` + `proto_poll(0)` + `co_app_poll()` + `co402_poll()` + deferred 0x1010 saves. Owns the shared ISR↔main globals (`g_mode`, `g_loop`, refs, PI states) and the `drive_*()` API the CiA 402 layer uses (`drive_check/enter/coast/trip_reset`, `g_can_owner`, `g_stop_reason`). |
| `hfi.c/h` | High-frequency injection: square wave on the estimated d axis (half wave `hfi_half` ≥ 2 periods), demodulation from consecutive samples, angle PLL, polarity pulses, and the PI current filter. `hfi_step()` is called from `currentLoop()` in `CTRL_RUN`; the HFI calibrations in `calib.c` drive it through `g_hfi_cmd`. |
| `boot.c/h` | `MSG_REBOOT`: `boot_reboot()` drops USB (so the host sees the port go) and resets; for DFU it first leaves a magic in TAMP backup register 0. `boot_check()`, the first call in `main()`, sees the magic, clears it and jumps to the ST system bootloader at 0x1FFF0000. |
| `link.c/h` | The byte pipe. `link_tx()` is called from main (replies) and from the link IRQ (telemetry, through `link_defer()`), never at priority 0; it masks priority ≥ 1 with BASEPRI. USB (default): a 2 KB TX ring; producers pend `USB_LP_IRQn`, and only the USB IRQ calls into the USB stack. Data flows only while the host holds DTR (pyserial does on open); otherwise it is discarded. UART fallback: HAL byte-IRQ RX with the 8-byte FIFO at priority 1, DMA TX by register (DMA1_Channel3, no interrupts). |
| `proto.c/h` | RX ring (256 bytes, fed by the link through `proto_rx_bytes()`), the binary frame parser/sender, and the message/status/event enums. Bytes outside frames are dropped. `proto_poll(busy)` is re-entrant: blocking commands call it from `chk()` with `busy = 1`. |
| `objdict.c/h` | Object table (index:subindex → variable, type, flags, min/max) behind READ/WRITE/LIST, the table hash and the build id. |
| `params.c/h` | `motor_params_t g_params`, stored in flash. Two A/B slots with seq number + CRC32. Versioned (`PARAM_VERSION` 7); a copy with any other version is ignored and the defaults load. |
| `pi_ctrl.h` | Inline PI with conditional-integration anti-windup. `pi_current_step()` shares the voltage circle between d and q, with d taking priority. |
| `cordicMgr.h` | Hardware CORDIC sin/cos (`sincos_q31`, Q32 angle in, floats out). |
| `CANopen/` | CANopen node, outside CubeMX's reach. `CANopenNode/`: the stack, copied in (don't edit; configure it in `port/CO_driver_target.h`, which also sets `#pragma GCC optimize("Os")` for all CANopen code). `port/CO_driver_stm32g4.c`: FDCAN driver. `od/`: exported OD. `app/co_app.c`: communication resets, the 1 kHz TIM7 tick, 0x1010/0x1011, the 0x4000–0x40FF bridge to `objdict.c`. `app/co_store.c`: the record in pages 60–61. `app/co402.c`: CiA 402 FSA, modes (PP, PV, HM, CSP, CSV, CST), faults → 0x603F/EMCY. |

### Execution model
- **ISR context (25 kHz):** TIM1 is center-aligned with ARR 3399. CC4 (`setSamplePoint`) triggers the injected ADC conversions; `cs_adc_irq()` calls `currentLoop()`. The outer speed/position loops run every `OUTER_DIV` = 10 cycles (2.5 kHz). Every 10 cycles the ISR fills a telemetry frame and `link_defer()`s it; the link IRQ (priority 1) adds the sum and queues it (`g_tlm_drop`, 0x4002:10, counts frames dropped because the queue was full or the previous frame was still waiting).
- **Main loop:** `cmd_poll()`, then `checkAndClearBridgeFault()`. Calibration routines block in main with `HAL_Delay` and drive the ISR through volatile globals. While they block, `chk()` runs `proto_poll(1)`: READ is still served, ABORT/STOP abort, everything else gets BUSY.
- **Encoder timing:** the TIM1 update interrupt at the counter bottom starts the read. 6 groups take 18.1 µs and end about 1 µs before the ADC samples (at 19.6 µs), so the last group is ≈4 µs old at the current sample; the exact age comes from the TIM1 count recorded at the trigger. `currentLoop()` must finish before the next bottom (≈19 µs after the ADC), or the read starts late. Diagnostics 0x4002 show the loop time and the encoder late/stall/CRC counters; if `late` grows, lower `MT6835_GROUPS`. `currentLoop()` advances the angle with the PLL speed for Park and further, by `OUT_DELAY_S`, for the inverse Park (`ENC_EXTRA_S`, `ANGLE_COMP` in `motorCtrl.c`). The PLL compares at the true sample time.
- **Encoder loss:** `MT6835_Sample()` never takes an all-0x00 / all-0xFF group (MISO stuck; zeros even pass the CRC), and the CRC check only switches itself off if no group has passed since `MT6835_StartBurst()`. `currentLoop()` counts cycles without a fresh angle (`g_enc_lost_n`); in the encoder modes `ENC_LOST_N` (10 ms) stops the motor with `ST_ENC_LOST` and the bridge off (EMCY 0x7305), and `rearm_safe()` then waits `SL_REARM_MS`. A sensor that lost its supply wakes up out of burst mode: `enc_recover()` in `calib.c` resends the burst command every 100 ms while the motor is off.
- **Flux observer** (`obs_update()`/`obs_weight()` in `motorCtrl.c`): runs every cycle in αβ, using R and L from params and λ derived from `kv`. Above `obs_lo_v` of back-EMF its angle is blended into the commutation angle, reaching full weight at `obs_hi_v`, but only with `obs_blend` = 1 and after it has passed its flux check and agreed with the encoder for `OBS_LOCK_N` cycles. Once locked, only a sustained failure (leaky counters `OBS_FLUX_DROP_N`/`OBS_ENC_DROP_N`) or MOE going off drops it. A second PLL (`obs_pll()`) on the observer angle gives an encoder-independent speed, blended into `s_vel` (used by the speed loop, `g_vel` and the output advance) with the same weight. Position always comes from the encoder. The speed/position loops slew their reference at `acc_lim`, which keeps the observer's load-dependent flux error small. `atan2_q32()` switches the CORDIC to phase mode and back, so call it only from the ISR.
- **Interrupt priorities:** the control IRQs (ADC, TIM1 update, TIM1 break) are at **priority 0**, so they cannot preempt each other. The exceptions:
  - The link IRQ at priority 1: `USB_LP_IRQn`, or `USART1_IRQn` (set in `USER CODE USART1_MspInit 1`; the .ioc still says 0). The control loop preempts it, so the data it shares (the RX ring, the TX ring tail) is single-producer/single-consumer, and the TX ring head / UART TX buffer is written only with priority ≥ 1 masked (main, and the deferred telemetry inside that IRQ).
  - `FDCAN1_IT0_IRQn` and `TIM7_IRQn` at priority 2 (`CO_IRQ_PRIO`). The TIM7 1 kHz tick runs `CO_process()`, SYNC, RPDO, `co402_tick()` and TPDO, so heartbeat and SDO keep going while main blocks. CANopenNode's locks mask priority ≥ 2 with BASEPRI, never the control ISR. SysTick (priority 15) doesn't run inside the tick, so `HAL_GetTick()` stands still there. The tick only writes references (`g_vel_ref`, `g_iq_ref`, the position handshake) while main has armed it; FSA transitions and communication resets run in main.
  - `hMT6835.turns`/`angle` are only written inside `currentLoop()`, so the ISR reads them directly; from thread context, use `MT6835_Position()`.
- `CTRL_FS_HZ` in `calib.h` must match the real ISR rate. PI gains, the PLL and the inductance decay timing depend on it.

### Control modes
- `g_mode`: `CTRL_OFF` (zero volts, bridge still switching, integrators reset), `CTRL_CAL` (open-loop `g_vd_cmd`/`g_vq_cmd` at the forced angle `g_theta_cmd`), `CTRL_RUN` (closed-loop current control).
- In `CTRL_RUN`, `g_loop` picks what drives `g_iq_ref`: `LOOP_CURRENT` (the command), `LOOP_VEL` (speed PI), `LOOP_POS` (position P → speed PI). Switching is bumpless.
- **Trips** (`trip_check()` in `motorCtrl.c`, `g_trip` in `calib.h`): overvoltage (`g_vin` > `vbus_max` for 1 ms; 0 = off) and overcurrent (|id| or |iq| > `max_current` for 3 cycles).
  - In `CTRL_RUN` the ISR coasts at zero current: id = iq = 0 refs, outer loops held. No torque, no regen into the bus, no short-brake, and the currents stay measured.
  - Outside `CTRL_RUN`, or if the current goes on rising in the coast (> 1.5 × imax, or still > imax after 5 ms), it calls `bridge_trip_off()`: MOE = 0 and `g_trip_moe`. `checkAndClearBridgeFault()` does not re-arm MOE while `g_trip_moe` is set.
  - The ISR never writes `g_mode`. `trip_watchdog()` in `calib.c` sends `EV_STOPPED` (`ST_VBUS_HIGH`/`ST_OVERCURRENT`) and sets `CTRL_OFF` once |speed| < `COAST_END_TURNS` (30 rpm).
  - An overvoltage coast (CTRL_RUN, bridge still on) resumes by itself once the bus has been below `vbus_max − vbus_resume` for 10 ms: the ISR restores the saved id ref (and the iq ref in current mode), the outer loops pick up bumplessly, `EV_RESUMED` is sent and 0x4002:11 counts it. Overcurrent never resumes.
  - Regen clamp: in `CTRL_RUN`, braking current (iq against `s_vel`) is scaled from full at `vbus_max − vbus_clamp` down to zero at `vbus_max`; motoring current is never limited. While it acts, the speed ramp holds its deceleration and the speed integrator is capped at the allowed braking current.
  - While a trip is latched every command except STOP/REBOOT gets `ST_TRIPPED`. STOP resets it once the coast has ended.
  - With MOE off, the body diodes only conduct if something external drives the shaft above the bus voltage (there is no field weakening).
- **Main → ISR handover rules:**
  - Write the reference first, then switch the mode/loop.
  - The 64-bit position ref goes through `g_pos_ref_new` + the `g_pos_ref_req` handshake (64-bit writes are not atomic on the M4). The same handshake carries the CSP/PP interpolation: `g_pos_ff_new` (turn/s feed-forward), `g_pos_ff_q_new` (Q32.32 step added each cycle) and `g_pos_ff_n_new` (for that many cycles); `run_pos` sends n = 0. `g_pos_here_req` makes the reference the present position (bumpless switch into `LOOP_POS`).
  - Homing uses `g_home_req`; the present position becomes `g_home_pos` (0 from USB HOME, −0x607C from CANopen homing).
  - `g_pos_cnt` (int32 encoder counts from home) is the atomic copy of the position for thread and tick code. `g_ramp_bypass` = 1 (CANopen modes) skips the `acc_lim` slew.
- CANopen owns the motor from enable operation until it is off again (`g_can_owner`): USB RUN_*/CAL/HOME/SAVE/REBOOT then get `ST_CAN_ACTIVE`. USB STOP always works; the CANopen side then faults with 0xFF01.

### Units / fixed-point conventions
- Angles are Q32: `2^32` = 360° (electrical angle, `elec_offset`, `g_theta_cmd`).
- The encoder gives 21 bits per turn: `angle << 11` is a mechanical Q32 angle.
- Position is `pos_q32_t`, an int64 Q32.32 in turns. `g_pos` (float) is for display only.
- Speed is in turn/s internally; commands and telemetry use rpm.
- `params` are in SI units (Ω, H, A, V). `ang_src` (`angsrc`, 0x4011) picks the angle source: 0 encoder only, 1 encoder with the observer blended in (`oblend`), 2 sensorless (HFI, then the observer above `olo`; no encoder in the loop, position mode refused, the encoder is only logged). `hfi_mode` is 0/1 (track and log in the encoder modes); sensorless always injects.
- `ki` is stored as continuous-time; `pi_apply()` turns it into `ki_ts`. Call `pi_apply()` after any gain change.

### Serial protocol (USB CDC, or USART1)
There is no text console: everything goes through binary frames, and the plotter is the user interface.
- **MCU → PC telemetry:** a 72-byte `Frame`: `A5 5A`, `seq u16`, 16 floats, then a `sum u32` computed with rotate-left-1-and-add from seed `0x5A5AA5A5`. `v[2..4]` carry HFI − encoder (deg elec), HFI rpm and the HF current step (NaN with HFI off); `v[11..15]` the observer − encoder error (deg elec), |flux|/λ, the blend weight, observer rpm and encoder rpm. The full list is in `PLOTS` in `software/plotter.py`.
- **Config protocol (both directions):** `A5 5C | type | tok | len | payload | crc16` (CCITT-FALSE, LE), defined in `proto.h`; rationale in `docs/design-notes.md`. Bytes outside frames are ignored.
  - PING/READ/WRITE/LIST go to the object table (`objdict.c`, native objects 0x4000–0x4042; the same objects answer over SDO), types up to `T_I16` = 8, errors as CANopen SDO abort codes. A WRITE takes effect in RAM; SAVE writes flash.
  - SAVE, CAL, RUN_CUR/VEL/POS, STOP, HOME, ABORT and REBOOT reply with an `st_t` status. REBOOT (mode 0 reset, 1 DFU) is refused unless `g_mode` is `CTRL_OFF`; after the reply the device drops off the bus. CAL replies with an ACK first, then sends `EV_*` events (`EV(code, floats…)` in `calib.c`, tagged with the request's `s_tok`) and ends with `EV_DONE`. The watchdog sends `EV_STOPPED` with tok 0.
  - Calibrations (`CAL_*`, described in the driver manual §5): prerequisites are enforced through the `PF_*` flags in `g_params.flags` (usual order: current offsets → encoder → R → Ld → autotune → kv). For Ld/Lq/HFI map the argument is a test current/bias: 0 stores the result, any other value measures that point only. Speed and position mode also need `vkp` and `vlim` (and `pkp`) set.
  - Replies are sent by `proto_send()` from main. On the UART a reply that collides with a telemetry frame costs that frame (a `seq` gap).
  - Message types must stay below `MSG_EVENT` (0x7F): a reply is `type | 0x80` and must never equal the event type.
- **PC side:** `software/config_tab.py` (Config tab, sidebar run panel and status boxes), `software/protocol.py` (framing, `Link`, status/event text), `software/objects.py` (labels, units, limits).
- **Keep in sync** (`python software/check_sync.py` checks the tables and enums):
  - Frame layout, `frame_sum()`, or what `telemetry.v[i]` holds ↔ `PLOTS` in `software/plotter.py` and `telem_ok()`/`Splitter` in `software/protocol.py`.
  - `k_tab` in `objdict.c` ↔ `OBJECTS` in `software/objects.py`: same order, index, sub, type and flags (both sides hash them; the GUI warns and disables mismatched rows).
  - Message types, `CAL_*`, `st_t`, `EV_*` and `SDO_*` in `proto.h` ↔ `software/protocol.py` (with the English text and event formatters).
  - Native objects 0x4000–0x40FF: `k_tab` ↔ `OBJECTS` ↔ `driverTCC.xdd` (then the user exports `OD.c`/`OD.h`/`driverTCC.eds`). `check_sync.py` compares the EDS with `objects.py` and notes when the XDD is newer than the exports.
  - The CANopen tab and `co_link.py` read everything about CAN objects from `software/driverTCC.eds`. The `st_t` codes the tests use (`ST_NEED_VGAINS`, `ST_RUNNING`, `ST_CAN_*`, `ST_FOLLOWING_ERROR`) are named in `protocol.py` and checked.

### Parameters in flash
- `params_save()` refuses to run while MOE is set (the page erase stalls the CPU ~20 ms). SAVE is refused unless the motor is off; it disables the bridge, saves, and re-inits it. SAVE writes both records: the motor params (pages 62–63) and the CANopen record (pages 60–61: node ID, bit rate, the `PERSIST_COMM`/`PERSIST_APP` OD parts). CANopen 0x1010 store requests are deferred to the main loop the same way (`:2` CANopen record, `:3` 402 settings + motor params).
- The CANopen record carries a signature of the OD layout: after an export that changes a persisted part, that part's stored copy is ignored and the exported defaults load (node ID and bit rate are kept). `PARAM_VERSION` is independent of it.
- When changing `motor_params_t`: keep `crc` last and the size a multiple of 8, give new fields a default in `k_defaults`, and bump `PARAM_VERSION`. There is no migration: after a layout change the stored copy is ignored, the defaults load, and everything must be recalibrated (or restored with the Config tab's JSON import) and saved. Changing only a default value needs no bump.

## Gotchas
- Keep the FDCAN1 interrupts and TIM7 **out of the .ioc**. `CANopen/port/CO_driver_stm32g4.c` owns `FDCAN1_IT0_IRQHandler` and the HAL FDCAN callbacks; `co_app.c` sets TIM7 up by register and owns `TIM7_IRQHandler`. If CubeMX generates either handler, the link fails.
- PB8 is BOOT0 and FDCAN1_RX, and the transceiver idles it high: each chip needs the option bytes nSWBOOT0 = 0, nBOOT0 = 1 once, or it boots the ST bootloader (commands in `docs/user-manual-driver.md` §4.1).
- Keep the TIM1 update interrupt (`TIM1_UP_TIM16_IRQn`) **disabled in the .ioc**. `mt6835.c` enables it at runtime and owns `TIM1_UP_TIM16_IRQHandler`; a second, CubeMX-generated handler breaks the link. The SPI1 DMA channels are reprogrammed by register on every read; leave them in NORMAL mode in the .ioc.
- Current and voltage scaling come from the hard-coded constants in `curSnsCtrl.h` (`CS_SHUNT`, `CS_GAIN`, `VIN_RL/RH`). The stored `temp_max` is **not used** yet.
- Current offsets: `cur_off[i]` = ADC zero − 2048 in counts. `cs_boot_offsets()` (end of `initMotor()`) measures them at every boot over 64 ms and keeps the result only if the spread is small and it lies within 25 counts of the fallback (`g_params.cur_off` once `PF_CUR_CAL_VALID` is set, else `CS_OFF_DEFAULT`), so a rotor turning at boot falls back. The offsets in use and their source are objects 0x4023:4–7.
- `SVPWM()` writes nothing for `FAULT_CLEAR_MIN_TIME` ms after a fault is cleared.
- **No zero vector above `COAST_END_TURNS` (30 rpm).** `CTRL_OFF` applies the zero vector, and so does re-arming MOE; on a spinning motor that is a short-brake of about λ/L, far above `imax`. Therefore:
  - Stops go through `coast_or_off()` in `calib.c`. Above that speed it stays in `CTRL_RUN` at id = iq = 0 (`g_coasting`, 0x4001:15), and `run_watchdog()` switches off once the motor is slow. STOP replies at once, and a run command takes over a coasting motor.
  - `run_stop()` still brakes under the speed loop first in speed/position mode.
  - A lost HFI or encoder angle switches the bridge off instead.
  - `checkAndClearBridgeFault()` re-arms only below that speed (encoder), or after `SL_REARM_MS` (sensorless, which cannot see the speed with the bridge off), returning `BRIDGE_ERR_SPIN` until then. Before MOE has been on once since boot, it re-arms at once.
  - Keep `kv_coast()` and this rule.
- **No text in the firmware.** Values go out as raw floats in events/replies, and the PC formats them. Don't call `printf`/`snprintf`/`sprintf`, `strtof`/`strtod`/`atof`, `strtok` (via `assert`) or `assert`: each links back several KB of stdio, bignum or float-printing code (together ~15–19 KB). Check with `arm-none-eabi-nm` for `_vfprintf_r`, `_printf_float`, `_strtod_r`. Avoid `double` too: float varargs are promoted to `double`, which is why `EV()` builds a `const float[]`. Check with `arm-none-eabi-objdump -d` for calls to `__aeabi_f2d`. `-fno-math-errno` makes `sqrtf` compile to a single `vsqrt` instruction.
- `motorCtrl.c`, `curSnsCtrl.c`, `hfi.c` and the burst/DMA half of `mt6835.c` carry `#pragma GCC optimize("O2")`, so the ISR path is `-O2` in both the `-Og` debug build and the `-Os` release build. In the debug build, locals in `currentLoop()` may show as optimized out. Put new ISR-path code in those files (or under the same pragma).
