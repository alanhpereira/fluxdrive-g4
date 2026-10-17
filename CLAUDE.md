# CLAUDE.md

This file guides Claude Code (claude.ai/code) when it works in this repository.

## What this is

This is the firmware for a graduation-thesis BLDC/PMSM motor driver. It runs field-oriented control (FOC) on an STM32G431VB (STSPIN32G4-class part, Cortex-M4F, 170 MHz, 128 KB flash, 32 KB RAM). The hardware it drives:
- A 3-phase bridge on TIM1, plus a gate driver on I2C3 at address `0x47`. The driver's nFAULT pin is PE15, which also feeds the TIM1 break input.
- Three low-side shunts (2 mΩ) read through the internal OPAMPs (gain 16). ADC1 and ADC2 sample them as injected conversions, triggered by TIM1 CC4.
- An MT6835 21-bit magnetic encoder on SPI1 (10.6 MHz), with nCS on PA15. It stays in burst mode (CS held low), and a TIM1-triggered DMA read of a few angle groups runs once per PWM period.
- USB CDC (full-speed virtual COM port, ST VID/PID 0483:5740). It carries the binary telemetry and the binary config protocol (no text console). USART1 at 2 Mbaud is the build-time fallback: `PROTO_LINK_USB 0` in `link.h`.

`software/plotter.py` is the PC-side GUI (PyQt + pyqtgraph). It shows the telemetry and sends commands.

## Build / flash / debug

The project is driven by the **STM32 for VSCode** extension. `STM32Make.make` is generated from `STM32-for-VSCode.config.yaml` plus the CubeMX `Makefile`. Toolchain paths are in `.stm32env`: arm-none-eabi-gcc 13.2 and OpenOCD 0.12 under `C:/DEV/TOOLS/...`.

- Build: VS Code task "Build STM", or `make -f STM32Make.make -j`. Output goes to `build/debug/Driver_TCC.elf`. That debug build is always `-Og`: `OPTIMIZATION` is only used with `DEBUG=0` (e.g. `make -f STM32Make.make -j DEBUG=0 OPTIMIZATION=-Os`), which builds into `build/release/`; `make ... DEBUG=0 ... flash` flashes that one. Tasks "Build STM release" / "Flash STM release" do this with `-Os -g`, and the launch config "Attach STM32 (release)" goes with them. `make` only rebuilds objects when sources or `STM32Make.make` change, not when `OPTIMIZATION` changes: delete `build/release/` when you switch flags. Flash budget and trimming plan: `docs/flash-optimization-handoff.md`, measured with `python software/flash_report.py`. Angle sources and sensorless (`angsrc`: 0 encoder, 1 encoder + observer, 2 sensorless; sensorless implemented, not yet run on hardware): `docs/hfi-handoff.md`. Binary config protocol + plotter Config tab (phases 1–4 done; USB transport not yet run on hardware): `docs/config-gui-handoff.md`. CANopen (generic CiA 301/402 drive on FDCAN1, all six modes, run on hardware): `docs/canopen-handoff.md`.
- Flash: task "Flash STM", or `make -f STM32Make.make flash`. This uses ST-Link through OpenOCD (`openocd.cfg`).
- Erase: `make -f STM32Make.make erase`. This also wipes the stored motor parameters.
- Debug: the cortex-debug launch configs "Debug STM32" and "Attach STM32" in `.vscode/launch.json`, with live watch enabled.
- Plotter: `pip install -r software/requirements.txt`, then `python software/plotter.py [COMx]`. A left sidebar shared by both tabs holds Connection, the Run panel (setpoints, Home, STOP; Enter sends), Status / faults and Statistics (loop time, encoder and link counters; polled at 4 Hz on either tab); Display and Plots show there only with the **Scope** tab. Tabs: **Scope** (telemetry plots) and **Config** (parameters, calibration, log). Numeric inputs use the C locale (`.` decimals) on purpose: with a `,` locale, typed `0.75` was read as 7. `python software/check_sync.py` checks the PC tables against the firmware sources.
- New hand-written `Core/Src/*.c` files must be in `C_SOURCES` in `STM32Make.make`. The extension adds them when it regenerates the file; for command-line builds, add them by hand. Sources and include paths under `CANopen/` are listed in `STM32-for-VSCode.config.yaml` too (`sourceFiles`, `includeDirectories`), so a regenerate keeps them.
- CANopen object dictionary: `driverTCC.xdd` (repo root) is the CANopenEditor project. `CANopen/od/OD.c`/`OD.h` and `software/driverTCC.eds` are **exported** from it by the user in CANopenEditor; never write or hand-edit them. After an XDD change, stop and let the user export, then build.

There are no unit tests. Changes are verified on hardware through the plotter (Config tab and telemetry).

## CubeMX rules (important)

`Driver TCC.ioc` is the source of truth for the peripheral setup. Everything below is CubeMX-generated, and CubeMX overwrites it when the project is regenerated:
- `Core/Src/{adc,dac,dma,fdcan,gpio,i2c,opamp,spi,tim,usart,cordic}.c`
- `stm32g4xx_hal_msp.c`, `stm32g4xx_it.c`
- `USB_Device/`, `Drivers/`, `Middlewares/`, `Makefile`, the linker script, and the startup file

Code you add to those files **must** sit between `/* USER CODE BEGIN x */` and `/* USER CODE END x */` markers, or CubeMX will delete it. `main.c` follows the same rule. All the hand-written logic is in its own files, listed below. USB CDC carries the protocol; its hooks are in USER CODE sections of `usbd_cdc_if.c` (receive, control/DTR, TX complete), `stm32g4xx_it.c` (`USB_LP_IRQn 0`: `link_usb_pump()`) and `usbd_conf.c` (`USB_MspInit 1`: USB IRQ priority 1 and CRS trimming HSI48 to the SOF). The .ioc sets USB_LP to priority 1 and `APP_RX/TX_DATA_SIZE` to 64. CANopen over FDCAN1 carries normal operation: the .ioc has FDCAN1 at 1 Mbit/s (prescaler 10, Seg1 13, Seg2 3, SJW 3) with AutoRetransmission on; the port re-times it at runtime for other bit rates. The FDCAN1 and TIM7 interrupts are **not** in the .ioc (see Gotchas).

The flash length in the linker script is set to 120K on purpose. Pages 60 and 61 hold the CANopen record (`co_store.c`), and the last two 2 KB pages (62 and 63) hold the parameters. If CubeMX regenerates `STM32G431XX_FLASH.ld`, set FLASH back to 120K (never above 124K).

The 25 kHz ISR path runs from CCM SRAM. The linker script splits RAM into `RAM` (22K at 0x20000000) and `CCMRAM` (10K at 0x10000000). A `.ccmram` output section placed **before** `.text` (the first matching rule wins) collects `currentLoop`, `SVPWM`, `hfi_*`, the ADC IRQ/callback chain, `MT6835_Sample`, `TIM1_UP_TIM16_IRQHandler` and the HAL calls they make every cycle. `main()` copies it from flash in `USER CODE BEGIN 1`, before `HAL_Init`. Flash at 4 wait states with a 1 KB cache was too slow: max 23.9 µs with HFI. A CubeMX regenerate drops all of this, so put it back. New per-cycle functions belong in that list; `--print-memory-usage` shows the CCMRAM fill (≈9 of 10 KB).

Formatting is set by `.clang-format`: Google base style, **tabs**, width 4, no column limit, and `else` on a new line after `}`.

## Architecture

### Hand-written modules (`Core/Src` + `Core/Inc`)
| File | Role |
|---|---|
| `motorCtrl.c/h` | `currentLoop()`, the FOC ISR body: Clarke/Park, current PI, SVPWM, PLL position/speed estimate, outer speed/position loops, step recorder, telemetry frames. `initMotor()` brings everything up. |
| `curSnsCtrl.c/h` | ADC/OPAMP/DAC setup. `HAL_ADCEx_InjectedConvCpltCallback` waits for both ADC1 and ADC2 to finish, scales the currents and the bus voltage, then **calls `currentLoop()`**. |
| `bridgeCtrl.c/h` | TIM1 PWM start/stop, `setDuty()`/`setSamplePoint()` (CCR1-4), and the DAC2 short-circuit reference. `checkAndClearBridgeFault()` reads the gate-driver fault register over I2C and re-arms MOE after a break. |
| `mt6835.c/h` | Encoder driver: register R/W, zero, EEPROM. Burst mode: `MT6835_StartBurst()` sends one burst command and leaves CS low. The sensor then shifts out 4-byte groups (angle H/M/L+status, CRC) whenever the SPI clocks. `TIM1_UP_TIM16_IRQHandler` (defined here) starts a register-level DMA read of `MT6835_GROUPS` groups at the TIM1 counter bottom, sized to end just before the ADC sample. `MT6835_Sample()`, called first in `currentLoop()`, takes the last complete group, checks its CRC, updates `hMT6835.angle` (21-bit) and the multi-turn `turns` counter, and returns the sample age. |
| `calib.c/h` | Command entry (`cmd_message()`: SAVE, CAL, RUN_*, STOP, HOME), calibration routines, run modes, safety watchdog. `cmd_poll()` = `run_watchdog()` + `proto_poll(0)` + `co_app_poll()` + `co402_poll()` + deferred 0x1010 saves. Owns the shared ISR↔main globals (`g_mode`, `g_loop`, refs, PI states), and the `drive_*()` API the CiA 402 layer uses (`drive_check/enter/coast/trip_reset`, `g_can_owner`, `g_stop_reason`). |
| `boot.c/h` | `MSG_REBOOT`: `boot_reboot()` drops USB (so the host sees the port go) and resets; for DFU it first leaves a magic in TAMP backup register 0. `boot_check()`, the first call in `main()`, sees the magic, clears it and jumps to the ST system bootloader at 0x1FFF0000 (USB DFU, flash with STM32CubeProgrammer). |
| `link.c/h` | The byte pipe. USB (default): a 2 KB TX ring filled by the ISR (telemetry) and main (replies); producers pend `USB_LP_IRQn`, and only the USB IRQ calls into the USB stack. Data flows only while the host holds DTR (pyserial does on open); otherwise it is discarded. UART fallback: byte-IRQ RX with the 8-byte FIFO, DMA TX. |
| `proto.c/h` | RX ring (256 bytes, fed by the link through `proto_rx_bytes()`), the binary frame parser/sender, and the message/status/event enums. Bytes outside frames are dropped. `proto_poll(busy)` is re-entrant: blocking commands call it from `chk()` with `busy = 1`. |
| `objdict.c/h` | Object table (index:subindex → variable, type, flags, min/max) behind READ/WRITE/LIST, the table hash and the build id. |
| `params.c/h` | `motor_params_t g_params`, stored in flash. Two A/B slots with seq number + CRC32. Versioned (v7); a copy with any other version is ignored and the defaults load. |
| `pi_ctrl.h` | Inline PI with conditional-integration anti-windup. `pi_current_step()` shares the voltage circle between d and q, with d taking priority. |
| `cordicMgr.h` | Hardware CORDIC sin/cos (`sincos_q31`, Q32 angle in, floats out). |
| `CANopen/` | CANopen node, outside CubeMX's reach (`docs/canopen-handoff.md`). `CANopenNode/`: the stack, copied in (don't edit; configure it in `port/CO_driver_target.h`, which also sets `#pragma GCC optimize("Os")` for all CANopen code). `port/CO_driver_stm32g4.c`: FDCAN driver. `od/`: exported OD. `app/co_app.c`: communication resets, the 1 kHz TIM7 tick, 0x1010/0x1011, the 0x4000–0x40FF bridge to `objdict.c`. `app/co_store.c`: the record in pages 60–61. `app/co402.c`: CiA 402 FSA, modes (PP, PV, HM, CSP, CSV, CST), faults → 0x603F/EMCY. |
| `hfi.c/h` | High-frequency injection: square wave on the estimated d axis (half wave `hfi_half` ≥ 2 periods), demodulation from consecutive samples, angle PLL, polarity pulses, and the PI current filter. `hfi_step()` is called from `currentLoop()` in `CTRL_RUN`; the HFI calibrations in `calib.c` drive it through `g_hfi_cmd`. |

### Execution model
- **ISR context (25 kHz):** TIM1 is center-aligned with ARR 3399, giving 25 kHz PWM. CC4 (`setSamplePoint`) triggers the injected ADC conversions. The ADC callback calls `currentLoop()`. The outer speed/position loops run every `OUTER_DIV` = 10 cycles, at 2.5 kHz. A telemetry frame is queued on the link every 10 cycles (`g_tlm_drop`, object 0x4002:10, counts frames that found the USB queue full).
- **Main loop:** `cmd_poll()` (command parsing plus `run_watchdog()`), then `checkAndClearBridgeFault()`. Calibration routines block in main with `HAL_Delay` and drive the ISR through volatile globals. While they block, `chk()` runs `proto_poll(1)`: READ is still served, ABORT/STOP abort, everything else gets BUSY.
- Encoder timing: the TIM1 update interrupt at the counter bottom starts the read. 6 groups take 18.1 µs and end about 1 µs before the ADC samples (at 19.6 µs), so the last group is ≈4 µs old at the current sample. The exact age comes from the TIM1 count recorded at the trigger. `currentLoop()` must finish before the next bottom (≈19 µs after the ADC), or the read starts late. The Config tab's diagnostics (0x4002) show the loop time and the encoder late/stall/CRC counters; if `late` grows, lower `MT6835_GROUPS`. `currentLoop()` advances the angle with the PLL speed for the Park transform and further, by `OUT_DELAY_S`, for the inverse Park (`ENC_EXTRA_S`, `ANGLE_COMP` in `motorCtrl.c`). The PLL compares at the true sample time.
- Flux observer (`obs_update()`/`obs_weight()` in `motorCtrl.c`): a nonlinear flux observer runs every cycle in the αβ frame, using R and L from params and λ derived from `kv`. Above `obs_lo_v` of back-EMF its angle is blended into the commutation angle, reaching full weight at `obs_hi_v`, but only when `obs_blend` is 1 and the observer has passed its flux check and agreed with the encoder for `OBS_LOCK_N` cycles. Once it is locked, only a sustained failure drops it (leaky counters `OBS_FLUX_DROP_N`/`OBS_ENC_DROP_N`), or MOE going off. The speed/position loops slew their speed reference at `acc_lim` (rpm/s), which keeps transient current, and with it the observer's load-dependent flux error, small. A second PLL (`obs_pll()`) on the observer angle gives an encoder-independent speed, blended into `s_vel` (what the speed loop, `g_vel` and the output advance use) with the same weight. Position always comes from the encoder. Telemetry `v[2..4]` carries HFI − encoder (deg elec), HFI rpm and the HF current step (NaN with HFI off; these used to be the phase currents). `v[11..15]` carries the observer−encoder error (deg elec), |flux|/λ, the blend weight, observer rpm and encoder rpm. `atan2_q32()` switches the CORDIC to phase mode and back, so call it only from the ISR.
- All IRQs (ADC, TIM1 update, UART, TIM1 break) are at **priority 0**, so they cannot preempt each other. The exceptions:
  - `USB_LP_IRQn` at priority 1: the control loop preempts it, so the data it shares (the RX ring, the TX ring tail) is single-producer/single-consumer, and the TX ring head is written only at priority 0 or with IRQs off.
  - `FDCAN1_IT0_IRQn` and `TIM7_IRQn` at priority 2 (`CO_IRQ_PRIO`). The TIM7 1 kHz tick runs `CO_process()`, SYNC, RPDO, `co402_tick()` and TPDO, so heartbeat and SDO keep going while main blocks. CANopenNode's locks mask priority ≥ 2 with BASEPRI, never the control ISR. SysTick (priority 15) doesn't run inside the tick, so `HAL_GetTick()` stands still there. The tick only writes references (`g_vel_ref`, `g_iq_ref`, the position handshake) while main has armed it; FSA transitions and communication resets run in main. `hMT6835.turns`/`angle` are only written inside `currentLoop()`, so it reads them directly. From thread context, use `MT6835_Position()` instead.
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
- Trips (`trip_check()` in `motorCtrl.c`, `g_trip` in `calib.h`): overvoltage (`g_vin` > `vbus_max` for 1 ms; default 60 V, 0 = off) and overcurrent (|id| or |iq| > `max_current` for 3 cycles).
  - In `CTRL_RUN` the ISR coasts at zero current: id = iq = 0 refs, outer loops held. That means no torque, no regen into the bus, and no short-brake, and the currents stay measured.
  - Outside `CTRL_RUN`, or if the current goes on rising in the coast (> 1.5 × imax, or still > imax after 5 ms), it calls `bridge_trip_off()`: MOE = 0 and `g_trip_moe`. `checkAndClearBridgeFault()` does not re-arm MOE while `g_trip_moe` is set.
  - The ISR never writes `g_mode`. `trip_watchdog()` in `calib.c` sends `EV_STOPPED` (`ST_VBUS_HIGH`/`ST_OVERCURRENT`) and sets `CTRL_OFF` once |speed| < `COAST_END_TURNS` (30 rpm).
  - An overvoltage coast (CTRL_RUN, bridge still on) resumes by itself once the bus has been below `vbus_max − vbus_resume` (param `vresume`, default 2 V) for 10 ms.
    - The ISR restores the saved id ref, and the iq ref too in current mode. The speed/position loops pick up bumplessly from the present speed.
    - `EV_RESUMED` is sent, and 0x4002:11 counts the resumes. Overcurrent never resumes.
  - Regen clamp (before the OV trip): in `CTRL_RUN`, braking current (iq against `s_vel`) is scaled from full at `vbus_max − vbus_clamp` (param `vclamp`, default 3 V, 0 = off) down to zero at `vbus_max`. Motoring current is never limited.
    - While the clamp acts, the speed ramp holds its deceleration and the speed integrator is capped at the allowed braking current, so it can't wind up.
    - A supply that sits inside that band also reduces braking.
  - While a trip is latched every command except STOP/REBOOT gets `ST_TRIPPED`. STOP resets it once the coast has ended.
  - With MOE off, the body diodes only conduct if something external drives the shaft above the bus voltage (there is no field weakening).
- Main hands values to the ISR through these rules:
  - Write the reference first, then switch the mode/loop.
  - The 64-bit position ref goes through `g_pos_ref_new` + the `g_pos_ref_req` handshake, because 64-bit writes are not atomic on the M4. The same handshake carries the CSP/PP interpolation: `g_pos_ff_new` (turn/s feed-forward), `g_pos_ff_q_new` (Q32.32 step added each cycle) and `g_pos_ff_n_new` (for that many cycles); `run_pos` sends n = 0. `g_pos_here_req` makes the reference the present position (bumpless switch into `LOOP_POS`).
  - Homing uses `g_home_req`; the present position becomes `g_home_pos` (0 from USB HOME, −0x607C from CANopen homing).
  - `g_pos_cnt` (int32 encoder counts from home) is the atomic copy of the position for thread and tick code. `g_ramp_bypass` = 1 (CANopen modes) skips the `acc_lim` slew.
- CANopen owns the motor from enable operation until it is off again (`g_can_owner`): USB RUN_*/CAL/HOME/SAVE/REBOOT then get `ST_CAN_ACTIVE`. USB STOP always works; the CANopen side then faults with 0xFF01.

### Units / fixed-point conventions
- Angles are Q32: `2^32` = 360°. That covers the electrical angle, `elec_offset`, and `g_theta_cmd`.
- The encoder gives 21 bits per turn: `angle << 11` is a mechanical Q32 angle.
- Position is `pos_q32_t`, an int64 Q32.32 in turns. `g_pos` (float) is for display only.
- Speed is in turn/s internally. Commands and telemetry use rpm.
- `params` are in SI units (Ω, H, A, V). The observer/ramp settings (`oblend`, `olo`, `ohi`, `ogain`, `oftol`, `oerr`, `acc`; fields `obs_*`, `acc_lim`, added in v4) and the HFI settings (`hmode`, `hhalf`, `hv`, `hbw`, `hid`, `hsal`, `hpi`; fields `hfi_*`, added in v5) are params too. `angsrc` (field `ang_src`, object 0x4011, added in v6) and the regen clamp band / OV resume margin (`vclamp`, `vresume`; fields `vbus_clamp`, `vbus_resume`, 0x4022:4–5, added in v7) picks the angle source: 0 encoder only, 1 encoder with the observer blended in (`oblend`), 2 sensorless (HFI, then the observer above `olo`; no encoder in the loop, position mode refused, the encoder is only logged). `hmode` is 0/1 only (track and log in the encoder modes); sensorless always injects. `ki` is stored as continuous-time; `pi_apply()` turns it into `ki_ts`. Call `pi_apply()` after any gain change.

### Serial protocol (USB CDC, or USART1)
There is no text console: everything goes through binary frames, and the plotter is the user interface.
- **MCU → PC telemetry:** a 72-byte binary `Frame`: `A5 5A`, `seq u16`, 16 floats, then a `sum u32` computed with rotate-left-1-and-add from seed `0x5A5AA5A5`.
- **Config protocol (both directions):** `A5 5C | type | tok | len | payload | crc16` (CCITT-FALSE, LE), defined in `proto.h`; design in `docs/config-gui-handoff.md`. Bytes outside frames are ignored.
  - PING/READ/WRITE/LIST go to the object table (`objdict.c`, native objects at 0x4000–0x4042 (0x4040 CAN node ID / bit rate / apply, 0x4041 CAN status, 0x4042 CiA 402 state; the same objects answer over SDO), types up to `T_I16` = 8, errors as CANopen SDO abort codes). A WRITE takes effect in RAM; SAVE writes flash.
  - SAVE, CAL, RUN_CUR/VEL/POS, STOP, HOME, ABORT and REBOOT reply with an `st_t` status. REBOOT (mode 0 reset, 1 DFU) is refused while `g_mode` is not `CTRL_OFF`; after the reply the device drops off the bus. CAL replies with an ACK first, then sends `EV_*` events (`EV(code, floats…)` in `calib.c`, tagged with the request's `s_tok`) and ends with `EV_DONE`. The watchdog sends `EV_STOPPED` with tok 0.
  - The calibrations (`CAL_*`): encoder dir/pole pairs/offset (averaged over one turn each way), R, Ld, Lq (hold the shaft), PI autotune from R, L and `bw`, kv (the motor spins), encoder zero reset, the HFI saliency map (argument = d bias, 0 = several biases and store `hsal`), the HFI lock + polarity test, and the current-sense offsets (`CAL_CUR_OFF`, rotor still: stores `cur_off`, sets `PF_CUR_CAL_VALID`). For Ld/Lq, the argument is a test current: 0 stores the result, and any other value measures that point only, without storing.
  - Prerequisites are enforced through the `PF_*` flags in `g_params.flags`. The usual order is encoder → R → Ld → autotune. Speed and position mode also need `vkp`/`vki`/`vlim` (and `pkp`) set.
  - Replies are sent by `proto_send()` from main, which starts the DMA with IRQs off once `gState` is READY. The telemetry ISR makes the same check, so each reply costs one telemetry frame (a `seq` gap).
  - Message types must stay below `MSG_EVENT` (0x7F): a reply is `type | 0x80` and must never equal the event type.
- **Plotter:** the **Config** tab (`software/config_tab.py`) has the parameter grid with JSON export/import, the calibration wizard and the log; it also builds the sidebar's run panel and status/statistics boxes. `software/protocol.py` holds the framing and the `Link`, and `software/objects.py` the labels, units and limits.
- **Keep in sync** (`python software/check_sync.py` checks the tables and enums):
  - Frame layout, `frame_sum()`, or what `telemetry.v[i]` holds ↔ `PLOTS` in `software/plotter.py` and `telem_ok()`/`Splitter` in `software/protocol.py`.
  - `k_tab` in `objdict.c` ↔ `OBJECTS` in `software/objects.py`: same order, index, sub, type and flags (both sides hash them; the GUI warns and disables mismatched rows).
  - Message types, `CAL_*`, `st_t`, `EV_*` and `SDO_*` in `proto.h` ↔ `software/protocol.py` (with the English text and event formatters).
  - Native objects 0x4000–0x40FF: `k_tab` ↔ `OBJECTS` ↔ `driverTCC.xdd` (then the user exports `OD.c`/`OD.h`/`driverTCC.eds`). `check_sync.py` compares the EDS with `objects.py` and notes when the XDD is newer than the exports.

### Parameters in flash
- `params_save()` refuses to run while MOE is set, because the page erase stalls the CPU about 20 ms. SAVE is therefore refused unless the motor is off; it disables the bridge, saves, and re-inits it. SAVE writes both records: the motor params (pages 62–63) and the CANopen record (pages 60–61: node ID, bit rate, the `PERSIST_COMM`/`PERSIST_APP` OD parts). CANopen 0x1010 store requests are deferred to the main loop the same way (`:2` CANopen record, `:3` 402 settings + motor params).
- The CANopen record carries a signature of the OD layout: after an export that changes a persisted part, that part's stored copy is ignored and the exported defaults load (node ID and bit rate are kept). `PARAM_VERSION` is independent of it.
- When changing `motor_params_t`:
  - Keep `crc` last and the size a multiple of 8. Give new fields a default in `k_defaults`.
  - Bump `PARAM_VERSION`. There is no migration: after a layout change the stored copy is ignored, the defaults load, and everything must be recalibrated (or restored with the Config tab's JSON import) and saved.

## Gotchas
- Keep the FDCAN1 interrupts and TIM7 **out of the .ioc**. `CANopen/port/CO_driver_stm32g4.c` owns `FDCAN1_IT0_IRQHandler` and the HAL FDCAN callbacks; `co_app.c` sets TIM7 up by register and owns `TIM7_IRQHandler`. If CubeMX generates either handler, the link fails.
- PB8 is BOOT0 and FDCAN1_RX, and the transceiver idles it high: each chip needs the option bytes nSWBOOT0 = 0, nBOOT0 = 1 once, or it boots the ST bootloader (commands in `docs/canopen-handoff.md`).
- Keep the TIM1 update interrupt (`TIM1_UP_TIM16_IRQn`) **disabled in the .ioc**. `mt6835.c` enables it at runtime and owns `TIM1_UP_TIM16_IRQHandler`. If CubeMX generates a second handler, the link fails. The SPI1 DMA channels are reprogrammed by register on every read; leave them in NORMAL mode in the .ioc.
- Current and voltage scaling come from the hard-coded constants in `curSnsCtrl.h` (`CS_SHUNT`, `CS_GAIN`, `VIN_RL/RH`). `cur_scale` and `vin_scale` were removed in `PARAM_VERSION` 6. The stored `temp_max` is **not used** yet.
- Current offsets: `cur_off[i]` = ADC zero − 2048 in counts.
  - `cs_boot_offsets()` (end of `initMotor()`) measures them at every boot over 64 ms.
  - It keeps the result only if the spread is small and it lies within 25 counts of the fallback (`g_params.cur_off` once `PF_CUR_CAL_VALID` is set, else `CS_OFF_DEFAULT`). A rotor turning at boot therefore falls back.
  - The offsets in use and their source are objects 0x4023:4–7.
- `SVPWM()` writes nothing for `FAULT_CLEAR_MIN_TIME` ms after a fault is cleared.
- `CTRL_OFF` applies the zero vector, and so does re-arming MOE. On a spinning motor that is a short-brake: about λ/L of current, far above `imax`. It made STOP from current mode trip on overcurrent. The rule now is **no zero vector above `COAST_END_TURNS` (30 rpm)**:
  - Stops go through `coast_or_off()` in `calib.c`. Above that speed it stays in `CTRL_RUN` at id = iq = 0 (`g_coasting`, 0x4001:15), and `run_watchdog()` switches off once the motor is slow. STOP replies at once, and a run command takes over a coasting motor.
  - `run_stop()` still brakes under the speed loop first in speed/position mode.
  - A lost HFI angle switches the bridge off instead.
  - `checkAndClearBridgeFault()` re-arms only below that speed (encoder), or after `SL_REARM_MS` (sensorless, which cannot see the speed with the bridge off), returning `BRIDGE_ERR_SPIN` until then. Before MOE has been on once since boot, it re-arms at once.
  - Keep `kv_coast()` and this rule.
- **No text in the firmware.** Values go out as raw floats in events/replies, and the PC formats them. Don't call `printf`/`snprintf`/`sprintf`, `strtof`/`strtod`/`atof`, `strtok` (via `assert`) or `assert`: each links back several KB of stdio, bignum or float-printing code (together ~15–19 KB). Check with `arm-none-eabi-nm` for `_vfprintf_r`, `_printf_float`, `_strtod_r`. Avoid `double` too: float varargs are promoted to `double`, which is why `EV()` builds a `const float[]`. Check with `arm-none-eabi-objdump -d` for calls to `__aeabi_f2d`. `-fno-math-errno` makes `sqrtf` compile to a single `vsqrt` instruction.
- `motorCtrl.c`, `curSnsCtrl.c` and the burst/DMA half of `mt6835.c` carry `#pragma GCC optimize("O2")`, so the ISR path is `-O2` in both the `-Og` debug build and the `-Os` release build. In the debug build, locals in `currentLoop()` may show as optimized out. Put new ISR-path code in those files (or under the same pragma).
