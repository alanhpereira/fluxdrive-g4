# Driver TCC: user manual (driver and firmware)

This manual covers the motor driver board and its firmware: connecting it, flashing it, commissioning a motor, running it, the protections, and operation over CANopen. The PC program (plotter, Config tab, CANopen tab, test runner) has its own manual: [user-manual-software.md](user-manual-software.md).

Developer documentation (internals, design decisions, measurements) is in the `*-handoff.md` files in this folder and in `CLAUDE.md`.

---

## 1. What the driver is

A field-oriented-control (FOC) driver for 3-phase permanent-magnet motors (BLDC/PMSM).

| Item | Value |
|---|---|
| Controller | STSPIN32G4 (STM32G431 Cortex-M4F at 170 MHz with an integrated 3-phase gate driver) |
| Bridge | 3-phase bridge of six 100 V MOSFETs (DI100N10PQ), 25 kHz center-aligned PWM |
| Current sensing | 3 low-side shunts, 2 mΩ, internal op-amps with gain 16 (about 12.6 mA per ADC count) |
| Bus voltage sensing | 270 kΩ / 10 kΩ divider |
| Position sensor | MT6835 21-bit magnetic encoder (2 097 152 counts per turn) over SPI |
| Control loops | current loop at 25 kHz; speed and position loops at 2.5 kHz |
| Angle sources | encoder; encoder with a back-EMF flux observer blended in at speed; sensorless (high-frequency injection at low speed, observer at speed) |
| PC link | USB full-speed virtual COM port (ST VID/PID 0483:5740). UART at 2 Mbaud as a build-time alternative |
| Field bus | CANopen on FDCAN1: CiA 301 node, CiA 402 drive profile (PP, PV, HM, CSP, CSV, CST), 20 kbit/s – 1 Mbit/s |
| Parameter storage | internal flash, two redundant copies with CRC |

The driver has no text console and no buttons. Everything is set up and operated from the PC program over USB, or from a CANopen master.

---

## 2. Safety

Read this before powering a motor.

- **The motor moves during calibration.** The encoder calibration turns the rotor slowly. The kv calibration spins it up to half the bus voltage or 10 000 rpm, whichever comes first. Keep hands and loose objects away, and unload the shaft when asked.
- **The Lq calibration needs the shaft held.** It applies torque on the q axis 5 s after the rotor aligns. Only run it if you can hold the shaft firmly or lock it mechanically. It is optional.
- **STOP is always available.** The Run panel's STOP button (and E-STOP on the CANopen tab) works in every state, including during a calibration and while a CANopen master has the motor. It ramps down in speed/position mode, then coasts.
- **A stop does not brake hard.** Above 30 rpm the driver never applies the zero voltage vector, because on a spinning motor that is a short-circuit brake with a current far above the limits. A stopped command therefore *coasts* at zero current until the motor is below 30 rpm, then switches off. A heavy flywheel takes as long as it takes to coast down. If you need a mechanical brake, fit one.
- **Overvoltage from regeneration.** Braking returns energy to the supply. A bench supply that cannot sink current will see its voltage rise. The driver limits braking current near `vmax` and coasts above it (section 8), but use a supply that tolerates back-feeding, a battery, or a brake resistor/clamp for large inertias.
- **Bus voltage.** The default overvoltage trip is 60 V. The bus has a 75 V TVS diode and 100 V MOSFETs and capacitors, so the trip must stay well below 75 V. Never set `vmax` above what the board and the supply can tolerate.
- **Set the current limits before anything else.** The defaults (rated 1 A, max 1.5 A) are deliberately low. Set `rated` and `imax` for your motor before calibrating (section 5).
- **After a flash erase or a parameter version change** all calibration data is gone and the defaults load. The motor will not run in closed loop until it is recalibrated (or a JSON backup is imported and saved).

---

## 3. Connections

Pin-outs below are taken from the KiCad schematic (`hardware/`, sheet "02 - Connections"; also `hardware/Driver TCC schematic.pdf`). Pin 1 is as marked on the board. The small signal connectors are JST PH (2.0 mm pitch).

### Power and motor
| Ref | Connector | Pins | Notes |
|---|---|---|---|
| J3, J4 | XT30 (male) | 1 GND, 2 VM | DC bus. The two are wired in parallel, so the second one can pass the supply on to another board. The driver needs at least `vmin` (default 8 V) to calibrate or run. The bus is protected by a 75 V TVS diode (SZ3KASMC75AT3G), and the MOSFETs and bus capacitors are 100 V parts: keep the supply and the regenerated voltage well below 75 V. |
| J2 | MR30 (3-pin) | 1 OUT1, 2 OUT2, 3 OUT3 | Motor phases, any order. The encoder calibration measures the direction, so swapping two phases only changes the stored direction; recalibrate the encoder after rewiring. |

The logic supply (+5 V, +3.3 V) comes from the STSPIN32G4's internal buck regulator when the bus is up, or from USB through a power multiplexer (TPS2116). With only USB connected, the MCU runs and the PC program can connect and edit parameters, but the motor cannot be driven.

### Communication and programming
| Ref | Connector | Pins | Notes |
|---|---|---|---|
| J1 | USB-C | USB 2.0 full speed | Enumerates as a virtual COM port (no driver needed on Windows 10/11): the configuration and telemetry link. ESD protected (USBLC6-2). |
| J6, J7 | JST PH 2-pin | 1 CANH, 2 CANL | CAN bus through a TJA1051T/3 transceiver. The two are wired in parallel for daisy-chaining. **There is no termination resistor on the board**: fit 120 Ω between CANH and CANL at both ends of the bus. |
| J5 | 2×4 pin header, 2.54 mm | 1 GND, 2 GND, 3 USART1_RX, 4 SWCLK, 5 USART1_TX, 6 SWDIO, 7 RESET, 8 +3.3 V | ST-Link (SWD) for flashing and debugging. The UART (2 Mbaud) is only used by a firmware built with `PROTO_LINK_USB 0`. Pin 8 is the board's 3.3 V for the probe's voltage reference: do not feed the board from it. |

### Sensors and I/O
| Ref | Connector | Pins | Firmware use |
|---|---|---|---|
| J9 | JST PH 6-pin | 1 +3.3 V, 2 GND, 3 SPI1_NSS, 4 SPI1_SCK, 5 SPI1_MISO, 6 SPI1_MOSI | **MT6835 encoder (required for the encoder angle sources).** Mount the magnet centered on the shaft and the sensor rigidly: mount play shows up as angle error under load. |
| J8 | JST PH 5-pin | 1 +3.3 V, 2 GND, 3 ENC_A, 4 ENC_B, 5 ENC_I | Incremental (ABI) encoder input. Not used. |
| J13 | JST PH 5-pin | 1 +3.3 V, 2 GND, 3 HALL1, 4 HALL2, 5 HALL3 | Hall sensors. Not used. |
| J12 | JST PH 2-pin | 1 +3.3 V, 2 M_TEMP | Motor temperature sensor (to a 10 kΩ divider). Not used yet. |
| J11 | JST PH 3-pin | 1 +3.3 V, 2 GND, 3 DO1 | Spare digital I/O (PB10). Not used. |
| J10 | JST PH 4-pin | 1 +3.3 V, 2 GND, 3 DO2, 4 DO3 | Spare digital I/O (PC15, PC14). Not used. |

The +3.3 V on the sensor connectors is the board's logic supply, which also powers the MCU: keep the load on it small.

On the board: an NTC thermistor for the driver temperature (not used yet: `tmax` is stored but not acted on) and an LED on PC13 (not driven by the firmware).

---

## 4. First-time setup of a new board

### 4.1 Option bytes (once per chip)
PB8 is both the BOOT0 pin and the CAN receive pin, and the CAN transceiver holds it high when idle. Out of the box this makes the chip start the ST bootloader instead of the firmware. Set the option bytes once with the ST-Link connected (they survive a flash erase):

```powershell
$cli = "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
& $cli -c port=SWD -ob displ                  # read the current values first
& $cli -c port=SWD -ob nSWBOOT0=0 nBOOT0=1    # boot from main flash, PB8 free for CAN
```

Do not set `nBOOT0=0`: with `nSWBOOT0=0` that boots the system bootloader every time.

### 4.2 Flashing the firmware
- **ST-Link (SWD):** in VS Code run the task "Flash STM" (debug build) or "Flash STM release", or from a terminal:
  ```
  make -f STM32Make.make -j flash
  make -f STM32Make.make -j DEBUG=0 OPTIMIZATION=-Os flash     # release build
  ```
  Delete `build/release/` when you change optimization flags.
- **USB DFU (no ST-Link):** in the PC program, Config tab, press **Reboot to DFU**. The driver resets into the ST system bootloader; flash the `.elf`/`.bin` with STM32CubeProgrammer over USB. A reset or power cycle returns to the firmware.
- **Erase:** `make -f STM32Make.make erase` wipes the firmware **and the stored parameters and CANopen settings.**

### 4.3 What happens at power-up
1. The firmware loads the motor parameters from flash. If there is no valid copy (new board, erased flash, or a firmware with a different parameter layout), it loads the defaults and the Config tab shows every calibration as "missing".
2. It measures the current-sense zero offsets for 64 ms. The result is used only if it is consistent and close to the stored (or default) offsets; otherwise the stored offsets are used. Keep the rotor still at power-up. The offsets in use and their source show in the Statistics box.
3. The encoder is put into its continuous read mode, the bridge is armed, the USB port enumerates, and the CANopen node boots (heartbeat, boot-up message) with the stored node ID and bit rate (default node 1, 1 Mbit/s).

---

## 5. Commissioning a motor

Do this once per motor, with the PC program connected over USB (Config tab). The steps follow the calibration list in the Config tab, top to bottom. Each step's state ("done" / "missing") is shown next to it.

### 5.1 Set the basics first
In the parameter grid, set and **Write changed**:

| Parameter | Key | What to enter |
|---|---|---|
| Rated current | `rated` | The motor's continuous current (A). Calibration currents are fractions of it, and current commands are limited to it. |
| Max current | `imax` | Peak current (A), above `rated`. Exceeding it trips the drive. |
| Bus min / max | `vmin`, `vmax` | Undervoltage stop and overvoltage trip (V). |
| Bandwidth | `bw` | Current-loop bandwidth for the autotune (Hz). At most 1250 Hz (fs/20); 500–1000 Hz is a usual start. |
| Angle source | `angsrc` | Leave at 1 (encoder + observer) or 0 (encoder only) for commissioning. |

### 5.2 Calibration sequence
Run each with **Run** in the Calibration panel. Results appear in the log and in the parameter grid. A calibration can be stopped at any time with **Abort** or **STOP**.

| Step | What it does | Motor | Stores |
|---|---|---|---|
| **c** Current-sense offsets | Measures the three channels' zero at zero volts. | Still | `coff_a/b/c`, flag "current cal" |
| **e** Encoder | Aligns the rotor at 0.5 × rated, sweeps 20 electrical turns, then averages the offset over one mechanical turn each way. Finds direction, pole pairs and the electrical offset. Fails if the rotor does not follow or the pole-pair count is not an integer (lost steps: raise `rated` or unload). | **Turns slowly**, shaft free | `dir`, `pp`, `offset` |
| **r** Resistance | Two DC points on the d axis (0.4 × rated and half that voltage), which cancels the dead-time drop. | Still (aligns) | `rd`, `rq` |
| **l** Inductance d | Decay of a 0.4 × rated d-axis current; also prints an L-vs-current curve. With an explicit test current it measures that point only and stores nothing. | Still | `ld` (and `lq` = `ld`) |
| **q** Inductance q *(optional)* | Same on the q axis. **Hold the shaft**; a 5 s countdown shows before it applies torque. | Held | `lq` |
| **t** PI autotune | Computes the current-loop gains from R, L and `bw` (pole-zero cancellation). | — | `kpd`, `kid`, `kpq`, `kiq` |
| **k** kv | Spins up at 0.25 × rated, then coasts and measures the back-EMF. **Unload the shaft**; it starts after a 2 s countdown. Needs e, r, t. | **Spins fast** | `kv` (the observer derives the flux from it) |

Then:
- **Speed and position gains** (group "Speed / position"): set `vkp` (A per turn/s) and `vki` (A per turn), check `vlim` (speed limit, default 600 rpm) and `acc` (acceleration limit), and `pkp` for position mode (default 20 /s). Speed mode refuses to start while `vkp` = 0. Start with a small `vkp`, increase until the speed follows without ringing, then add `vki`.
- **Save to flash.** Everything above lives in RAM until saved. The save is refused while the motor runs (the flash erase stalls the CPU for ~20 ms, so the bridge is switched off for it).
- **Export JSON** to keep a backup of the calibration.

Optional steps:
- **z** Reset encoder zero: rewrites the MT6835's zero position into its own EEPROM (takes ~6 s). Run **e** again afterwards.
- **h** and **p**: HFI saliency map and polarity test, only needed for sensorless operation (section 7).

### 5.3 Prerequisites the driver enforces
If something is missing, the command is refused with a status that names it, e.g. "run 'e' and 't' first", "set the speed gains first (vkp, vki)", "set the bandwidth first". The usual order is c → e → r → l → (q) → t → k.

---

## 6. Running the motor (USB)

All from the Run panel in the left sidebar of the PC program. Pressing Enter in a field sends that command.

| Command | Input | Behavior |
|---|---|---|
| **Current** | id, iq (A) | Closed-loop current (torque) control. The vector is scaled down to `rated` if larger. iq produces torque; id is normally 0. With no load the motor accelerates until something limits it — use with care. |
| **Speed** | rpm | Speed loop. The setpoint is limited to `vlim` and ramped at `acc` (rpm/s). |
| **Position** | turns from home | Position loop (P) feeding the speed loop, limited to `vlim`. Exact to about 1e-9 turn. Encoder angle sources only. |
| **Home** | — | Makes the present position 0 (stops first if running). |
| **STOP** | — | Speed/position: ramps to zero, then coasts. Current mode: coasts at zero current. Below 30 rpm the bridge then applies zero volts (motor off). |

Switching between modes while running is bumpless: the new loop starts from the present speed/position/current.

Over-limit commands are clipped and reported in the log ("speed setpoint limited to …", "current setpoint limited …").

### Run-time watchdog
While running, the driver stops by itself (and reports why in the log and the Run panel banner) on:
- a gate-driver fault or a bridge break,
- bus voltage below `vmin`,
- overspeed: |speed| > 1.5 × `vlim`,
- encoder lost: no valid angle for 10 ms (cable, supply, CRC errors) — the bridge is switched off and the motor coasts,
- sensorless: HFI lost for 50 ms, or a polarity result too weak to trust,
- the trips in section 8.

---

## 7. Angle sources

Set with `angsrc` (only while stopped), then save.

| `angsrc` | Name | Use |
|---|---|---|
| 0 | Encoder only | Simplest. The observer and HFI still run for logging if enabled, but do not affect control. |
| 1 | Encoder + observer *(default)* | Below `olo` of back-EMF the encoder is used; between `olo` and `ohi` the observer's angle and speed are blended in (with `oblend` = 1), once it has passed its flux check and agreed with the encoder. Position always comes from the encoder. Gives a cleaner speed estimate at high speed. |
| 2 | Sensorless | HFI (high-frequency injection) at standstill and low speed, the observer above `olo`. The encoder is not used for control (only logged). **Position mode is refused.** |

### Sensorless commissioning
Sensorless needs, in addition to r, l, t and k:
1. **h** HFI saliency map. The rotor should stay still; the encoder is the reference. It injects at 24 angles with a d bias and reports the saliency (Lq−Ld)/(Lq+Ld): ≥ 10 % comfortable, 5–10 % workable, < 5 % too low for this method. With the argument at "default" it maps several biases and stores `hsal` at `hid`.
2. Tune `hv` (injection voltage: the "HFI signal" plot should be several × 12.6 mA), `hid` (d bias), `hpi` (polarity pulse current: must be high enough to saturate the d axis, a few A on an 8 A motor).
3. **p** Lock + polarity test. Run it from many rotor positions; it must pick the right side every time.
4. Set `angsrc` = 2, save. Each run then starts with ~185 ms of locking and polarity pulses. The motor whines audibly at the injection frequency (6.25 kHz by default).

Starting while the motor is already spinning fast is not supported in sensorless mode. With the encoder fitted, the plotter's "HFI − encoder" and "Observer − encoder" plots show the estimation error.

`hmode` = 1 runs HFI as an estimate only (logged, not used) in the encoder angle sources, for tuning.

---

## 8. Protections

| Protection | Condition | Reaction | How it ends |
|---|---|---|---|
| Overcurrent | \|id\| or \|iq\| > `imax` for 120 µs | Coasts at zero current. If the current keeps rising (> 1.5 × `imax`, or still above `imax` after 5 ms), the bridge is switched off. | STOP once the motor has coasted down. Never resumes by itself. |
| Overvoltage | bus > `vmax` for 1 ms (`vmax` = 0 disables) | Coasts at zero current: no torque, no regen. | Resumes **by itself** after the bus has been below `vmax − vresume` for 10 ms (counted in "Overvoltage resumes"). Outside a run, STOP resets it. |
| Regen clamp | bus between `vmax − vclamp` and `vmax` | Braking current is reduced linearly to zero at `vmax`; decelerations stretch to what the bus can absorb. Motoring is never limited. `vclamp` = 0 disables. | Automatic. |
| Undervoltage | bus < `vmin` | Stop. Calibrations and runs are refused below it. | Raise the supply. |
| Overspeed | \|speed\| > 1.5 × `vlim` | Stop (coast). | — |
| Encoder lost | no valid angle for 10 ms (encoder modes) | Bridge off, motor coasts. The driver re-sends the encoder setup every 100 ms while stopped, so a sensor that lost power comes back. | Home again if absolute position matters. |
| Gate driver fault | nFAULT / break input | Bridge off by hardware. | The driver re-arms the bridge once the motor is below 30 rpm (sensorless: after 3 s). |

While a trip is latched, every command except STOP and REBOOT is refused with "a trip is latched". The Status box shows the trip and whether the bridge was switched off.

---

## 9. Operation over CANopen

The driver is a generic CiA 402 drive. The EDS for masters is `software/driverTCC.eds`.

### 9.1 Node ID and bit rate
Set over USB in the Config tab (group CAN): `can_id` (1–127, default 1) and `can_kbit` (20, 50, 125, 250, 500, 1000; default 1000). Then **Apply CAN settings** (Statistics box) for an immediate communication reset, and **Save to flash** to keep them. LSS is not supported.

### 9.2 Bring-up sequence (master side)
1. NMT start (operational). SDO also works in pre-operational; PDOs need operational.
2. Select the mode in 0x6060.
3. Controlword 0x6040: 0x06 (shutdown) → 0x07 (switch on) → 0x0F (enable operation).
4. Write set-points (by SDO or RPDO). Read status from 0x6041 / TPDOs.
5. Configure a heartbeat consumer (0x1016) so the drive notices if the master is lost.

"Switch on" checks the same prerequisites as USB (calibration, gains, bus voltage, bridge, trips) for the selected mode. If they fail, the drive stays in "switched on", sets the warning bit (statusword bit 7), and the reason is in 0x4042:2 (shown as text in the PC program).

### 9.3 Modes
| Mode | 0x6060 | Set-point | Notes |
|---|---|---|---|
| Profile position (PP) | 1 | 607A target, bit 4 new set-point, bit 5 change immediately, bit 6 relative | Trapezoid profile at 6081 / 6083 / 6084. One buffered set-point. |
| Profile velocity (PV) | 3 | 60FF | Ramped at 6083 / 6084, limited by 607F and `vlim`. |
| Homing (HM) | 6 | bit 4 rising | Methods 37 and 35 (home at the present position). The position at home becomes −607C. |
| Cyclic sync position (CSP) | 8 | 607A each cycle (+ SYNC) | Interpolated over 0x60C2 (default 1 ms); set 0x60C2 to the master's cycle time. |
| Cyclic sync velocity (CSV) | 9 | 60FF + 60B1 | Clamped to `vlim`. |
| Cyclic sync torque (CST) | 10 | 6071 + 60B2 (‰ of rated current) | Limited by 6072 and `imax`. |

Halt (bit 8) ramps to zero and holds (PV, CSV, HM, CSP), pauses the profile (PP), or gives zero torque (CST). Quick stop ramps at 6085.

### 9.4 Units on the CAN side
| Quantity | Unit |
|---|---|
| Position | increments, 2²¹ per turn (wraps at ±1024 turns) |
| Velocity | inc/s (1 rpm = 34 953 inc/s) |
| Acceleration | inc/s² |
| Torque / current | ‰ of rated current |
| Rated current 6075 / rated torque 6076 | mA / mNm |
| DC link 6079 | mV |

0x606D (velocity window) and 0x606F (velocity threshold) are UNSIGNED32 instead of the standard's UNSIGNED16, because at 2²¹ inc/turn a 16-bit window is below 2 rpm.

### 9.5 Default PDOs
| PDO | COB-ID | Content | Sent |
|---|---|---|---|
| RPDO1 | 0x200+ID | 6040 controlword, 6060 mode | — |
| RPDO2 | 0x300+ID | 607A target position, 60B1 velocity offset | — |
| RPDO3 | 0x400+ID | 60FF target velocity, 6071 target torque | — |
| TPDO1 | 0x180+ID | 6041 statusword, 6061 mode display, 603F error code | on change, at least every 100 ms |
| TPDO2 | 0x280+ID | 6064 position, 606C velocity | every SYNC |
| TPDO3 | 0x380+ID | 6077 torque, 6078 current, 6079 DC link | every SYNC |

The mapping can be changed by the master and stored with 0x1010:2.

### 9.6 Faults (0x603F and EMCY)
| Cause | Code |
|---|---|
| Overcurrent | 0x2310 |
| DC link over / under voltage | 0x3210 / 0x3220 |
| Gate driver fault / break | 0x5400 |
| Encoder lost | 0x7305 |
| Sensorless angle lost / polarity weak | 0x7300 |
| Overspeed | 0x8400 |
| Following error (PP, CSP: 6065 / 6066) | 0x8611 |
| Master lost (heartbeat timeout, or NMT left operational) | 0x8130 |
| Local STOP from USB | 0xFF01 |
| Other | 0x1000 |

The reaction is always the coast described in section 2. Fault reset (controlword bit 7 rising) clears it once the coast has ended. What happens on master loss is set by 0x6007 (0 nothing, 1 fault (default), 2 disable voltage, 3 quick stop).

### 9.7 USB and CAN together
While the CANopen master has the motor (operation enabled), USB run, calibration, home, save and reboot commands are refused ("the CANopen master has the motor"). **USB STOP always works**; the CAN side then faults with 0xFF01. The Statistics box in the PC program shows the CAN node state and the CiA 402 state.

### 9.8 Storing CANopen settings
- 0x1010:2 stores the communication part (PDO mapping, heartbeat, SYNC…) and the node ID / bit rate.
- 0x1010:3 stores the CiA 402 settings and the motor parameters. 0x1010:1 stores all.
- 0x1011:2 / :3 restore the defaults from the next reset.
- Store and restore are refused while the motor runs or a trip is latched.
- A USB **Save to flash** writes both the motor parameters and the CANopen record.

### 9.9 Not supported
LSS, SDO client, TIME, node guarding, SRDO. Homing methods other than 35/37 (no limit switches or index on this hardware). SYNC or RPDO timeouts do not stop the motor; use the heartbeat consumer.

---

## 10. Parameter reference

All parameters are in the Config tab grid, grouped as below. "Stopped" means it can only be changed while the motor is off. Values are in RAM until **Save to flash**. Defaults are what loads with no valid stored copy.

### Motor
| Key | Name | Unit | Default | Notes |
|---|---|---|---|---|
| `kv` | kv | rpm/V | 0 | From **k**, or the datasheet. Needed by the observer and sensorless. |
| `pp` | Pole pairs | — | 7 | From **e**. Stopped. |
| `dir` | Encoder direction | ±1 | +1 | From **e**. Stopped. |
| `offset` | Electrical offset | 2³² = 360° | 0 | From **e**. Stopped. |
| `rd`, `rq` | Phase resistance | mΩ | 1000 | From **r**. |
| `ld`, `lq` | Inductance | µH | 500 | From **l** / **q**. |
| `rated` | Rated current | A | 1.0 | Calibration currents and current-command limit. |
| `imax` | Max current | A | 1.5 | Overcurrent trip level. |
| `flags` | Calibration flags | bits | 0 | 1 offset, 2 current cal, 4 R, 8 L, 16 PI tuned. Restored by JSON import. |
| `angsrc` | Angle source | 0/1/2 | 1 | Section 7. Stopped. |

### Current loop
| Key | Name | Unit | Default | Notes |
|---|---|---|---|---|
| `kpd`, `kpq` | Kp d / q | V/A | 0 | From **t**. Applied immediately. |
| `kid`, `kiq` | Ki d / q | V/(A·s) | 0 | From **t**. |
| `bw` | Bandwidth | Hz | 0 | Autotune input, ≤ 1250 Hz. |

### Speed / position
| Key | Name | Unit | Default | Notes |
|---|---|---|---|---|
| `vkp` | Speed Kp | A/(turn/s) | 0 | Must be > 0 for speed/position mode. |
| `vki` | Speed Ki | A/turn | 0 | |
| `pkp` | Position Kp | 1/s | 20 | Must be > 0 for position mode. |
| `vlim` | Speed limit | rpm | 600 | Speed-command clamp, position-mode limit, overspeed trip at 1.5×. |
| `acc` | Acceleration limit | rpm/s | 60 000 | Speed reference ramp; 0 = off. Not used in CANopen modes (the profile comes from the master). |

### Limits
| Key | Name | Unit | Default | Notes |
|---|---|---|---|---|
| `vmin` | Bus min | V | 8 | |
| `vmax` | Bus max | V | 60 | 0 = off. |
| `vclamp` | Regen clamp band | V | 3 | 0 = off. |
| `vresume` | Overvoltage resume margin | V | 2 | |
| `tmax` | Max temperature | °C | 80 | Not used yet. |

### Current sensing
| Key | Name | Unit | Notes |
|---|---|---|---|
| `coff_a/b/c` | Current offsets | ADC counts | From **c**. The fallback when the boot measurement is not trusted. Stopped. |

### Observer
| Key | Name | Unit | Default | Notes |
|---|---|---|---|---|
| `oblend` | Blend observer | 0/1 | 1 | 0 = run and log only. |
| `ogain` | Flux correction gain | rad/s | 1000 | |
| `olo` / `ohi` | Blend start / full | V (peak back-EMF) | 0.7 / 1.4 | |
| `oftol` | Flux tolerance | — | 0.3 | \|flux\|/λ must stay within 1 ± tol. |
| `oerr` | Max observer error | ° elec | 30 | Sustained disagreement with the encoder drops the observer. |

### HF injection
| Key | Name | Unit | Default | Notes |
|---|---|---|---|---|
| `hmode` | HFI mode | 0/1 | 0 | 1 = estimate and log in encoder modes. Stopped. |
| `hhalf` | Half wave | PWM periods | 2 | 2 = 6.25 kHz. Stopped. |
| `hv` | Injection voltage | V | 0.5 | |
| `hbw` | PLL bandwidth | Hz | 50 | |
| `hid` | d bias | A | 0 | |
| `hsal` | Saliency | % | 0 | From **h**; 0 = not measured. |
| `hpi` | Polarity pulse current | A | 1.0 | |

### CAN
| Key | Name | Default | Notes |
|---|---|---|---|
| `can_id` | Node ID | 1 | Used from the next communication reset. |
| `can_kbit` | Bit rate (kbit/s) | 1000 | 20, 50, 125, 250, 500, 1000. |

---

## 11. Troubleshooting

| Symptom / message | Likely cause and what to do |
|---|---|
| Driver does not appear as a COM port; CAN silent | Option bytes not set: the chip boots the ST bootloader (section 4.1). |
| "bus voltage too low" | Supply below `vmin`, or not connected. |
| "set rated/imax first" | `rated` or `imax` is 0. |
| "run 'e' and 't' first" and similar | A calibration prerequisite is missing; see the Config tab's step list. |
| "set the speed gains first (vkp, vki)" | `vkp` = 0. |
| "rotor did not follow the field" / "pole pairs not an integer" | Encoder calibration lost steps: shaft loaded or `rated` too low; or the encoder is not reading (check the encoder counters). |
| "test current must stay below 0.9 × imax" | Lower the L test current or raise `imax`. |
| "current-sense offsets implausible" | The rotor turned during **c**, or a sense channel is faulty. |
| "a trip is latched" | Press STOP after the motor has coasted down. |
| "encoder: no valid angle for 10 ms" | Check the encoder cable and supply; the Statistics box shows CRC errors, stalls and the sensor status (weak field = magnet too far). |
| "not allowed while the motor runs: stop first" | Saving, rebooting and calibrating need the motor off. |
| "the CANopen master has the motor" | Disable the drive on the CAN side, or press STOP. |
| Loop time max red (≥ 19 µs), or encoder "late" growing | Firmware timing problem; report it with the diagnostics. |
| Motor stick-slips in one direction | Usually mechanical, or a wrong electrical offset: rerun **e**. |
| Everything "missing" after a firmware update | The parameter layout changed (`PARAM_VERSION`): import the JSON backup, **Write changed**, **Save to flash**. |
