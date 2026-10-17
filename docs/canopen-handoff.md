# CANopen (CiA 301 / 306 / 402) handoff

**Goal:** CAN carries normal operation. The drive is a **generic CiA 402 drive**, not an emulation of any vendor's drive:
- **CiA 301 v4.2:** NMT, SDO, PDO, SYNC, EMCY, heartbeat.
- **CiA 306 v1.3:** an EDS describes it (`software/driverTCC.eds`).
- **CiA 402 v4.0:** the state machine and the modes PP, PV, HM, CSP, CSV and CST.

The intended masters are ROS (`ros2_canopen`, usually CSP/CSV) and a PC with a USB-CAN adapter. The node ID and bit rate are set over the USB/serial config protocol (object 0x4040). LSS (CiA 305) is not implemented.

**Status, 2026-10-17:** all of it is implemented and run on hardware on the branch `Feat/CANOpen`. Everything below marked "measured" was measured on the bench (WeAct USB2CANISO, SLCAN, 1 Mbit/s).
- **CiA 301:** boot-up, heartbeat, SDO, NMT, EMCY, PDOs, 0x1010/0x1011.
- **Settings:** node ID and bit-rate change over USB, then apply / SAVE / reboot.
- **CiA 402:** the FSA, CSV ±60 rpm, CST pulses, CSP over RPDO + SYNC, PV, HM 37, and PP (absolute, relative, change immediately, buffered set-point, halt).
- **Faults and stops:** quick stop, a local STOP over USB (→ 0xFF01), heartbeat loss (→ 0x8130), following error (→ 0x8611).
- **Timing:** the loop-time max stays at 17.7–18.3 µs in every mode, and encoder `late` stays 0.

## Layout
| Path | What |
|---|---|
| `CANopen/CANopenNode/` | CANopenNode v4 (upstream `v4.1-9-g9b8beed`), copied in: only the files that are built. `README.md` there records the commit and what was left out (SDO client, TIME, node guarding, LSS…). Don't edit it; configure it from the port header. |
| `CANopen/port/CO_driver_target.h` | Stack configuration: `CO_USE_GLOBALS` (no malloc), features on/off, BASEPRI locks, error-register conditions. Also `#pragma GCC optimize("Os")`: every CANopen source includes this header first, so all of CANopen builds at `-Os` even in the `-Og` debug build. Nothing under `Core/` includes it. |
| `CANopen/port/CO_driver_stm32g4.c` | Lean FDCAN driver: runtime bit timing, software ID matching, bus-off recovery, counters. Owns `FDCAN1_IT0_IRQHandler` and the HAL FDCAN callbacks. |
| `CANopen/od/OD.c`, `OD.h` | **Generated** by CANopenEditor from `driverTCC.xdd`. Never edit by hand. |
| `CANopen/app/co_app.c/h` | Glue: communication resets, the 1 kHz TIM7 tick, identity, 0x1010/0x1011 hooks, the 0x4000–0x40FF bridge to `objdict.c`. |
| `CANopen/app/co_store.c/h` | The CANopen record in flash pages 60–61. |
| `CANopen/app/co402.c/h` | CiA 402: FSA, modes, profile generators, faults, unit conversion, the parameter views. |
| `driverTCC.xdd` (repo root) | The CANopenEditor project: **source of truth for the OD.** |
| `software/driverTCC.eds` | Exported EDS for masters. |

## Changing the object dictionary
1. Edit `driverTCC.xdd` in CANopenEditor (v4.2.3 was used), or edit the XML directly in the editor's format and then open it in the editor to check it.
2. Export **CANopenNode V4** to `CANopen/od/` (`OD.c`/`OD.h`) and **EDS** to `software/driverTCC.eds`. Commit all three together.
3. Run `python software/check_sync.py`. It compares the EDS 0x4000-range entries with `OBJECTS` in `software/objects.py` (index, sub, type, writable or not) and warns when the XDD is newer than the exports. Saving the XDD after exporting triggers that note too; it is harmless when the content didn't change.
4. **Persistence:** a layout change in `PERSIST_COMM` or `PERSIST_APP` makes the stored copy of that part invalid (the record carries a signature of the OD layout). The exported defaults load once; the node ID and bit rate are kept.

New native objects go in three places: `k_tab` in `objdict.c`, `OBJECTS` in `objects.py`, and the XDD, all with the same index, sub and type. SDO reads and writes of 0x4000–0x40FF go to `od_read_fast()`/`od_write()`, so the checks and limits are the same as over USB. The one difference: 0x4001:6 returns the cached value, because the tick must not touch I2C.

## Execution model
| What | Where | Priority |
|---|---|---|
| Control ISR chain (ADC, TIM1, SPI DMA, UART) | unchanged | 0 |
| USB_LP | unchanged | 1 |
| `FDCAN1_IT0` (RX FIFO 0 into the CANopenNode buffers) | `CO_driver_stm32g4.c` | 2 (`CO_IRQ_PRIO`) |
| **TIM7 1 kHz tick:** `CO_process` → SYNC → RPDO → `co402_tick()` → TPDO | `co_app.c` | 2 |
| `co_app_poll()` (communication resets, node reset), `co402_poll()` (FSA transitions, ownership, faults), deferred 0x1010 saves | main loop, from `cmd_poll()` | thread |

- **Not in the `.ioc`:** TIM7 is set up by register in `co_app_init()` (PSC 169, ARR 999). The FDCAN1 and TIM7 interrupts are enabled in code. **Keep both IRQs out of the `.ioc`,** or CubeMX generates handlers that clash with the ones in `CANopen/`.
- **Locks:** `CO_LOCK_*` masks priority ≥ 2 with BASEPRI. The control ISR is never held off.
- **SysTick (priority 15) does not run inside the tick,** so `HAL_GetTick()` stands still there. Count ticks instead.
- **Why the tick runs the stack:** heartbeat and SDO keep going while main blocks (calibrations, `run_stop()`).
- **Why the FSA runs in main:** transitions call `drive_enter()`/`drive_coast()`/`drive_trip_reset()` in `calib.c`, which may wait on the ISR. That never races `run_watchdog()`.
- **Tick → ISR:** the tick only writes references (`g_vel_ref`, `g_iq_ref`, the position handshake), and only while main has armed it (`s_stream`) and nothing else took the motor (`g_can_owner`, `CTRL_RUN`, no coast, no trip).
- **Communication reset** (NMT reset communication, 0x4040:5 apply): `co_app_poll()` runs it in main, because `HAL_FDCAN_Init()` waits on `HAL_GetTick()`. NMT reset node waits for `CTRL_OFF`, then reboots.

## Units (CAN side)
| Quantity | Unit | Conversion |
|---|---|---|
| Position (6062, 6064, 607A, 607C, 607D, 6065, 6067, 60F4) | increments, 2²¹ per revolution (0x608F) | `g_pos_cnt`; int32, wraps at ±1024 turns. The 402 layer unwraps it into int64 internally |
| Velocity (606B, 606C, 606D, 606F, 607F, 6080, 6081, 60B1, 60FF, 6099) | inc/s | turn/s × 2²¹ (1 rpm = 34953 inc/s) |
| Acceleration (6083, 6084, 6085, 609A) | inc/s² | |
| Torque, current (6071–6074, 6077, 6078, 60B2) | ‰ of rated current | iq / `rated_current` × 1000 (Kt cancels) |
| Rated current 6075 / rated torque 6076 | mA / mNm | 6076 = Kt × rated, Kt = 1.5 · pp · λ, λ from kv |
| DC link 6079 | mV | |

**Parameter views:** 6073 (max current, ‰ of rated), 6075 (rated current) and 6080 (max motor speed = `vlim`) read and write `g_params` through `od_write()`, so the same limits apply as over USB. 6076 is read-only. They are saved with the motor parameters, not in the CANopen record.

**Departure from CiA 402:** 0x606D (velocity window) and 0x606F (velocity threshold) are **UNSIGNED32**, not UNSIGNED16. With 2²¹ inc/rev, a 16-bit window is at most 1.9 rpm, which the speed loop's ripple (±10–14 rpm at 60 rpm with today's gains) never fits into. Default 606D = 10 rpm, 606F = 1 rpm.

## Default PDO mapping (dynamic; a master may remap and save it with 0x1010:2)
| PDO | COB-ID | Content | Transmission |
|---|---|---|---|
| RPDO1 | 0x200+ID | 6040 controlword, 6060 mode | 255 (on receipt) |
| RPDO2 | 0x300+ID | 607A target position, 60B1 velocity offset | 255 |
| RPDO3 | 0x400+ID | 60FF target velocity, 6071 target torque | 255 |
| TPDO1 | 0x180+ID | 6041 statusword, 6061 mode display, 603F error code | 255, on change (inhibit 1 ms, event timer 100 ms) |
| TPDO2 | 0x280+ID | 6064 position, 606C velocity | 1 (every SYNC) |
| TPDO3 | 0x380+ID | 6077 torque, 6078 current, 6079 DC link | 1 (every SYNC) |

RPDO4 and TPDO4 are disabled. The statusword TPDO is requested from `co402_tick()` whenever 6041 or 6061 changes.

## CiA 402 behaviour
**FSA ↔ firmware:**
- Not ready / switch on disabled / ready to switch on / switched on = `CTRL_OFF`.
- "Switch on" checks the run prerequisites (`drive_check()`: calibration flags, gains, bus voltage, bridge, trip) for the mode in 0x6060, and is refused if they fail. The `st_t` reason shows in 0x4042:2 and the statusword warning bit (7).
- Enable operation = `drive_enter(loop)`: `CTRL_RUN` with `g_loop` from the mode, bumpless. CSP/PP start at the present position.
- Mode changes while enabled are bumpless.
- Shutdown, disable operation, quick stop and disable voltage follow 605B/605C/605A:
  - **Ramps:** the speed loop ramps to zero at 6084, or 6085 for quick stop, then the drive coasts.
  - **No zero vector above 30 rpm:** a stop is a coast through `coast_or_off()`, as everywhere else.
  - **Quick stop hold:** options 5–8 hold in quick stop active, and enable operation goes back to the mode.
- **Halt (bit 8):**
  - **PV, CSV, HM:** ramp to zero at 6084 (605D = 2: 6085) and hold.
  - **PP:** the profile stops and holds, then continues to the target when halt is released.
  - **CST:** zero torque.
  - **CSP:** not applied; the master owns the trajectory.

**Ownership:**
- **USB is refused while CAN has the motor.** `g_can_owner` is set from enable operation until the motor is off again. During that time USB RUN_*/CAL/HOME/SAVE/REBOOT get `ST_CAN_ACTIVE`.
- **USB STOP always works.** It is a local stop: the CAN side faults with 0xFF01.
- **Statusword bit 9 (remote)** is 0 while USB has the motor or a calibration runs; enable is then refused (`ST_RUNNING`).

**Modes:**
| Mode | 6060 | Loop | What the tick does | Statusword 10 / 12 / 13 |
|---|---|---|---|---|
| PP | 1 | position | Trapezoid (6086 = 0 only) at 6081 (≤ 607F, `vlim`), 6083 up, 6084 down. Replans every tick. Set-point handshake: bit 4 new set-point, bit 5 change immediately, bit 6 relative (to the last target), one buffered set-point when bit 5 = 0 (a further set-point while the buffer is full is ignored). 607D clamps the target. | target reached (6067 for 6068 ms, profile done) / set-point acknowledge / following error |
| PV | 3 | speed | Ramp to 60FF at 6083 (speeding up) / 6084 (slowing down), limited by 607F and `vlim`. | target reached (\|60FF − v\| ≤ 606D for 606E ms, on a 10 ms filtered speed) / speed = 0 (606F/6070) / – |
| HM | 6 | speed (held at 0, ramp 609A) | Methods **37 and 35** (home here) on bit 4 rising; 6098 refuses others (0x06090030). | 10 / 12 / 13 per CiA 402: 001 not started, 000 homing, 011 done, 101 error |
| CSP | 8 | position | Each new 607A is interpolated over 0x60C2 (default 1 ms, 0.4 ms–0.1 s): start point + exact per-cycle increment + feed-forward to the ISR. 607D clamps targets. | – / follows the command / following error |
| CSV | 9 | speed | `g_vel_ref` = 60FF + 60B1, clamped to `vlim`. | – / follows / – |
| CST | 10 | current | iq = (6071 + 60B2) ‰ of rated, limited by 6072 and `imax`. | – / follows / – |

- **Home offset sign:** after homing, the position at home is **−607C**. This follows the CiA 402 text: zero position = home position + home offset. Some vendors use +607C, so check the master's expectation. The ISR applies it through `g_home_pos`; USB HOME still sets 0.
- **Ramps bypass `acc_lim`:** in all CAN modes the speed loop takes its reference as is (`g_ramp_bypass`), because the profile comes from the master or the tick. The regen clamp still applies.
- **Speed limit:** in the position modes `vlim` is applied in the outer loop. Statusword bit 11 (internal limit) shows when it or a 607D clamp acts. The reference itself always completes the step.
- **Following error** (CSP, PP): \|demand − actual\| > 6065 for 6066 ms → fault 0x8611. 0xFFFFFFFF switches it off. Measured: CSP tracks a 5 ms RPDO stream within 3–5° at 94 rpm; PP lags 0.5–1.4 % of a turn at 60 rpm with `pkp` = 5.

**Faults → 603F, EMCY, 0x1001.** `run_watchdog()`/`trip_watchdog()` leave their reason in `g_stop_reason`; `co402_poll()` maps it. The fault reaction is the existing coast. Fault reset (bit 7 rising) runs `drive_trip_reset()` once the coast has ended.

| Cause | 603F | 0x1001 bit |
|---|---|---|
| Overcurrent | 0x2310 | current |
| DC link over / under voltage | 0x3210 / 0x3220 | voltage |
| Gate driver fault / break | 0x5400 | device profile |
| Overspeed | 0x8400 | device profile |
| Sensorless angle lost / polarity weak | 0x7300 | device profile |
| Following error | 0x8611 | device profile |
| Master lost (heartbeat consumer timeout, or NMT left operational) | 0x8130 | communication |
| Local STOP over USB | 0xFF01 | device profile |
| Anything else | 0x1000 | generic |

Master loss applies 0x6007: 0 nothing, 1 fault (default), 2 disable voltage, 3 quick stop. Set up the heartbeat consumer in 0x1016.

## Persistence
- **Motor parameters** (`g_params`, pages 62–63): unchanged, `PARAM_VERSION` 7.
- **CANopen record** (pages 60–61, `co_store.c`): A/B slots with seq and CRC32. It holds the node ID and bit rate, a copy of `OD_PERSIST_COMM` (0x1000–0x1FFF: PDO mapping, heartbeat, SYNC…), a copy of `OD_PERSIST_APP` (the 402 settings) and the OD layout signature. Defaults come from the `.data` load image. The comm part is reapplied at every communication reset, the app part once at boot.
- **USB SAVE** writes both records.
- **0x1010 store:**
  - `:2` (comm) writes the CANopen record.
  - `:3` (application) writes the 402 settings **and** the motor parameters.
  - `:1` writes all of it.
- **0x1011 restore** `:2`/`:3`: the exported defaults load at the next communication reset / boot. The motor parameters have no restore.
- **Only with the motor off:** store and restore are refused unless `g_mode == CTRL_OFF`. The SDO only records the request; the main loop does the save with the bridge off, because each erase stalls the CPU ~20 ms.
- **Identity 0x1018:** vendor 0 (no CiA vendor ID), product 1. The revision is `PROTO_VERSION << 16 | PARAM_VERSION` and the serial is a hash of the STM32 UID, both set at runtime.

## Native objects for CAN (also over USB; Config tab, groups CAN / Diagnostics)
| Object | Content |
|---|---|
| 0x4040:1 / :2 | Node ID (1–127) / bit rate (kbit/s: 20, 50, 125, 250, 500, 1000). Stored in the CANopen record; used from the next communication reset |
| 0x4040:3 / :4 | Node ID / bit rate in use |
| 0x4040:5 | Write 1: communication reset now (the Config tab's "Apply CAN settings"), refused while running |
| 0x4041:1–5 | NMT state, CAN error bits, RX / TX frames, RX overruns |
| 0x4042:1–4 | FSA state, last refusal (`st_t`), fault (`st_t`), CAN has the motor |

The Statistics box in the sidebar shows the CAN row.

## Hardware notes
- **Transceiver:** TJA1051T/3 on FDCAN1. The first board had it mounted rotated 180° (there is no clear pin-1 mark on the package, only text). The symptoms were RXD at 0.2 V and CANH/CANL at 0 V; after rotating it, it worked.
- **PB8 is BOOT0 and FDCAN1_RX.** The transceiver idles RX high, so set the option bytes once per chip (they survive `make erase`):
  ```powershell
  $cli = "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
  & $cli -c port=SWD -ob displ                  # read first
  & $cli -c port=SWD -ob nSWBOOT0=0 nBOOT0=1    # boot from the nBOOT0 bit (1 = main flash), PB8 free
  ```
  Don't set nBOOT0 = 0: with nSWBOOT0 = 0 that boots system memory every time. OpenOCD alternative: `stm32l4x option_write 0 0x20 0x08000000 0x0C000000`, then `stm32l4x option_load 0` (nSWBOOT0 is bit 26, nBOOT0 bit 27 of `FLASH_OPTR`). The BOOT0-pin way into the ST bootloader is gone; DFU goes through `MSG_REBOOT` mode 1.
- **Bit timing:** 170 MHz FDCAN clock, 17 tq per bit (Seg1 13, Seg2 3, SJW 3, sample point 82 %). The prescaler is 10/20/40/80/200/500 for 1000/500/250/125/50/20 kbit/s, set at runtime (`co_can_prescaler()`). AutoRetransmission is on.

## Bench setup and testing
- **Adapter:** WeAct USB2CANISO with the SLCAN firmware.
- **Python:** `pip install python-can canopen`.
- **Two links:** the config link (USB CDC, or the UART on the ST-Link VCP) runs alongside, for STOP and the diagnostics.
  ```python
  import canopen
  net = canopen.Network(); net.connect(interface="slcan", channel="COM8", bitrate=1000000)
  node = net.add_node(1, "software/driverTCC.eds"); node.sdo.RESPONSE_TIMEOUT = 1.0
  node.nmt.state = "OPERATIONAL"
  node.sdo[0x6060].raw = 9                       # CSV
  for cw in (0x06, 0x07, 0x0F): node.sdo[0x6040].raw = cw
  node.sdo[0x60FF].raw = 2097152                 # 60 rpm
  ```
- **Bench artefacts, not firmware faults:**
  - **SLCAN drops a reply now and then.** It drops about one SDO reply in a few hundred; the drive's own counters show nothing lost and a 2–3 ms response time. Use a 1 s SDO timeout and retry. For 1 kHz cyclic tests on Linux, a gs_usb/candleLight adapter as `can0` times better.
  - **EMCYs at connect:** with the adapter port closed, nothing acknowledges the drive's frames. Opening the port gives EMCYs 0x8120 (error passive) and 0x8110 info 0x14 (TX buffer overflow) and their resets.

## Known limits / not done
- **60B1 in CSP:** not used. The feed-forward comes from the target steps.
- **Halt in CSP:** not applied.
- **SYNC and RPDO timeouts don't stop the motor.** Master loss is detected through the heartbeat consumer and the NMT state only. Configure 0x1016 for a real master.
- **Homing:** only 35/37. There are no limit switches or index pulse on this hardware; a hard-stop method would need current sensing in the homing logic.
- **PP:** a single buffered set-point. Bit 9 (change on set-point) and 0x60F2 positioning options are not implemented, and the profile type is trapezoid only.
- **PP/CSP settling** is set by `pkp` (5 /s now: ~0.2 s time constant). Tune it on the motor; it is not a CANopen issue.
- **Position wraps at ±1024 turns** on the CAN side (int32 increments). Internally it is unwrapped, so moves across the wrap work.
- **Not implemented:** LSS, SDO client, TIME, node guarding, SRDO, GFC.
- **Not run yet:** ROS (`ros2_canopen` with `driverTCC.eds` + `bus.yml`, CSP at 1 kHz SYNC).

## Flash (2026-10-17)
Release `-Os` 96.5 KB of 120 KB, debug 105.8 KB (CANopen at `-Os` in both). CANopen and the 402 layer added about 30 KB over the 66.4 KB before. The largest parts: `co402.o` 6.0 KB, `OD.o` 3.6 KB, `CO_PDO.o` 3.5 KB, `CO_SDOserver.o` 2.5 KB. See `docs/flash-optimization-handoff.md` for where to trim (USB CDC ~10 KB, UART HAL ~4 KB).
