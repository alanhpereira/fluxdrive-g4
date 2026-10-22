# Design notes

Written for developers working on the firmware or the PC software. They record why things are built the way they are, what was measured on the bench, and what is still open. How to *use* the driver and the software is in [user-manual-driver.md](user-manual-driver.md) and [user-manual-software.md](user-manual-software.md); the architecture, build rules and conventions are in [developer-guide.md](developer-guide.md). The tables there (CANopen units, default PDOs, fault codes, parameters) are not repeated here.

These notes condense six per-feature handoff documents (config protocol, CANopen drive, CANopen PC tool, HFI, sensorless tuning, flash budget). The originals, with the full planning history, are in git history before this file was added.

Bench reference: a 345 kv outrunner, Ld 67 µH / Lq 171 µH, rated 8 A, 31 V supply; WeAct USB2CANISO (SLCAN) at 1 Mbit/s.

---

## 1. Binary config protocol

### Why
It replaced a text console. Commands, replies and telemetry share one link as checksummed frames; floats travel as raw IEEE 754 values, so the firmware formats and parses no text; the GUI owns names, units, limits and help. Removing the console saved 5.8 KB of flash (release `-Os`: 62.1 KB → 56.3 KB at the time).

### Format
```
A5 5C | type u8 | tok u8 | len u8 | payload[len] | crc u16
crc = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type..payload, little-endian; len <= 64
```
- `tok` is chosen by the host and echoed in the reply; 0 marks unsolicited device events.
- All fields little-endian. Example: READ of 0x4010:7 with tok 0x12 is `A5 5C 10 12 03 10 40 07 <crc lo> <crc hi>`.
- Telemetry keeps its own sync (`A5 5A`, 72 bytes, rotate-add sum), so recordings stay readable.
- The reply type is `type | 0x80`. Events are type `0x7F` (no reply bit): an earlier `0xC0` collided with the RUN_CUR reply (`0x40 | 0x80`), found on hardware. Request types must stay below `0x7F`.
- Message types, `CAL_*`, `st_t`, `EV_*` and the SDO abort codes are defined in `Core/Inc/proto.h` and mirrored in `software/protocol.py` (checked by `check_sync.py`).

### Decisions
- **Object table, CANopen-style:** every parameter and status value is an index:subindex object (`objdict.c`, native range 0x4000–0x40FF), with type, flags (RO, stored, needs `pi_apply()`, not while running) and device-side min/max. The same table answers SDO over CAN, so both links share the checks. Object access errors are CANopen SDO abort codes.
- **Table hash:** FNV-1a over (index, sub, type, flags) of every entry, compared by the GUI on PING; on a mismatch it LISTs the table and disables the rows that differ.
- **Build id:** FNV-1a of the flash image at start-up (< 1 ms), so every rebuild changes it.
- **Busy mode:** calibrations block in main. While they do, `chk()` calls `proto_poll(1)`: READ is served, ABORT and STOP abort, everything else gets `ST_BUSY`. `proto_poll()` is therefore a re-entrant function, not a second main loop.
- **CAL:** the immediate reply is an ACK (or the refusal); the result is the `EV_DONE` event once the motor is off. If that event is lost, the GUI notices `busy` (0x4001:8) falling back to 0.
- **RUN_*** reject NaN/Inf; a failed RUN stops the motor. SAVE is refused unless the motor is off. STOP during a calibration aborts it.
- **Unsaved-changes flag** (0x4001:9) compares RAM with the active flash slot. Diagnostic counters reset by writing 0 (min = max = 0).
- **Transport:** USB CDC by default (`link.c`). Telemetry and replies share a 2 KB TX ring drained only from the USB IRQ; data flows only while the host holds DTR. USART1 at 2 Mbaud remains the `PROTO_LINK_USB 0` fallback, with its 8-byte RX FIFO enabled at runtime (without it, bytes arriving every 5 µs overran while `currentLoop()` ran). At 2.5 kHz × 72 B, telemetry uses ~90 % of the UART; a reply costs one telemetry frame there (a `seq` gap).

---

## 2. CANopen drive (firmware)

Generic CiA 301 v4.2 / CiA 306 / CiA 402 v4.0 drive, not an emulation of a vendor drive. Intended masters: ROS (`ros2_canopen`, CSP/CSV) and a PC with a USB-CAN adapter. Status 2026-10-17: everything run on hardware.

### Layout
| Path | What |
|---|---|
| `CANopen/CANopenNode/` | CANopenNode v4 (upstream `v4.1-9-g9b8beed`), only the files that are built; its `README.md` records what was left out. Don't edit; configure from the port header. |
| `CANopen/port/CO_driver_target.h` | Stack configuration: `CO_USE_GLOBALS` (no malloc), features, BASEPRI locks, and `#pragma GCC optimize("Os")` for all CANopen code (every CANopen source includes it first; nothing under `Core/` does). |
| `CANopen/port/CO_driver_stm32g4.c` | FDCAN driver: runtime bit timing, software ID matching, bus-off recovery, counters. Owns `FDCAN1_IT0_IRQHandler` and the HAL FDCAN callbacks. |
| `CANopen/od/OD.c`, `OD.h` | Exported by CANopenEditor from `driverTCC.xdd`. Never edit by hand. |
| `CANopen/app/co_app.c` | Communication resets, the 1 kHz TIM7 tick, identity, 0x1010/0x1011, the 0x4000–0x40FF bridge to `objdict.c`. |
| `CANopen/app/co_store.c` | The CANopen record in flash pages 60–61. |
| `CANopen/app/co402.c` | FSA, modes, profile generators, faults, unit conversion, parameter views. |

### Changing the object dictionary
1. Edit `driverTCC.xdd` in CANopenEditor (v4.2.3 used).
2. Export **CANopenNode V4** to `CANopen/od/` and **EDS** to `software/driverTCC.eds`; commit all three together.
3. Run `python software/check_sync.py` (compares the EDS 0x4000 range with `objects.py`, warns when the XDD is newer than the exports).
4. A layout change in `PERSIST_COMM` or `PERSIST_APP` invalidates that part of the stored record (it carries a layout signature): the exported defaults load once; node ID and bit rate are kept.

New native objects go in three places with the same index, sub and type: `k_tab` in `objdict.c`, `OBJECTS` in `objects.py`, and the XDD. SDO access to 0x4000–0x40FF goes through `od_read_fast()`/`od_write()`; the one difference from USB is that 0x4001:6 (gate-driver status) returns a cached value, because the tick must not touch I2C.

### Execution model
| What | Where | Priority |
|---|---|---|
| Control ISR chain (ADC, TIM1) | unchanged | 0 |
| USB_LP / USART1 link | unchanged | 1 |
| `FDCAN1_IT0` (RX FIFO 0 into the stack's buffers) | `CO_driver_stm32g4.c` | 2 |
| TIM7 1 kHz tick: `CO_process` → SYNC → RPDO → `co402_tick()` → TPDO | `co_app.c` | 2 |
| `co_app_poll()` (communication / node reset), `co402_poll()` (FSA transitions, ownership, faults), deferred 0x1010 saves | main, from `cmd_poll()` | thread |

- TIM7 (PSC 169, ARR 999) and both IRQs are set up in code. **Keep them out of the `.ioc`**, or CubeMX generates clashing handlers.
- The tick runs the stack so heartbeat and SDO keep going while main blocks (calibrations, `run_stop()`). SysTick doesn't run inside the tick, so count ticks there instead of `HAL_GetTick()`.
- The FSA runs in main because transitions call `drive_enter()`/`drive_coast()`/`drive_trip_reset()`, which may wait on the ISR; that never races `run_watchdog()`.
- The tick writes only references (`g_vel_ref`, `g_iq_ref`, the position handshake), and only while main has armed it and nothing else took the motor.
- Communication reset runs in main (`HAL_FDCAN_Init()` waits on `HAL_GetTick()`); frames already queued go out first (up to 20 ms). NMT reset node waits for `CTRL_OFF`, then reboots.

### CiA 402 behaviour worth knowing
- **Switch on** runs `drive_check()` for the mode in 0x6060 and is refused if it fails; the `st_t` goes to 0x4042:2 and statusword bit 7. A mode change while enabled is bumpless; one `drive_enter()` refuses is tried once per 0x6060 value and the old mode carries on.
- **Stops** follow 605A–605D: the speed loop ramps at 6084 (6085 for quick stop), then the drive coasts. The no-zero-vector-above-30-rpm rule applies here too. Quick-stop options 5–8 hold in quick stop active; a new enable goes back to the mode.
- **Halt in CSP:** the reference brakes from the master's rate to rest through the PP profile and holds; targets are ignored meanwhile, and on release only targets written from then on count, relative to the halted position.
- **CSP interpolation:** each 607A written is a segment over 0x60C2 (default 1 ms, 0.4 ms–0.1 s) at the master's rate, plus half the reference's offset from the master's path per period. A late frame is ridden through at the same rate for up to one period, so arrival jitter does not reach the speed. An unchanged 607A is a hold that ends exactly on the target. Only targets written after the mode starts count (an earlier build stepped toward a stale 607A at CSP start: 300 rpm jumps).
- **PP:** trapezoid only, replanned every tick, one buffered set-point. Bit 9 and 0x60F2 not implemented.
- **Following error** (CSP, PP): \|demand − actual\| > 6065 for 6066 ms → 0x8611; demand is the ISR's interpolated reference (`g_pos_ref_cnt`). 0xFFFFFFFF disables it.
- **Home offset sign:** position at home = −607C (CiA 402 text: zero = home + offset). Some vendors use +607C; check the master.
- **Ramps bypass `acc_lim`** in all CAN modes (`g_ramp_bypass`): the profile comes from the master or the tick. The regen clamp still applies. Statusword bit 11 shows the `vlim` or 607D clamp acting.
- **Faults:** a trip is a fault in any FSA state; only a fault reset clears it over CAN. A communication reset re-reports an active fault.
- **Heartbeat loss** sends the drive to pre-operational (`NMT_CONTROL` has no `ERR_TO_STOPPED`), so TPDOs stop.
- **Identity 0x1018:** vendor 0, product 1, revision `PROTO_VERSION << 16 | PARAM_VERSION`, serial = hash of the STM32 UID.
- **0x606D / 0x606F are UNSIGNED32** (a deliberate departure: kept from the 2²¹ inc/rev era, when a 16-bit window was under 2 rpm; at 2¹⁶ inc/rev it would reach 60 rpm). Defaults 10 rpm / 1 rpm.

### Persistence
- Motor parameters: pages 62–63 (`params.c`). CANopen record: pages 60–61 (`co_store.c`), A/B slots with seq and CRC32, holding node ID, bit rate, a copy of `OD_PERSIST_COMM` (reapplied at every communication reset) and `OD_PERSIST_APP` (applied once at boot), and the OD layout signature. Defaults come from the `.data` load image.
- 0x1011 restore stores nothing: the other part, node ID and bit rate are copied from the old record. The motor parameters have no restore.
- Store/restore are only recorded by the SDO; main does them with the bridge off (each erase stalls the CPU ~20 ms), and only in `CTRL_OFF` with no trip latched.

### Hardware notes
- **Transceiver:** TJA1051T/3. The first board had it mounted rotated 180° (no clear pin-1 mark, only text): RXD sat at 0.2 V and CANH/CANL at 0 V.
- **PB8 = BOOT0 = FDCAN1_RX:** option bytes nSWBOOT0 = 0, nBOOT0 = 1 once per chip (commands in the driver manual §4.1). OpenOCD alternative: `stm32l4x option_write 0 0x20 0x08000000 0x0C000000`, then `stm32l4x option_load 0` (nSWBOOT0 is bit 26, nBOOT0 bit 27 of `FLASH_OPTR`). The BOOT0-pin way into the bootloader is gone; DFU goes through `MSG_REBOOT` mode 1.
- **Bit timing:** 170 MHz FDCAN clock, 17 tq per bit (Seg1 13, Seg2 3, SJW 3, sample point 82 %); prescaler 10/20/40/80/200/500 for 1000/500/250/125/50/20 kbit/s, set at runtime. AutoRetransmission on.

### ISR timing work (2026-10-17)
- **Before:** `currentLoop()` max 18.9 µs in CSP/PP and 2–13 encoder `late` counts in several motion tests; the ISR ended up to 22 µs after the current sample, past the next encoder trigger at 20.4 µs.
- **Causes (DWT checkpoints):** `HAL_ADC_IRQHandler` ×2 on the first conversion cost 3.6 µs before `currentLoop()` started; the telemetry cycle cost 7.2 µs (HAL UART DMA start, byte-wise `memcpy` from flash, DMA/UART TC interrupts at priority 0); `outer_loops()` ran from flash with an `__aeabi_l2f`.
- **Fixes:** `cs_adc_irq()` (one JEOS interrupt, register reads); telemetry filled in the ISR and finished from the link IRQ (`link_defer()`); UART TX by register at priority 1, `link_tx()` masks with BASEPRI; `outer_loops` in CCM; divides off the per-cycle path.
- **After:** loop max 10.5–12.9 µs across the whole `cotest.py` run, `late` 0 in every motion test; `currentLoop()` starts 2.0–2.5 µs after the CC4 trigger. `store_restore` still counts 1–2 `late`, most likely the flash erase stalling the vector fetch (motor off then; not confirmed).

### Measured on the bench
- SDO response 2–3 ms in the drive. CSP tracks a 5 ms RPDO stream within 3–5° at 94 rpm (one period of lag). PP lags 0.5–1.4 % of a turn at 60 rpm with `pkp` = 5 (settling is set by `pkp`, ~0.2 s time constant, not by CAN). Heartbeat-loss fault 0.23 s after the last heartbeat with 0x1016 = 300 ms.
- Before the CSP interpolation rework, late targets dropped the feed-forward: current spikes 26 → 0 and speed-reference dips 74 → 0 on the CSP sine after it; the same change fixed 500 Hz feed-forward dips in PP.

### Known limits
- 60B1 is not used in CSP (feed-forward comes from the target steps).
- SYNC and RPDO timeouts don't stop the motor; master loss is detected through the heartbeat consumer and the NMT state only.
- Homing: methods 35/37 only (no limit switches or index on this hardware).
- CAN-side position is Q16.16 turns (0x608F = 2¹⁶ inc/rev; the encoder's 2²¹ is truncated by 5 bits) and wraps at ±32 768 turns (unwrapped internally).
- Not implemented: LSS, SDO client, TIME, node guarding, SRDO, GFC.
- Not run yet: ROS (`ros2_canopen` with `driverTCC.eds` + `bus.yml`, CSP at 1 kHz SYNC).

---

## 3. CANopen PC tool

### Structure
- `software/co_link.py` (no Qt): `CoBus` (adapter, `canopen.Network`, raw frame tap into a bounded ring, PC heartbeat producer), `DriveNode` (SDO with retry and abort-code text, native objects by `objects.py` key, NMT, heartbeat watch, EMCY log, PDO read/send, store/restore), `Drive402` (controlword commands, per-mode statusword decoding, `goto_enabled()`, PP handshake, homing, unit helpers), `Streamer` (CSP/CSV/CST set-points + SYNC on a thread).
- `software/canopen_tab.py`: the GUI tab. `software/cotest.py` + `software/cotests/`: the test runner (`check(name, cond, detail)`; every test ends with disable voltage, the mode restored, NMT pre-operational and changed objects restored, in a `finally`).
- The OD comes only from `software/driverTCC.eds` (`canopen.import_od()`); there is no second table of CAN objects to keep in sync.
- `canopen.BaseNode402`'s state machine is deliberately not used: it hides the controlword timing and the refusals the tool needs to show. 0x6040 is driven directly.
- Thread rule: python-can/canopen callbacks only append to locked rings and set flags; the GUI polls them on a `QTimer`.

### Bench facts the design handles
- **python-can 4.6 SLCAN on Windows** reads one byte per `read(1)` and flushes after every frame (pyserial's Win32 `flush()` sleeps 50 ms while the driver still holds TX bytes): replies came 50–300 ms late and SYNC had 50–100 ms gaps. `co_link._fast_slcan()` replaces both: replies 2–4 ms, SYNC 5.00 ms median.
- **GIL:** the GUI starved the SYNC thread (20 ms late every cycle). The busy-wait yields with `sleep(0)`, the tab sets `sys.setswitchinterval(0.0005)`, and the hidden Scope plot doesn't redraw.
- **Lost frames:** about one SDO reply in a few hundred is lost on the PC side (the drive's counters show nothing lost); 1 s timeout, 2 retries. Lost requests happen too (a controlword write that never arrived), so `DriveNode.write(retry=False)` reads back and resends only when the old value is still there. Never blindly retry a 0x6040 edge or 0x4040:5.
- **Software timing:** 5–10 ms periods are clean over SLCAN on Windows; 1 ms is not realistic. For 1 kHz work use a gs_usb/candleLight adapter on Linux, or ROS.
- **EMCYs at connect** (0x8120 error passive, 0x8110 info 0x14 TX overflow, and their resets): with the adapter port closed, nothing acknowledges the drive's frames. Tagged "at connect" for the first 0.5 s.
- **PP target reached** can come on while 6064 passes through the window during slow `pkp` settling; the test checks 6062 on target and 6064 inside 6067 when bit 10 sets.
- **Safety:** never enable on connect; the mode combo sends only on a user pick; on close, disable voltage first, then stop the heartbeat.
- **Shared adapter:** an SLCAN port has one owner; close the GUI's connection before running scripts.

### Seed tests and expected results
| Test | Expected (measured 2026-10-17) |
|---|---|
| identity | boot-up `00`; 0x1018 vendor 0, product 1, revision as above; heartbeat at 0x1017 |
| sdo_errors | 0x06010002, 0x06090030/31/32, 0x06070010… for RO / range / size; 6060 = 2 and 6098 = 1 refused |
| node_id_bitrate | answers at the new ID / rate after 0x4040 + apply; 0x4040:3/4 show it; back again |
| store_restore | a 402 value survives a reset after 0x1010:3; back to the EDS default after 0x1011:3; refused while running |
| fsa_walk | 0x0231 → 0x0233 → 0x0237 → back to 0x0250 |
| refusal | stays switched on, warning bit 7, 0x4042:2 = the reason |
| csv, stops_at_speed | ±60 rpm follows (bit 12); 0x07 / 0x02 at 60 rpm ramp at 6084 / 6085, coast, end in SO / SOD |
| local_stop, master_loss | 0xFF01 / 0x8130 (within ~0.3 s), EMCY; fault reset works after the coast |
| cst | ±20 ‰ pulses followed, speed guard at 4 turn/s |
| csp | sine 0.5 turn, 0.5 Hz, 5 ms stream: within 3–5° at 94 rpm; loop max ≤ 18.3 µs, `late` 0 |
| following_error | 6065 = 200 inc, 6066 = 5 ms, ¼-turn step → 0x8611 |
| pv | ramps ~0.5 s per 120 rpm at 240 rpm/s; target reached with 606D = 10 rpm (flickers with the ripple); halt → bit 10 at standstill |
| hm | 6098 = 37, 607C = ¼ turn → 6064 = −¼ turn, statusword 0x1637 |
| pp | ends within 6067; a buffered pair moves exactly 1.0 turn; halt holds, release continues |
| timing | SDO latency 2–3 ms (n = 500); reports lost replies |

Open: after the full run the motor started to stick-slip going negative in PV (0 rpm at rated current, then breaking loose at −450 rpm). Looks mechanical or a commutation offset, not CAN; check the bench and rerun the motion tests.

---

## 4. High-frequency injection (HFI)

### Why and what
HFI gives a rotor angle at zero and low speed without the encoder, below the observer's band (`olo`, about 400–800 rpm on the bench motor). I/f startup was ruled out on purpose. Method: square-wave pulsating injection on the estimated d axis, demodulated from consecutive current samples (Yoon, Sul et al., 2011). It needs no filters or extra sampling and runs at the PWM rate.

- **Module:** `Core/Src/hfi.c` (`-O2` like the rest of the ISR path). `hfi_step()`, called after Park in `CTRL_RUN`, returns the αβ voltage to add plus one of: idle; inject (the PIs get `hfi_filter()`'s output, the mean of id/iq over one injection period, and `hv` of voltage is kept free); pulse (PIs held). The voltage-circle clamp runs on the αβ vector after the injection is added.
- **Injection period, a key finding:** a square wave that flips every PWM period gives *zero* signal on this hardware. Duties load at the counter bottom and the current is sampled at the top, so i_n − i_{n−1} = Ts/(2L)·(u_{n−2} + u_{n−1}): the samples land on the midpoints of the current triangle and +Vh/−Vh cancel. Hence `hhalf` ≥ 2 periods (default 2 = 6.25 kHz). Demodulation weights each difference with (h_{n−2} + h_{n−1})/2, so (hhalf−1)/hhalf of the cycles carry information; the PLL gains are scaled for the empty cycles.
- **Voltage timing:** a duty written in cycle k loads at the next bottom and is centred on the following top (`OUT_DELAY_S` ≈ 40.4 µs). The current change between samples k and k+1 comes from the voltage written in cycle k−1 (and half of k−2). Get this alignment wrong and the demodulated sign flips.
- **Error signal:** across/along = 2·sal·θ̃ with sal = (Lq−Ld)/(Lq+Ld) from the map (`hsal`, signed). This makes the PLL gain independent of Vh, L and the bus voltage. A PC model with this timing locks at 0° or 180° depending on the start, stays within ±0.5° with 1 LSB of noise, and sits at 90° with the wrong sign of sal.
- **Polarity:** HFI locks at θ or θ + π. 8 pairs of +d/−d pulses (2 periods each, sized from `hpi` and Ld, 50 periods of rest) compare the peak currents; the axis that saturates (more current) is +d. The pulses must saturate the d axis: `hpi` = 1 A gave 0.3–3 % contrast on the 8 A motor and was always positive (wrong whenever the lock landed on −d); 5–6 A works.
- **Hand-over:** once the observer has full weight (`ohi`), the injection pauses and the HFI estimate follows the commutation angle and speed; it resumes below 90 % weight from that angle with the HF amplitude kept, so the estimate stays valid.
- **Sensorless start:** `S_ACQ` (150 ms of tracking from angle 0, PIs and outer loops held), then the polarity pulses, then control: a run starts ~185 ms late. Each start sends `EV_HFI_START` (contrast, flipped, HFI − encoder) and stops with `ST_HFI_POL_WEAK` below 5 % contrast. Starting while spinning fast is not handled.
- **In sensorless the observer** takes its PLL seed, back-EMF level and agreement check from HFI (or from the commutation speed while paused). Position mode is refused; the encoder is only logged.

### What HFI needs from the motor
With Σ = (Ld + Lq)/2 and Δ = (Lq − Ld)/2, the response varies with 2θ and its amplitude is proportional to Δ. Surface-magnet outrunners usually have little geometric saliency but some saturation saliency (Ld drops when the d axis is pushed toward saturation), so a positive id bias (`hid`) can help. Rule of thumb: Δ/Σ ≥ 10 % comfortable, 5–10 % workable with averaging and a slow PLL, < 5 % unlikely to be robust. Feasibility is decided by the saliency map, not by calculation.

Sizing: current resolution ≈ 12.6 mA/LSB. HF step per period ≈ `hv`·40 µs/L; make the "HFI signal" several LSB. Dead time is 20 ticks ≈ 118 ns (≈ 60 mV at 20.5 V, 0.09 V per phase at 31 V), which distorts near phase-current zero crossings.

### Bench results (2026-10-12)
- **Saliency:** +24.7 % at id = 0: feasible.
- **Standstill tracking** at `hv` = 0.5 V: sd ≈ 3° raw, HF step 0.21 A, but a rotor-position-dependent offset (+5.5°, −37°, +24°). Likely dead time (~18 % of `hv`); a 24° offset needs about 0.1 V of cross-axis error. Remedy: `hv` 1.5–2 V and `hid` 1–2 A so the phase currents stop crossing zero, then re-map at that `hid`.
- **HFI − encoder over ±20 rpm sweeps** repeats once per *mechanical* turn, the same in both directions (+37° … −41° … +40° elec), and the observer at 7200 rpm shows the same shape: the encoder (magnet/mount, ±6° mech) is at fault, not HFI. Against the electrical angle HFI is flat within ~3° with a +11° constant offset. A firmware linearization table was put off; the mechanical fix is pending. The encoder mount also has ~1.9° mech of play that shifts under ~8 A.
- **HFI in control** (speed mode): 0 to ±50 rpm and up to 500 rpm, held 0 rpm with the shaft twisted (iq ±4 A, HFI − encoder ≤ 56°), never tripped. Iq ripple ~2× the encoder's (HFI speed noise sd ≈ 15–20 rpm at `hbw` 50, `vkp` 0.5).
- **Handover 0 → 1000 → 0 rpm:** observer weight 0.08 at 430 rpm to 0.95 at 850 rpm; injection off at 1000 rpm, back at 720 rpm without a glitch.
- **Loop time:** 20 µs average from flash (4 wait states, prefetch off) was over budget; with the prefetch on and the ISR path in CCM SRAM, 7.8 µs average / 14.3 µs max. `hfi_step` is ~2 KB at `-O2` and adds one CORDIC sin/cos per cycle.
- One start tripped `ST_HFI_LOST` because `hpi` = 5 A had been written but not saved, so the pulses ran at 1 A: not a firmware fault, and the reason the start verdict event exists.

### Calibration routines
- **Saliency map** (`CAL_HFI_MAP`): closed-loop current control on the encoder at id = bias, iq = 0; injects at 24 angles from the encoder d axis, 40 ms each; logs L per angle, then a fit 1/L = a0 + a2 cos 2φ + b2 sin 2φ, the axis offset from d (cross-saturation) and the saliency with a verdict. Default argument maps at 0, 0.25 and 0.5 × rated and at `hid`, storing `hsal` from the `hid` sweep.
- **Lock + polarity** (`CAL_HFI_POL`): locks from a random angle (300 ms), runs the pulses, logs HFI − encoder before and after and the contrast. Run it from many positions (nine runs on the bench: seven started on the wrong side, all nine ended within ±7°).

---

## 5. Sensorless tuning

State 2026-10-12: sensorless (`angsrc` 2) runs on the bench with the encoder fitted as the reference: start from 0, reversals at ±2000 rpm, 10 rpm and zero-speed holds.

### Bench parameters (differ from the defaults)
| Param | Bench | Default | Note |
|---|---|---|---|
| `hbw` | 120 Hz | 50 Hz | The change that fixed the reversals. Whether the default should change is open (a default change needs no `PARAM_VERSION` bump). |
| `acc` | 40 000 rpm/s | 60 000 rpm/s | 60 000 overcurrents from standstill in sensorless. |
| `rated` / `imax` | 8 A / 12 A | 1 / 1.5 A | One overshoot to ~12.5 A raw seen at `imax` 12. |
| `hpi` | 5–6 A | 1 A | Must be saved before a run. |

### Results
- **Start from standstill:** locked; over 18 s at 10 rpm, HFI − encoder median +3.3° (80 % within −1.7° to +5.8°).
- **Reversals:** at 60 000 rpm/s and `hbw` 50, overcurrent at the zero crossing and the start; at 20 000 rpm/s and `hbw` 50, one of three failed; at 40 000 rpm/s and `hbw` 120, six reversals and the start completed, largest filtered current 2.4 A, observer and encoder within 50–75 rpm, observer weight dipping to 0 for 14–20 ms at the crossing.
- **±10 rpm hold:** average on setpoint (±10.18 rpm); encoder speed varies 11.5–11.7 rpm with iq variation 0.13 A: real rotor motion, probably cogging or commutation ripple.
- **Zero hold:** steady between hand pushes (position within 0.001 turn, iq within ±0.2 A); returns to position within ~0.3 s after a push. The "hunting" seen earlier was the operator.

### Open problems, in priority order
1. **Observer weight at low speed uses the HFI speed.** In sensorless, `obs_weight()` computes the back-EMF level from `vel_base` = HFI speed, which is wrong around a zero crossing (−273 rpm against a true −826 rpm), so the weight drops to 0 while the observer is right (within 5–12 % of the encoder). The agreement check below `SL_AGREE_MAX_TURNS` also uses HFI; that mechanism caused an earlier overcurrent. Proposed, not implemented: use `s_obs_vel` for the back-EMF level, and check agreement against HFI only while HFI and observer speeds agree. Test on a zero-crossing reversal at `hbw` 120 / `acc` 40 000 first.
2. **±10 rpm ripple:** check whether it repeats with rotor position (cogging); a position-dependent torque feed-forward or a speed-loop notch might help.
3. **Start latency** (~185 ms): shorten `ACQ_N`, or skip the acquisition when HFI is already locked.
4. **CCM fill** was 97.5 % at the time; re-measure loop time (0x4002) before adding ISR code.

### Test procedure
1. Set parameters, **Write changed**, **Save to flash** (two runs were lost to an unsaved value).
2. Sequence: 0 → −2000 rpm, to 0, → +2000, to 0, then a 10 rpm hold (no pushes) and a zero hold (shaft alone ≥ 10 s).
3. Record every run as `.bin`, named with the settings (e.g. `bw120_acc40k_<run>.bin`). Don't commit recordings.
4. Channels: `v[5]`/`v[6]` id/iq, `v[7]`/`v[15]` controller/encoder speed, `v[8]` ramped reference, `v[9]` position, `v[11]` observer − base, `v[12]` flux ratio, `v[13]` weight, `v[14]` observer speed, `v[2]`/`v[3]` HFI error/speed.
5. Note the stop reason from the log (`EV_STOPPED`); the telemetry doesn't carry it. Note the times of hand pushes.

Don't lower `vkp`/`vki` for the zero hold: the oscillation was from pushes.

---

## 6. Flash, RAM and CCM budget

### Layout (STM32G431, 128 KB, 2 KB pages)
| Pages | Range | Use |
|---|---|---|
| 0–59 | 0x08000000–0x0801DFFF | Program (`FLASH` LENGTH = 120K in the linker script) |
| 60–61 | 0x0801E000–0x0801EFFF | CANopen record, A/B (`co_store.c`) |
| 62–63 | 0x0801F000–0x0801FFFF | Motor parameters, A/B (`params.c`) |

### Usage (2026-10-17, with CANopen)
- Flash: release `-Os` 96.5 KB (23.5 KB free), debug 105.8 KB (14.2 KB free). CANopen + CiA 402 + FDCAN HAL added ~30 KB; the largest parts are `co402.o` 6.0 KB, `OD.o` 3.6 KB, `CO_PDO.o` 3.5 KB, `CO_SDOserver.o` 2.5 KB.
- RAM: 19.9 of 22 KB including the 4 KB stack reserve (~2.1 KB free). Heap is 0 (`_Min_Heap_Size` = 0: nothing calls `malloc`; restore a heap if something ever needs it). The stack reserve is a guard so a static-data overflow fails at link time.
- CCM: ≈ 9.6 of 10 KB; 10 144 of 10 240 bytes after the Hall sources (2026-10-20, section 7).

### What was done and why
- **ISR path at `-O2`** in every build: `#pragma GCC optimize("O2")` at the top of `motorCtrl.c`, `curSnsCtrl.c`, `hfi.c`, and over the burst/DMA half of `mt6835.c` (costs ~0.8 KB against plain `-Os`). CANopen is `-Os` everywhere through its port header.
- **No newlib text:** `printf`/`strtof`/`strtok`/`assert` each pull in several KB (together ~15–19 KB); the text console's removal and float-array events took out the last `__aeabi_f2d` users. `cal_kv()` uses float Welford means instead of `double` sums.
- **RAM:** the Ld/Lq fit reads the recorder `g_rec` directly instead of a 2.4 KB copy (the ISR stops writing it at state 3); the MT6835 CRC-8 table is `const` in flash.
- **CCM:** cold code moved out to flash: `send_telemetry()`, `outer_loops()` body (`noinline`), `pol_step()`. Later the telemetry fill was split (filled in the ISR, finished in the link IRQ) and `outer_loops` went back into CCM for timing (section 2). The Hall sources added ~2.2 KB to the ISR path; `hall_update()` and the HFI start-up (`start()` in `hfi.c`) went to flash to make it fit (section 7).

### Options if more room is needed
| Option | Saving | Cost |
|---|---|---|
| LTO (`-flto`) | ~5–7 KB | STM32Make's per-file listing flags (`-Wa,-a,-ad,-alms=…`) must go; check weak HAL callbacks, `TIM1_UP_TIM16_IRQHandler`, and pragma interaction; re-measure timing |
| HAL UART → LL | ~3–4 KB | Only if the UART fallback stays |
| Drop USB CDC | ~10 KB | Loses the PC link; not planned |
| `currentLoop` at `-Os` (release) | −1.3 KB CCM | Slower loop |
| Drop the O2 pragma from `motorCtrl.c` / `hfi.c` | −2.8 / −0.8 KB CCM (debug) | Slower loop |
| `outer_loops` to flash | −636 B CCM | Slower outer-loop cycles (the longest); would let `hall_update()` (696 B) back into CCM |

### How to measure
```
make -f STM32Make.make -j DEBUG=0 "OPTIMIZATION=-Os -g"     # release build -> build/release/
python software/flash_report.py                           # component / object / section breakdown
arm-none-eabi-nm --size-sort -S build/release/Driver_TCC.elf | tail -40
arm-none-eabi-size -A build/release/Driver_TCC.elf | grep -E 'ccmram|\.bss|\.data'
```
The map file (`build/Driver_TCC.map`) describes whichever build linked last. Delete `build/release/` after changing optimization flags.

After any change to the ISR path, check the Config tab diagnostics (0x4002) in both builds: `currentLoop()` max under ~19 µs, encoder `late` 0.

---

## 7. Hall angle sources (`ang_src` 3, 4)

### Why and what
For motors with Hall sensors and no encoder. 3 uses the Halls alone (any motor); 4 adds HFI at low speed, where the Halls are coarsest, and uses the Halls for what HFI is bad at: its 180° ambiguity, the start, slips, and the speed range above HFI and below the observer. The observer blends in at speed in both, as with the encoder.

### Decoder (`hall.c`)
- **Polled, not EXTI.** Once per cycle (40 µs). At low speed, where the Halls set the angle, 40 µs is nothing next to the edge spacing; at speed the observer takes over. The EXTI lines in the .ioc fired on noise with J13 empty and would preempt nothing useful at priority 0. A state counts once two samples agree; the edge time is then corrected by 1.5 periods.
- **Speed per edge:** the calibrated width of the sector just crossed over the time it took. The table carries each sensor's placement error, so no averaging is needed, and the estimate is one sector behind the rotor. A first version averaged six edges (one electrical turn) and lagged a whole turn when the speed changed: on the bench (unloaded, 7 pole pairs) the speed loop then went into a limit cycle at 100 rpm (−70…+250 rpm). The first edge from standstill and a reversal give no speed; a reversal or a jump restarts the estimate.
- **Interpolation** advances from the edge at that speed and stops at the sector's end: a late edge means the rotor slowed. Once the time since the edge exceeds the last interval the speed is capped at the sector's width / elapsed, so the estimate decays with the rotor; after 100 ms it reads zero and the angle sits in the sector middle (≤ 30° off: cos 30° = 87 % torque).
- **Table:** for each state the angle where it begins going forward (`hall_edge`, Q16). `CAL_HALL` averages the forward and backward crossings of a forced-angle sweep: the rotor lag and the sensor hysteresis flip sign with the direction, so they cancel. Sector widths must be 60° ± 25°.
- **Position** in 3/4 is the base angle accumulated per cycle (`hall_pos_step()`, int32 maths with a remainder carry, no 64-bit division). In 4 that gives HFI resolution at standstill; the guard keeps it from slipping.

### Hall + HFI
- HFI autostarts on the Hall angle (`th_ref` of `hfi_step()`). Within ±30° of the truth, the PLL locks on the right side: no polarity pulses, no ~185 ms start.
- **Guard:** HFI more than 20° outside the present Hall sector for 2 ms → `hfi_reseed()` on the Hall angle, counted in 0x4002:13. That only costs accuracy; nothing stops the motor while the Halls are there.
- **Crossfade** by Hall speed between `hyb_lo` and `hyb_hi`; the injection pauses once the Hall weight reaches 1 (hysteresis 0.9), like it does for the observer.

### Bench stand-in
The MT6835's U/V/W outputs (register 0x00B: `UVW_RES` = pp − 1, `UVW_MUX` 0, `UVW_OFF` 0; `CAL_UVW` writes and saves it) behave as ideal Hall sensors. `ZERO_POS` moves all outputs, so after 'e' `CAL_UVW` also puts the zero on electrical zero (`uvw_zero()`: 12-bit steps of 0.088° mech; `elec_offset` keeps the rounding, the Hall table is cleared) and the UVW edges sit like well-mounted Halls. The SPI angle stays there as the reference: telemetry v[2] is Hall − encoder in 3 and HFI − Hall in 4. Being ideal they do not exercise the placement correction or the glitch filter; that needs a real Hall motor.

### Bench results (2026-10-21)
Setup: the MT6835's UVW outputs as the Halls, 7 pole pairs, 30 V, the SPI encoder as the reference (unwrapped from telemetry v[0]). **u** moved the encoder zero by 30.06° mech and left `offset` at +0.21°; **e** afterwards measured +0.32°, so `uvw_zero()` has the right sign. Speed gains `vkp` 0.35 / `vki` 15 (tuned on the encoder) unless noted. "Ripple" is the sd of the true speed over 10 ms windows, in rpm.

**Table and decoder.** **H** gave sectors of 56–63°; the sequence 5 → 1 → 3 → 2 → 6 → 4 repeats every 8.6° mech; no invalid state in any run.

**Loop time.** `currentLoop()` max 15.6 µs in 3, 19.2–19.5 µs in 4 with HFI injecting (`enc_late` did not grow). That is at the limit; `outer_loops` to flash and `hall_update()` to CCM would give margin.

**Speed, unloaded.**

| | 10 rpm | 50 rpm | 100 rpm | 300 rpm | 600 rpm | 1000 rpm | 3000 rpm |
|---|---|---|---|---|---|---|---|
| 4 | 9.9 avg (stick-slip on the cogging) | 49.7 avg | 99.6 avg | 299.4 avg (crossfade) | 606.7 avg (Hall, HFI paused; observer weight 0.42, as `olo`/`ohi` give: 1.0 V of back-EMF) | — | — |
| 3, `vkp` 0.35 | runs to > 100 | — | limit cycle −70…+250 | — | — | — | — |
| 3, `vkp` 0.05 / `vki` 2.1 | — | ripple 58 (1: 50) | ripple 28 (1: 11) | ripple 6.8 (1: 5.8) | — | 1000.2 avg, ripple 12, Hall − enc ±9° | Hall − enc −13…+6° |

HFI in 4 started on the Hall angle with no polarity pulses and settled within ±7° of it in 20 ms; no re-seed in any run. With the first, six-edge speed the limit cycle in 3 was worse: the estimate trailed a whole turn (55 rpm while the rotor went from 53 to 280 rpm), hence the per-edge speed. What is left in 3 is the loop: the speed is new once per sector (14 ms at 100 rpm), and the unloaded rotor gains or loses a few hundred rpm within one sector at 0.5 A.

**Position (4).** +0.25 turn and back to 0 landed at +0.2500 and +0.0008; ±2° mech against the encoder while moving. A 1-turn move ran at the speed the gains ask for (~1200 rpm with `pkp` 20) through the Hall range without a slip.

**Torque, unloaded.** ±0.3 A from standstill: 100 → 1000 rpm in 114–117 ms in 1, 3 and 4 alike (~7900 rpm/s), so the Hall angle costs no measurable torque once turning; id within ±0.25 A, iq within ±0.1 A of the reference. At 0.12 A, 1 and 3 stay held by the cogging at the same detents (3 with the sector centre 24° off), while 4 breaks away every time: the injection shakes the rotor out of the detent. Nothing starts at 0.06 A.

**Shaft held lightly by hand** (friction load, 0.5–1.7 A):

| | 1 (encoder) | 4 (Hall + HFI) | 3 (Hall) |
|---|---|---|---|
| 20 rpm | 20.2 avg, ripple 3 | 20.2 avg, ripple 6 | 28 avg, ripple 45 |
| 50 rpm | 49.8 avg, ripple 7 | 49.8 avg, ripple 19 | 38 avg, ripple 52 |
| 100 rpm | 97.4 avg, ripple 12 | 100.2 avg, ripple 11 | 83 avg, ripple 79 |
| ±0.3 A, held | iq sd 0.02 | iq sd 0.01 | iq sd 0.02–0.03, sector centre 19–25° off |

So 4 is near the encoder for low-speed speed and position control; 3 is good for torque control and above a few hundred rpm, and its low-speed speed loop needs its own gains.

**One unexplained runaway.** The first speed run after soldering the UVW wires (mode 1, 10 rpm) ran to full speed; STOP did not end it, since it brakes under the speed loop first. The encoder counters then showed 823 917 CRC errors (none before or since): the wires had been soldered with the board powered. Not reproduced afterwards. The suspect is the CRC rule in `MT6835_Sample()`: after `MT6835_StartBurst()` (also called by `enc_recover()` when the sensor comes back from a supply dip) the check switches itself off after 25 failures in a row if no group has passed yet, and from then on corrupted groups are used as angles.

### Open
- 3: separate (lower) speed gains below the Hall/observer handover, or a gain scheduled by the Hall edge rate.
- 4: with the motor off the angle is the Hall sector centre, so enabling moves the position by up to ±30° electrical; a home taken while off carries that. Fix: hold the position while off, or home only while running.
- 4: the injection lowers the breakaway current, so a small torque command holds less firmly than in 1 or 3.
- Encoder: allow the CRC switch-off only on the first `MT6835_StartBurst()` after boot, not after a recovery.
- `obs_err_max` (30°) may be tight against the Hall angle under acceleration in 3.
- `hyb_lo`/`hyb_hi` defaults (150/400 rpm) are guesses; they worked on the bench.

---

## 8. Temperature measurement (`temp.c`)

### Why and what
`tmax` had been stored since the start without anything reading it. The board has an NTC (TH1, ANTC1608-103F3450FB: 10 kΩ at 25 °C, B 3450 K) against 10 kΩ / 0.1 % from 3.3 V on PC4, and J12 puts a motor sensor from 3.3 V to PC3, against 10 kΩ to GND. The motor NTC is to be the same type, at the end of a cable. Both dividers are ratiometric to VDDA, so `g_vdda` only enters the die-sensor conversion (its TS_CAL values are taken at 3.0 V).

### Decisions
- **Regular conversions, timed away from the current sample.** The injected sequences (CC4) carry the shunts. A regular conversion started from main could overlap one. RM0440 says the injected conversion preempts it, but whether that adds latency was not checked. So main only arms `g_temp_start`, and the TIM1 bottom interrupt sets ADSTART 19.6 µs before CC4. At 42.5 MHz the dividers take 47.5 + 12.5 cycles = 1.4 µs (C26/C19 hold the input for the 5 pF sample capacitor), and the die sensor 247.5 + 12.5 = 6.1 µs (≥ 5 µs required). The two never overlap.
- **The TIM1 update interrupt is always on.** It was enabled by `MT6835_StartBurst()`, so without an answering encoder (Hall-only motor, a lost sensor) the temperatures would never be sampled. The handler already checked `bursting`. A conversion that has not come back in 100 ms makes the readings invalid instead of stale.
- **Derating without touching `currentLoop()`.** CCM had about 50 bytes left, and an `iq_ref` clamp there cost 48. The cap is applied at the sources instead: `g_i_vel` is the speed loop's limit, CST caps its own reference in the 1 kHz tick, and `temp_poll()` caps a current-mode `g_iq_ref` every 20 ms. The cap falls linearly from `imax` to zero over the 10 °C below a limit. At the limit the run stops (coast); it starts again 10 °C below.
- **No parameter version bump.** `mtemp_max` is the former `reserved0` byte (°C, U8), which every stored copy holds as 0 = no motor sensor. The NTC constants, band, hysteresis and the die limit (110 °C) are constants in `temp.h`.
- **Sensor faults.** A reading outside 100…4000 counts (about −50…157 °C) is open or shorted: NaN, and with that sensor's limit set, a stop with `ST_TEMP_SENSOR`. With nothing on J12 the input reads ≈ 0, hence `mtmax` = 0 by default.
- EMCY: 0x4210 (board / die), 0x4310 (motor), 0x4000 (sensor). The CiA 402 meaning of 0x4310 ("drive") is to be checked.

### Bench results (2026-10-21)
- **Idle readings:** board 40.0–40.5 °C and die 59.8–61.0 °C at 31 V, motor off. The chip felt warm, which is consistent. Not checked against a thermometer. The conversions check out against the raw values: board 1 482 counts; die 1 032 counts at VDDA 3.32 V with TS_CAL1 1 040 / TS_CAL2 1 373. J12 open reads 0 counts.
- **No effect on current sensing (300 rpm unloaded, 3 s of telemetry, dev vs this branch on the same board):**

  | | dev | this branch |
  |---|---|---|
  | iq sd | 0.0735 A | 0.0734 A |
  | id sd | 0.0223 A | 0.0225 A |
  | rpm sd | 12.29 | 12.25 |
  | `loop_cyc_max` | 2 588 cycles | 2 586 cycles |

  `enc_late` and CRC stayed 0, with no telemetry drops. Boot offsets unchanged (−6/−22/−7).
- **Derating:**
  - At standstill, `tmax` 45 with the board at 40.4 °C gave a cap of 6.86 A.
  - At 300 rpm with the cap at ≈1 A and the shaft loaded by hand, mean iq held at 0.94–1.04 A (peaks 1.13 A, the current ripple) while the speed fell to 1 rpm. Unloaded it went back to 300 rpm. The same load without a cap had drawn 2.4 A.
- **Stops:**
  - `tmax` 35 refuses a run. At 300 rpm it gives `EV_STOPPED` (`ST_OVERTEMP`, 40.1 °C) and a coast to off.
  - `mtmax` 60 with J12 open refuses with `ST_TEMP_SENSOR`.
  - In CANopen PV: fault 0x603F = 0x4210, EMCY 0x4210 (error register 0x21), and a fault reset to switch on disabled once `tmax` is back.
- `cotest.py --no-motion`: 7 of 7 pass.
- Not tested: USB current mode under derating (an unloaded motor runs away in current mode), and the die limit (110 °C).
