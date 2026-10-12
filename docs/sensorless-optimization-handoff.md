# Sensorless optimization handoff

**Written for:** an engineer picking up the sensorless work on this firmware, who knows the FOC code but not the last session.
**Status, 2026-10-12:** sensorless (`angsrc` 2) runs on the bench with the encoder still fitted. The start from 0, reversals at ±2000 rpm, and the 10 rpm and zero-speed holds all work. Two problems are still open: the observer's back-EMF weight at low speed, and the cost of the standstill and low-speed behaviour. Nothing is committed yet.

Read first: `docs/hfi-handoff.md` (how HFI works, and the bring-up history), and `CLAUDE.md` (architecture and the parameter rules).

## 1. What exists now

| Item | Where | Notes |
|---|---|---|
| Angle source | `Core/Inc/params.h` (`ANG_ENCODER` 0, `ANG_ENC_OBS` 1, `ANG_SENSORLESS` 2), param `ang_src`, object `0x4011` sub 1 ("Angle source") | Default 1 (today's behaviour). |
| Angle selection | `currentLoop()` in `Core/Src/motorCtrl.c` | Base is the encoder, or HFI when sensorless. The observer blends in above `obs_lo_v`. Position is refused when sensorless (`ST_NEED_ENC_POS`). |
| Sensorless start | `Core/Src/hfi.c`, `hfi_step()` | `S_ACQ` (150 ms of lock with the PIs held), then polarity pulses, then control. A run command therefore starts about 185 ms late. |
| Calibrations, encoder-free | `Core/Src/calib.c`: `hfi_map_at()`, `hfi_lock_and_pol()`, `cal_kv()` | Map uses a frozen reference and stores the magnitude. Polarity is decided by the pulses. kv reads speed from HFI. |
| Safety | `sl_lost()` in `calib.c`, `g_sl_lost_n` in `motorCtrl.c` | `ST_HFI_LOST` after `SL_LOST_N` (1250) cycles, about 50 ms, of HFI not tracking. |
| Agreement band | `SL_AGREE_MAX_TURNS` (10 turn/s, about 600 rpm) in `Core/Inc/calib.h` | Above this the observer is not checked against HFI. |
| `set_off()` | `Core/Src/hfi.c` | Clears `s_vel`, so the speed telemetry no longer freezes after a stop. |

The PC side is in `software/objects.py` (`angsrc`, `hmode`, and the rest of the HFI and observer objects), and `software/check_sync.py` checks it against the firmware (63 entries).

## 2. Parameters in use

These are the values the bench tests ran with. The defaults in `params.c` are different, so check the device, not the defaults.

| Param | Test value | Default in `params.c` | Notes |
|---|---|---|---|
| `hbw` (HFI PLL bandwidth) | 120 Hz | 50 Hz | The change that fixed the reversals. Set in the Config tab and saved; not yet in the defaults. |
| `acc` (speed ramp) | 40000 rpm/s | 60000 rpm/s | Measured from the recordings. |
| `rated` / `imax` | 8 A / 12 A | 1 A / 1.5 A | Overcurrent trips at `imax`. Overshoot to about 12.5 A raw was seen once at `imax` 12. |
| `oblend`, `olo`, `ohi` | 1, 0.7 V, 1.4 V | same | Blend start and full weight. |
| `oftol`, `oerr` | 0.3, 30° | same | Flux tolerance and agreement limit. |
| `hid`, `hv`, `hhalf`, `hpi` | from the bring-up | | `hpi` must be saved before a run (the start failed once because it was not). |
| `vkp`, `vki` | from the bring-up | 0 | Set before speed mode. Not retuned this session. |

**Lesson from the session:** write, then **Save to flash** before a test. Two runs were lost to an unsaved value.

## 3. What was measured

All figures come from the plotter recordings (`*.bin`, 72-byte telemetry frames). None of the files included the calibration event text, so `hsal` and the polarity event values are not in them.

**Polarity and lock (`calib.bin`):** nine tests from random rotor positions. Seven started on the wrong side (about ±178°) and the pulse flipped each one. After the pulse all nine were within ±7° of the encoder. Lock HF step 0.51–0.61 A.

**Sensorless start from standstill (`calib2.bin`):** HFI locked at 7 s. Over 18 s of tracking at 10 rpm the HFI-minus-encoder error had a median of +3.3° (80 % within −1.7° to +5.8°).

**Reversals (`reversal.bin`, `run_20261012_195213.bin`):**
- At 60000 rpm/s with `hbw` 50: overcurrent at the zero crossing and at the start from 0.
- At 20000 rpm/s with `hbw` 50: one of three full reversals failed at the zero crossing.
- At 40000 rpm/s with `hbw` 120: six full reversals and the start from 0 all completed. Largest filtered current 2.4 A. Observer and encoder agree within 50–75 rpm through each reversal; the observer weight dips to 0 for 14–20 ms at the crossing.

**Holds (`run_20261012_195423.bin`):**
- ±10 rpm (quiet, no pushes): average speed on setpoint (+10.18 / −10.18 rpm). Encoder speed variation 11.5–11.7 rpm, controller-used speed 15 rpm, iq variation 0.13 A. Real rotor motion, most likely cogging or commutation ripple.
- Zero speed: the earlier "hunting" was the operator pushing the shaft. Between pushes the hold is steady (position within 0.001 turns, iq within ±0.2 A). The speed loop returns the rotor to the same position within about 0.3 s.

## 4. Open problems, in priority order

### 4.1 Observer weight at low speed uses the HFI speed (not fixed)
In `currentLoop()`, `vel_base` is the HFI speed when sensorless, and `obs_weight()` computes the back-EMF level from it. Around zero crossing HFI speed is wrong (for example −273 rpm against a true −826 rpm, frame 174420 of `reversal.bin`), so the weight drops to 0 while the observer is right: its speed matched the encoder to within about 5–12 % through the crossing.

The agreement check inside `obs_weight()` also still uses HFI below `SL_AGREE_MAX_TURNS`. That is the mechanism behind the earlier overcurrent (the observer was dropped while still correct).

Proposed fix, not implemented: use the observer's own speed `s_obs_vel` for the back-EMF level, and keep the agreement check against HFI only while HFI speed and observer speed agree. Test on a zero-crossing reversal with `hbw` 120 and `acc` 40000 first, because the current parameters already pass there.

### 4.2 Low-speed encoder ripple at ±10 rpm
Encoder speed varies by about 11.5 rpm at a 10 rpm setpoint, with a quiet current. Check whether the ripple repeats with rotor position (cogging). If it does, a position-dependent torque feedforward or a notch on the speed loop might help. Not started.

### 4.3 Start latency
The sensorless start takes about 185 ms before the speed loop is active (`S_ACQ` plus polarity). This only matters for fast step commands from standstill. Options: shorten `ACQ_N`, or allow a speed step to start when HFI is already locked.

### 4.4 Loop time and CCM SRAM
`CCMRAM` is at 9984 of 10240 bytes (97.5 %). The ISR path grew with `hfi_step()` and the sensorless branches. Re-measure the loop time in the Config tab diagnostics (object `0x4002`) before adding any code to the ISR. A regenerated `.ioc` drops the CCM placement, so re-check it after any CubeMX run.

### 4.5 Defaults
`hbw` is 50 Hz in `params.c`, but the bench value is 120 Hz. Decide whether the default should change. Changing a default value does not change the flash layout, so it needs no `PARAM_VERSION` bump. Devices that already saved parameters keep their saved value.

## 5. Test procedure for any change

1. Set the parameters in the Config tab, **Write**, then **Save to flash**.
2. Sequence: start 0 → −2000 rpm, decelerate to 0, reverse to +2000, back to 0, then a 10 rpm hold (no pushes) and a zero hold (with the shaft left alone for at least 10 s).
3. Record every run as `.bin` and name it with the settings, for example `bw120_acc40k_<run>.bin`.
4. Read these channels: `v[5]` and `v[6]` (id, iq), `v[7]` and `v[15]` (controller speed, encoder speed), `v[9]` (position), `v[11]` (observer minus base), `v[12]` (flux ratio), `v[13]` (observer weight), `v[14]` (observer speed), `v[2]` and `v[3]` (HFI error and speed), `v[8]` (ramped reference).
5. Note the stop reason from the console (`EV_STOPPED`). The telemetry does not contain it.
6. Pushes by hand are normal. Note the time when you push, so the analysis can skip those frames.

## 6. Things not to do
- Do not lower `vkp` or `vki` for the zero hold. The zero-hold oscillation was from pushes.
- Do not use `acc` 60000 rpm/s with sensorless. It overcurrents from standstill.
- Do not treat `calib.bin` or the earlier `angsrc2.bin` as current behaviour. Both came from earlier builds or earlier settings.
- Do not commit the recordings. Move them to a `recordings/` folder, outside the repo tree if possible.
