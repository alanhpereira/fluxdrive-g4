# HFI (high-frequency injection) handoff

**Goal:** get a rotor-angle estimate at zero and low speed without the encoder, to cover the range below the back-EMF observer's band (`obs_lo_v`, about 400–800 rpm on the 345 kv test motor). Together with the existing observer this makes encoderless operation possible. I/f startup was ruled out on purpose.

**Status:** steps 1–6 are implemented as the `angsrc` = 2 (sensorless) mode, with the encoder modes unchanged (`angsrc` 0 and 1). Steps 4–6 ran on hardware on the 2026-10-12 motor with the encoder fitted (see below); the encoder-free bring-up and the new mode are not yet tested on hardware. Keep the encoder fitted for the checks: it is the reference every step is compared against.

## First hardware run (2026-10-12, 31 V, Ld 67 µH / Lq 171 µH, rated 8 A)
- **Saliency (go/no-go):** `hsal` = +24.7 % at id = 0, so HFI is feasible on this motor.
- **Standstill tracking:** stable, with sd ≈ 3° (raw, per frame) and an HF step of 0.21 A at `hv` = 0.5 V. The offset depends on rotor position: +5.5°, −37° and +24° at three positions. Likely cause is dead time: about 0.09 V per phase at 31 V, which is ~18 % of `hv` and changes with the injection direction. Working the numbers back, a 24° offset needs about 0.1 V of cross-axis voltage error. Next: `hv` 1.5–2 V and `hid` 1–2 A, so the phase currents stop crossing zero. Re-map at that `hid`.
- **Polarity:** the contrast was 0.3–3 % and always positive, so the result was wrong whenever the lock landed on −d. `hpi` = 1 A does not saturate an 8 A motor. Next: `hpi` 5–6 A.
- **At 7200 rpm** (observer in control) the HFI estimate is meaningless, as expected. The handover/fade-out is step 5.
- **Loop time 20 µs avg, over budget:** flash runs at 4 wait states and the prefetch buffer was off (`PREFETCH_ENABLE 0U` in hal_conf). `main.c` now enables it at start-up. If that is not enough, the next step is running the ISR path from CCM SRAM. That is a linker-script change: CCM is 10 KB at 0x10000000, today part of the 32 KB "RAM" region at 0x20005800.

- **Encoder error (±20 rpm sweeps):** HFI − encoder repeats once per *mechanical* turn, the same in both directions (+37° … −41° … +40° elec). The observer at 7200 rpm shows the same shape, so the encoder is at fault (magnet/mount, ±6° mech), not HFI. Against the electrical angle HFI is flat within ~3°, with a +11° constant offset. A firmware linearization table was considered and put off; the mechanical fix is pending.
- **Step 4 done on hardware:** with `hmode` = 2 the motor ran on HFI in speed mode from 0 to ±50 rpm and up to 500 rpm, held 0 rpm while the shaft was twisted (iq ±4 A, HFI−encoder ≤ 56°), and never tripped. Iq ripple is ~2× the encoder's (HFI speed noise sd ≈ 15–20 rpm with `hbw` 50 and `vkp` 0.5). Loop time with the ISR path in CCM SRAM: 7.8 µs avg, 14.3 µs max.
- **Steps 5 and 6, first run (`run_20261012_182722`):**
  - **Sensorless start:** locked from angle 0 and kept the correct polarity; HFI−encoder ≈ +15°.
  - **Standstill:** held while the shaft was twisted.
  - **±50 rpm:** fine.
  - **Handover, 0 → 1000 → 0 rpm:** the observer weight rose from 0.08 at 430 rpm to 0.95 at 850 rpm. The injection was off at 1000 rpm and came back at 720 rpm without a glitch.
  - **Failure seen:** another start tripped `ST_HFI_LOST` at id = iq = 0. Cause: `hpi` was raised to 5 A in RAM but not saved, so the polarity pulses ran at 1 A and came out wrong. Not a firmware fault. Each start now sends `EV_HFI_START` (contrast, flipped, HFI−encoder) and stops with `ST_HFI_POL_WEAK` below 5 % contrast.
- **Steps 5 and 6, design:**
  - **Handover:** once the observer reaches full weight (`ohi`), the injection pauses and the HFI estimate follows the commutation angle and speed. The injection resumes below 90 % weight from that angle, with the HF amplitude kept, so the estimate stays valid. The current PIs then drop the injection filter too.
  - **Sensorless start (`angsrc` = 2):** each run begins with `S_ACQ`, 150 ms of tracking from angle 0 with the PIs and outer loops held, then the polarity pulses, then control. A run command therefore starts ~185 ms late. Starting while the motor already spins fast is not handled.
  - **Observer without the encoder:** in `angsrc` = 2 the observer's PLL seed, its back-EMF level and its agreement check come from HFI (or from the commutation speed while paused).
  - **What still uses the encoder:** nothing in sensorless except logging. Position mode is refused (`ST_NEED_ENC_POS`). The safety stop is now `ST_HFI_LOST` after 50 ms without HFI tracking (`SL_LOST_N`).
  - **CCM fill:** 9.2–9.5 KB of 10 KB.

## What is implemented (and how to run it)
- **Module:** `Core/Src/hfi.c` + `Core/Inc/hfi.h` (`-O2` pragma like the rest of the ISR path). `currentLoop()` calls `hfi_step()` after Park. It returns the αβ voltage to add, plus one of: idle, inject (the PIs get `hfi_filter()`'s output, the mean of id/iq over one injection period, and `hv` of voltage is kept free for the injection), or pulse (the PIs are held). The voltage-circle clamp now runs on the αβ vector after the injection is added. That gives the same result as before when no injection is added.
- **Injection period, a correction to the plan below:** a square wave that flips **every** period gives zero signal on this hardware. Duties load at the bottom and the current is sampled at the top, so i_n − i_{n−1} = Ts/(2L)·(u_{n−2} + u_{n−1}). The samples land on the midpoints of the current triangle, and +Vh/−Vh cancel. The half wave is therefore `hhalf` ≥ 2 periods (default 2 = 6.25 kHz). Demodulation weights each difference with (h_{n−2} + h_{n−1})/2, so (hhalf−1)/hhalf of the cycles carry information. The PLL gains are scaled to make up for the empty cycles.
- **Error signal:** across/along = 2·sal·θ̃, with sal = (Lq−Ld)/(Lq+Ld) taken from the map (`hsal`, signed). This makes the PLL gain independent of Vh, L and the bus voltage. A PC model of the demodulation and PLL against a salient motor with this timing locks at 0° or 180°, depending on where it starts. With 1 LSB of noise it stays within ±0.5°. It tracks a slow rotation. With the wrong sign of sal it sits at 90°.
- **Params (v5, object 0x4031, Config tab group "HF injection"):** `hmode` (0 off / 1 estimate only), `hhalf`, `hv` (V), `hbw` (Hz, default 50), `hid` (d bias, A), `hsal` (stored by the map), `hpi` (polarity pulse current, A).
- **Telemetry:** `v[2..4]` used to be the phase currents. They now carry HFI − encoder (deg elec), the HFI PLL speed (rpm) and the demodulated HF current step (A). All three are NaN when HFI is off. With HFI on, `v[5..6]` (id/iq) are the filtered values the PIs see. The plotter has "HFI − encoder", "HFI speed" and "HFI signal" plots.
- **Step 1, saliency map:** calibration `h` (`CAL_HFI_MAP`) in the Config tab. It runs closed-loop current control on the encoder with id = bias and iq = 0, and injects at 24 angles from the encoder d axis, 40 ms of averaging each. It logs L at each angle, then a fit: L min/max, the axis offset from d (cross-saturation), and saliency with a verdict (≥10 % comfortable, 5–10 % workable, below 5 % too low). With the argument at "default" it maps at bias 0, 0.25 and 0.5 × rated and at `hid`, and stores `hsal` from the `hid` sweep. With an explicit bias it maps only that bias and stores nothing. Size `hv` so that the "HFI signal" is several × 12.6 mA: roughly `hv`·40 µs/L per step.
- **Step 2, estimate only:** set `hmode` = 1 (with `angsrc` 0 or 1). HFI then starts from the encoder angle whenever the motor runs (not during calibrations) and logs HFI − encoder. Commutation stays on the encoder. In speed/position mode, id = `hid`. In current mode, set id yourself.
- **Step 3, polarity:** calibration `p` (`CAL_HFI_POL`). HFI locks from a random angle (300 ms). Then come 8 pairs of +d/−d pulses: each pulse is 2 periods, sized from `hpi` and Ld, followed by 50 periods of rest. The routine compares the peak currents, flips the estimate by 180° if −d gave more current, and logs HFI − encoder before and after plus the contrast. It fails with "did not lock" or "wrong side". Run it from many rotor positions.
- **To check after every step:** `currentLoop()` max time and encoder `late` in diagnostics. `hfi_step` is about 2 KB at `-O2` and adds one CORDIC sin/cos per cycle.

## What HFI needs from the motor
HFI reads the rotor angle from how inductance varies with angle. With Σ = (Ld + Lq)/2 and Δ = (Lq − Ld)/2, the current response to an injected voltage varies with 2θ, and its amplitude is proportional to Δ.

- **Surface-magnet outrunners (like the 345 kv test motor) usually have little geometric saliency** (Δ/Σ a few percent). What they do have is saturation saliency: Ld drops when the d axis is driven toward saturation. HFI on such motors often works only with a small positive id bias, or not at all.
- **Rule of thumb:** Δ/Σ ≥ ~10 % is comfortable, 5–10 % is workable with averaging and a slow tracking loop, and below ~5 % is unlikely to be robust.
- **Feasibility is decided by measurement, not by this document.** If step 1 shows too little saliency, stop and report: the alternatives (I/f, or a different motor) are a design decision for the owner.

## Hardware and firmware facts that shape the design
| Item | Value | Consequence |
|---|---|---|
| PWM / control rate | 25 kHz, center-aligned, `currentLoop()` once per period | Square-wave injection must alternate every 2 periods (6.25 kHz) or slower: alternating every period gives no signal (see "What is implemented") |
| Current sample | Once per period, 75 ticks before the counter top (CC4 ≈ 19.6 µs after the bottom) | One sample per period; a second sample (at the bottom) would need an ADC/.ioc change |
| Voltage timing | A duty written in cycle k loads at the next bottom and is centred on the following top (`OUT_DELAY_S` ≈ 40.4 µs) | The current change seen between samples k and k+1 comes from the voltage written in cycle k−1 (and half of k−2). Get this alignment right or the demodulated sign flips |
| Current resolution | ≈ 12.6 mA/LSB (2 mΩ shunt, gain 16, `g_curMult` in `curSnsCtrl.c`) | Example: Σ ≈ 50 µH, Vh = 1 V, one period → Δi ≈ 0.8 A total; a 10 % saliency gives a ~80 mA signal, about 6 LSB. Works, but needs averaging |
| Dead time | 20 ticks ≈ 118 ns (`tim.c`) | ≈ 60 mV of voltage error at 20.5 V bus: small next to Vh of about 1 V, but distorts near phase-current zero crossings |
| Loop time | Budget ≈ 19 µs. The ISR path is now `-O2` in both builds | Re-measure with the Config tab diagnostics (0x4002) with HFI on |
| Telemetry | All 16 floats of the frame are in use | `v[2..4]` (the phase currents) now carry HFI |
| Parameters | v5, no migration | Restore earlier values with the Config tab's JSON import |

## Recommended method
**Square-wave pulsating injection on the estimated d axis**, demodulated from consecutive current samples (Yoon, Sul et al., 2011; widely used on fast-sampling MCU drives). It fits this firmware because it needs no filters or extra sampling and runs at the PWM rate.

1. **Inject:** add ±Vh on the estimated d axis, alternating sign every period (or every 2), on top of the current-PI output and before inverse Park.
2. **Separate the signals:** with Δi_k = i_k − i_{k−1} in the estimated frame, the high-frequency part is the sign-demodulated difference, and the fundamental is the average of consecutive samples. Feed the current PIs with the fundamental only, so the injection doesn't disturb them.
3. **Angle error:** the demodulated q-axis component is proportional to Vh·Ts·Δ/(Ld·Lq)·sin(2θ̃). Normalize it by the d-axis component so the gain doesn't depend on Vh or the bus voltage.
4. **Tracking:** a PLL (the same structure as `obs_pll()`) drives θ̃ to zero. Its bandwidth is a trade-off between noise and how fast the rotor can move; start low (e.g. 50–100 Hz).
5. **Polarity (N/S):** HFI locks at θ or θ + π. Resolve this once at start-up. Apply a short +id pulse and a −id pulse of the same voltage along the estimated d axis and compare the current rise: the axis that saturates (higher current, i.e. lower L) is +d. An Ld measurement at an explicit test current (Config tab, `l` with a current) already shows whether Ld drops at high current, which is what this check relies on.
6. **Hand-over to the observer:** crossfade HFI → observer over a speed band, like the existing encoder → observer blend (`obs_weight()`), and fade the injection amplitude to zero above the band. That removes the noise and losses at speed.

Alternatives, if the square-wave method struggles: sinusoidal rotating injection (lower audible noise, needs band-pass filters and more CPU), or injection every 2 periods (stronger signal, more audible).

## Steps, in order (check each against the encoder)
1. **Saliency map (go/no-go).** Write a test that holds the rotor (shaft locked, or the encoder angle known) and injects the square wave at a set of electrical angles across 0–360° in `CTRL_CAL`. Record the HF current amplitude at each angle and fit Σ + Δ·cos(2θ − φ). Repeat with id bias at 0, +0.25 × rated and +0.5 × rated. **Report Δ/Σ and whether it grows with bias.** Stop here if it stays below ~5 %.
2. **Estimate-only mode.** Commutate from the encoder as today, run injection, demodulation and the PLL alongside, and log HFI − encoder error at standstill, under load (iq), and while turning slowly (speed 50 and 200 rpm). Tune Vh, the injection period and the PLL bandwidth here. Expect an offset from cross-saturation that depends on iq; it can be compensated later.
3. **Polarity check:** run it 20 times from random rotor positions. It must be right every time.
4. **HFI in control:** commutate from the HFI angle at standstill and low speed, keeping the encoder for logging only. Test position hold and slow speed (100 rpm) with load steps.
5. **Blend with the observer** across the speed band, in both directions and through zero speed.
6. **Encoderless start:** power up, check polarity, run HFI, accelerate into the observer band. The observer still depends on the encoder in three places in `motorCtrl.c`, and all three must go for this step:
   - `obs_pll()` seeds its speed from `s_pll_vel`.
   - `obs_weight()` computes the back-EMF level from the encoder speed `we`.
   - `obs_weight()` runs the encoder agreement check (`obs_err_max`, `OBS_ENC_DROP_N`).

   Position mode also needs the encoder's turn counting. Without the encoder it would need HFI angle accumulation, and its accuracy is limited.

## Code structure for steps 4–6
- **Angle source:** `currentLoop()` picks the base from `angsrc`: the encoder, or HFI when sensorless, with the observer blended on top. Keep the existing rule that every angle refers to the current-sample time (`theta_m`), with `OUT_DELAY_S` added for inverse Park. `hfi.c` keeps its estimate in `s_ph`; `hfi_estimate()` returns it always, with a valid flag.
- **Hand-over band:** step 5 needs `hlo`/`hhi` params (a v6 bump), with the injection faded out above the band.

## Risks
- **Low saliency** on this motor (see above): the main risk, settled by step 1.
- **CPU time:** injection, demodulation and the PLL add a few hundred cycles per period. Re-check the diagnostics (max loop time, encoder `late`) after every step.
- **Audible noise** at 6.25–12.5 kHz: acceptable for a thesis, but note it.
- **Cross-saturation:** the angle error depends on iq. Measure it in step 2 and compensate as a function of iq if needed.
- **Encoder mount slip:** the encoder mount on the test motor has ~1.9° mechanical of play that shifts under ~8 A (see the observer notes). Fix it before relying on the encoder as ground truth, or keep currents below that level.
