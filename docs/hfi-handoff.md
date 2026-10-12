# HFI (high-frequency injection) handoff

**Goal:** get a rotor-angle estimate at zero and low speed without the encoder, to cover the range below the back-EMF observer's band (`obs_lo_v`, about 400–800 rpm on the 345 kv test motor). Together with the existing observer this makes encoderless operation possible. I/f startup was ruled out on purpose.

**Status, 2026-10-12:** nothing is implemented. Whether HFI is feasible at all depends on the motor's saliency (step 1), and that hasn't been measured yet. Keep the encoder fitted throughout development: it is the reference every step is checked against.

## What HFI needs from the motor
HFI reads the rotor angle from how inductance varies with angle. With Σ = (Ld + Lq)/2 and Δ = (Lq − Ld)/2, the current response to an injected voltage varies with 2θ, and its amplitude is proportional to Δ.

- **Surface-magnet outrunners (like the 345 kv test motor) usually have little geometric saliency** (Δ/Σ a few percent). What they do have is saturation saliency: Ld drops when the d axis is driven toward saturation. HFI on such motors often works only with a small positive id bias, or not at all.
- **Rule of thumb:** Δ/Σ ≥ ~10 % is comfortable, 5–10 % is workable with averaging and a slow tracking loop, and below ~5 % is unlikely to be robust.
- **Feasibility is decided by measurement, not by this document.** If step 1 shows too little saliency, stop and report: the alternatives (I/f, or a different motor) are a design decision for the owner.

## Hardware and firmware facts that shape the design
| Item | Value | Consequence |
|---|---|---|
| PWM / control rate | 25 kHz, center-aligned, `currentLoop()` once per period | Square-wave injection can alternate every period (12.5 kHz) or every 2 periods (6.25 kHz) |
| Current sample | Once per period, 75 ticks before the counter top (CC4 ≈ 19.6 µs after the bottom) | One sample per period; a second sample (at the bottom) would need an ADC/.ioc change |
| Voltage timing | A duty written in cycle k loads at the next bottom and is centred on the following top (`OUT_DELAY_S` ≈ 40.4 µs) | The current change seen between samples k and k+1 comes from the voltage written in cycle k−1 (and half of k−2). Get this alignment right or the demodulated sign flips |
| Current resolution | ≈ 12.6 mA/LSB (2 mΩ shunt, gain 16, `g_curMult` in `curSnsCtrl.c`) | Example: Σ ≈ 50 µH, Vh = 1 V, one period → Δi ≈ 0.8 A total; a 10 % saliency gives a ~80 mA signal, about 6 LSB. Works, but needs averaging |
| Dead time | 20 ticks ≈ 118 ns (`tim.c`) | ≈ 60 mV of voltage error at 20.5 V bus: small next to Vh of about 1 V, but distorts near phase-current zero crossings |
| Loop time | `currentLoop()` 13 µs typical, 18.7 µs max at `-Og`; budget ≈ 19 µs | **No room for HFI at `-Og`.** Do step 1 of `docs/flash-optimization-handoff.md` first (`-Os`, with the control path at `-O2`) and re-measure with `f` |
| Telemetry | All 16 floats of the frame are in use | HFI debug values must replace something (e.g. `v[2..4]` phase currents), with `software/plotter.py` `PLOTS` updated to match |
| Parameters | v4, no migration | New HFI params mean bumping `PARAM_VERSION` and recalibrating (see CLAUDE.md) |

## Recommended method
**Square-wave pulsating injection on the estimated d axis**, demodulated from consecutive current samples (Yoon, Sul et al., 2011; widely used on fast-sampling MCU drives). It fits this firmware because it needs no filters or extra sampling and runs at the PWM rate.

1. **Inject:** add ±Vh on the estimated d axis, alternating sign every period (or every 2), on top of the current-PI output and before inverse Park.
2. **Separate the signals:** with Δi_k = i_k − i_{k−1} in the estimated frame, the high-frequency part is the sign-demodulated difference, and the fundamental is the average of consecutive samples. Feed the current PIs with the fundamental only, so the injection doesn't disturb them.
3. **Angle error:** the demodulated q-axis component is proportional to Vh·Ts·Δ/(Ld·Lq)·sin(2θ̃). Normalize it by the d-axis component so the gain doesn't depend on Vh or the bus voltage.
4. **Tracking:** a PLL (the same structure as `obs_pll()`) drives θ̃ to zero. Its bandwidth is a trade-off between noise and how fast the rotor can move; start low (e.g. 50–100 Hz).
5. **Polarity (N/S):** HFI locks at θ or θ + π. Resolve this once at start-up. Apply a short +id pulse and a −id pulse of the same voltage along the estimated d axis and compare the current rise: the axis that saturates (higher current, i.e. lower L) is +d. The new `l <A>` command already shows whether Ld drops at high current, which is what this check relies on.
6. **Hand-over to the observer:** crossfade HFI → observer over a speed band, like the existing encoder → observer blend (`obs_weight()`), and fade the injection amplitude to zero above the band. That removes the noise and losses at speed.

Alternatives, if the square-wave method struggles: sinusoidal rotating injection (lower audible noise, needs band-pass filters and more CPU), or injection every 2 periods (stronger signal, more audible).

## Steps, in order (check each against the encoder)
1. **Saliency map (go/no-go).** Write a test that holds the rotor (shaft locked, or the encoder angle known) and injects the square wave at a set of electrical angles across 0–360° in `CTRL_CAL`. Record the HF current amplitude at each angle and fit Σ + Δ·cos(2θ − φ). Repeat with id bias at 0, +0.25 × rated and +0.5 × rated. **Report Δ/Σ and whether it grows with bias.** Stop here if it stays below ~5 %.
2. **Estimate-only mode.** Commutate from the encoder as today, run injection, demodulation and the PLL alongside, and log HFI − encoder error at standstill, under load (iq), and while turning slowly (`v 50`, `v 200`). Tune Vh, the injection period and the PLL bandwidth here. Expect an offset from cross-saturation that depends on iq; it can be compensated later.
3. **Polarity check:** run it 20 times from random rotor positions. It must be right every time.
4. **HFI in control:** commutate from the HFI angle at standstill and low speed, keeping the encoder for logging only. Test position hold (`a`) and slow speed (`v 100`) with load steps.
5. **Blend with the observer** across the speed band, in both directions and through zero speed.
6. **Encoderless start:** power up, check polarity, run HFI, accelerate into the observer band. The observer still depends on the encoder in three places in `motorCtrl.c`, and all three must go for this step:
   - `obs_pll()` seeds its speed from `s_pll_vel`.
   - `obs_weight()` computes the back-EMF level from the encoder speed `we`.
   - `obs_weight()` runs the encoder agreement check (`obs_err_max`, `OBS_ENC_DROP_N`).

   Position mode (`a`) also needs the encoder's turn counting. Without the encoder it would need HFI angle accumulation, and its accuracy is limited.

## Suggested code structure
- A new module `Core/Src/hfi.c` + `Core/Inc/hfi.h`, hand-written (not CubeMX), with three calls from `currentLoop()`:
  - `hfi_inject()`: returns the dq voltage to add this cycle.
  - `hfi_update(i_alpha, i_beta)`: demodulation and PLL. Returns the angle and a quality flag.
  - `hfi_fundamental()`: the filtered currents for the PIs.
- Angle source selection in `currentLoop()` grows from encoder/observer to encoder/HFI/observer. Keep the existing rule that every angle is referred to the current-sample time (`theta_m`), with `OUT_DELAY_S` added for inverse Park.
- Params (v5): injection amplitude, injection period (1 or 2 PWM periods), PLL bandwidth, the speed band for HFI → observer, and polarity pulse voltage/duration. Use the `kTab` keys in `calib.c` the same way as the observer ones (`o*`), e.g. `hv`, `hper`, `hbw`, `hlo`, `hhi`.
- A console command for the step-1 saliency map, e.g. `j` ("inJection map"), printing L(θ) and the Σ/Δ fit.

## Risks
- **Low saliency** on this motor (see above): the main risk, settled by step 1.
- **CPU time:** injection, demodulation and the PLL add a few hundred cycles per period. Re-check `f` (max loop time, encoder `late`) after every step.
- **Audible noise** at 6.25–12.5 kHz: acceptable for a thesis, but note it.
- **Cross-saturation:** the angle error depends on iq. Measure it in step 2 and compensate as a function of iq if needed.
- **Encoder mount slip:** the encoder mount on the test motor has ~1.9° mechanical of play that shifts under ~8 A (see the observer notes). Fix it before relying on the encoder as ground truth, or keep currents below that level.
