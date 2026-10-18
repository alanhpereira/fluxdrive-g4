#ifndef HFI_H
#define HFI_H

/* High-frequency injection: rotor angle from the inductance saliency at zero and low speed
 * (docs/design-notes.md, HFI). Runs in currentLoop(), only in CTRL_RUN */

#include <stdint.h>

/* what main asks for. HFI_OFF: autostart while running (sensorless, or hfi_mode 1), not during a calibration */
typedef enum {
	HFI_OFF = 0,
	HFI_TRACK, /* inject on the estimated d axis, the PLL tracks it; starts from g_hfi_theta0 */
	HFI_MAP,   /* inject at g_hfi_map_phi from the commutation angle, no tracking (saliency map) */
	HFI_POL,   /* polarity pulses on the estimated d axis; the ISR sets HFI_TRACK again when done */
} hfi_cmd_t;

/* what currentLoop() does with the result of hfi_step() */
typedef enum {
	HFI_IDLE = 0, /* nothing added */
	HFI_INJECT,	  /* add the injection, feed the PIs through hfi_filter(), keep hfi_v of voltage free */
	HFI_HOLD,	  /* polarity pulses or the sensorless start: hold the PIs and the outer loops, apply only the returned voltage */
} hfi_out_t;

#define HFI_HALF_MAX 8 /* PWM periods per half wave, upper limit of g_params.hfi_half */

extern volatile uint8_t g_hfi_cmd;		/* hfi_cmd_t, written by main (and by the ISR at the end of HFI_POL) */
extern volatile uint32_t g_hfi_theta0;	/* HFI_TRACK start angle, Q32 electrical */
extern volatile uint32_t g_hfi_map_phi; /* HFI_MAP injection angle from the commutation d axis, Q32 electrical */

/* HFI_MAP sums: main clears par/perp/n, then sets acc_left = cycles; the ISR counts it down to 0 and adds
 * the demodulated current steps (A, along and across the injection axis) of every informative cycle */
extern volatile uint16_t g_hfi_acc_left, g_hfi_acc_n;
extern volatile float g_hfi_acc_par, g_hfi_acc_perp;

extern volatile uint8_t g_hfi_pol_flip;		  /* last HFI_POL: 1 if the estimate was turned by 180 deg */
extern volatile float g_hfi_pol_contrast;	  /* (peak+ - peak-) / (peak+ + peak-) before any flip */
extern volatile uint8_t g_hfi_start_done;	  /* sensorless start finished (set by the ISR, cleared by main) */
extern volatile float g_hfi_err, g_hfi_err_f; /* HFI - encoder, electrical turns, raw / ~4 ms low-pass; NaN when not tracking */
extern volatile float g_hfi_rpm;			  /* HFI PLL speed, mechanical rpm; NaN when not tracking */
extern volatile float g_hfi_amp;			  /* demodulated HF current step along the injection axis, A; NaN when off */

/* once per cycle, after Clarke/Park. run: CTRL_RUN. sensorless: the sensorless start runs when HFI starts
 * (lock, then polarity pulses). pause: the observer has full weight, stop injecting and follow th_m at vel
 * (mechanical turn/s, the speed in use). i_a, i_b: currents just sampled. th_m: commutation angle at the
 * sample, th_enc: encoder angle at the sample (logging only). Returns what to do; *v_a, *v_b is the voltage
 * to add (alpha-beta, volts) */
hfi_out_t hfi_step(int run, int sensorless, int pause, float vel, float i_a, float i_b, uint32_t th_m, uint32_t th_enc, float* v_a, float* v_b);

/* HFI angle for this cycle's sample (Q32 electrical, predicted by the last hfi_step()) and mechanical speed
 * (turn/s). Always fills both, even when not valid; returns 1 when HFI is tracking with a usable signal or
 * paused at speed (then it is the commutation angle carried forward). Call before hfi_step() */
int hfi_estimate(uint32_t* th, float* vel);

/* with HFI_INJECT: replaces id, iq by their mean over one injection period (the fundamental) */
void hfi_filter(float* id, float* iq);

#endif
