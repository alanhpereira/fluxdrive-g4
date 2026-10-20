/* ISR path (called from currentLoop()): -O2 even in the -Os build, like motorCtrl.c */
#pragma GCC optimize("O2")

/* High-frequency injection (docs/design-notes.md, HFI). Square wave of +-hfi_v on the estimated d axis,
 * demodulated from consecutive current samples.
 *
 * Timing: the duty written in cycle n loads at the next bottom and stays for a whole period, and the
 * currents are sampled near the top, in the middle of that period. So between two samples
 *   i_n - i_{n-1} = Ts/(2L) * (u_{n-2} + u_{n-1})        (half of each of the last two writes)
 * A square wave that flips every period cancels in every difference (the samples sit on the midpoints of
 * the current triangle): a half wave must last hfi_half >= 2 periods. Weighting each difference with
 * w = (h_{n-2} + h_{n-1}) / 2 (h = +-1, the sign injected) demodulates it; w = 0 on the cycles that
 * straddle a flip, so (hfi_half - 1) / hfi_half of the cycles carry information.
 *
 * With Sigma = (Ld+Lq)/2, Delta = (Lq-Ld)/2 and err = rotor - estimate, an informative cycle gives
 *   par  = Vh*Ts * (Sigma + Delta cos 2err) / (Ld*Lq)    along the injection axis
 *   perp = Vh*Ts *  Delta sin 2err / (Ld*Lq)             across it
 * so perp / par ~= 2 * sal * err, with sal = Delta/Sigma (g_params.hfi_sal, from the saliency map). That
 * normalized error drives a PLL like the encoder one. It locks on d or on d + 180 deg: HFI_POL settles
 * which, from the saturation of the d axis (the +d pulse, along the magnet flux, gives more current) */

#include "hfi.h"

#include <math.h>

#include "calib.h"
#include "cordicMgr.h"
#include "params.h"
#include "proto.h" /* g_busy */

#define TS (1.0f / CTRL_FS_HZ)
#define Q32 4294967296.0f
#define TWO_PI 6.2831853f
#define HFI_MIN_SIG 0.03f /* A: an HF step below ~2 LSB of current has nothing to track */
#define AMP_LP 0.02f	  /* per informative cycle: ~2 ms for the HF step amplitude */
#define ERR_LP 0.01f	  /* g_hfi_err_f, per cycle: 4 ms */
#define ERR_CLAMP 0.125f  /* turns electrical: one sample never pulls the PLL more than 45 deg */
/* polarity: POL_PAIRS x (+pulse, rest, -pulse, rest). A pulse is short next to L/R (~3 periods here), so
 * its current rise still shows L, not just R */
#define POL_PULSE 2
#define POL_REST 50
#define POL_PAIRS 8
#define ACQ_N 3750 /* sensorless start: 150 ms of tracking from an arbitrary angle before the polarity check */

/* internal states next to the hfi_cmd_t ones */
#define S_ACQ 10   /* sensorless start: tracking, PIs held, then HFI_POL */
#define S_PAUSE 11 /* injection off at speed (observer in full control): the estimate follows th_m */

volatile uint8_t g_hfi_cmd;
volatile uint32_t g_hfi_theta0, g_hfi_map_phi;
volatile uint16_t g_hfi_acc_left, g_hfi_acc_n;
volatile float g_hfi_acc_par, g_hfi_acc_perp;
volatile uint8_t g_hfi_pol_flip;
volatile float g_hfi_pol_contrast;
volatile uint8_t g_hfi_start_done;
volatile float g_hfi_err = NAN, g_hfi_err_f = NAN, g_hfi_rpm = NAN, g_hfi_amp = NAN;

static uint8_t s_state;		  /* hfi_cmd_t actually running */
static uint8_t s_half;		  /* hfi_half latched at the start */
static float s_gain, s_inv_n; /* s_half/(s_half-1) for the PLL, 1/(2*s_half) for the filter */
static uint8_t s_cnt;		  /* position in the square wave, 0 .. 2*s_half-1 */
static int8_t s_h1, s_h2;	  /* sign written in the last cycle and the one before, 0 = none */
static float s_c1, s_s1;	  /* injection axis of the last write */
static float s_ia, s_ib;	  /* currents at the previous sample */
static uint32_t s_ph;		  /* estimated electrical angle at the sample, Q32 */
static float s_vel;			  /* electrical turn/s */
static float s_amp;			  /* low-passed par, A; 0 = no informative cycle yet */
/* fundamental for the PIs: mean over one injection period */
static float s_rd[2 * HFI_HALF_MAX], s_rq[2 * HFI_HALF_MAX];
static uint8_t s_ri, s_fill;
/* polarity sequence */
static uint16_t s_pt;
static float s_i0, s_pk, s_sp, s_sn;
static uint8_t s_pol_auto; /* this HFI_POL is the sensorless start, not the CAL_HFI_POL one */
static uint16_t s_acq;
static uint8_t s_report; /* raise g_hfi_start_done on the first tracking cycle after the start */

/* (re)start the square wave: the next differences are not demodulated until two signs are written */
static void wave_restart(float i_a, float i_b) {
	s_cnt = 0;
	s_h1 = s_h2 = 0;
	s_ia = i_a;
	s_ib = i_b;
	s_fill = 1;
}

static void set_off(void) {
	s_state = HFI_OFF;
	s_vel = 0.0f; /* the estimate's speed must not outlive the tracking (telemetry, observer seed) */
	g_hfi_err = g_hfi_err_f = g_hfi_rpm = g_hfi_amp = NAN;
}

static int ready(void) {
	return (g_params.flags & PF_L_VALID) && g_params.hfi_sal != 0.0f && g_params.hfi_v > 0.0f;
}

/* polarity pulses: a few hundred cycles per start, so it stays out of CCM */
static __attribute__((noinline)) hfi_out_t pol_step(float i_a, float i_b, float* v_a, float* v_b) {
	float s, c;
	sincos_q31(s_ph, &s, &c);
	const float id_h = i_a * c + i_b * s;
	const uint16_t per = POL_PULSE + POL_REST;
	const uint16_t t = s_pt % per;
	const int neg = (s_pt / per) & 1;
	const float sg = neg ? -1.0f : 1.0f;
	if (t == 0) {
		s_i0 = id_h; /* sampled before this pulse is applied */
		s_pk = 0.0f;
	}
	else if (t <= POL_PULSE + 2) { /* the peak is at the bottom after the last pulse period, between two samples */
		float x = sg * (id_h - s_i0);
		if (x > s_pk) s_pk = x;
		if (t == POL_PULSE + 2) {
			if (neg)
				s_sn += s_pk;
			else
				s_sp += s_pk;
		}
	}
	if (t < POL_PULSE) {
		/* volts for about hfi_pol_i after POL_PULSE periods, from the d-axis inductance */
		float vp = sg * g_params.hfi_pol_i * g_params.ld * (CTRL_FS_HZ / POL_PULSE);
		*v_a = vp * c;
		*v_b = vp * s;
	}
	if (++s_pt >= 2u * per * POL_PAIRS) {
		float sum = s_sp + s_sn;
		g_hfi_pol_contrast = sum > 0.0f ? (s_sp - s_sn) / sum : 0.0f;
		g_hfi_pol_flip = s_sp < s_sn;
		if (g_hfi_pol_flip) s_ph += 0x80000000u;
		wave_restart(i_a, i_b);
		s_state = HFI_TRACK;
		if (!s_pol_auto)
			g_hfi_cmd = HFI_TRACK; /* tells main it is done */
		else
			s_report = 1;
	}
	return HFI_HOLD;
}

/* injection starts: once per run, so it stays out of CCM. acq: the sensorless start (lock from wherever, then
 * the polarity pulses; the PIs wait). Otherwise the estimate starts at th0: the reference for an autostart
 * (the encoder, or the Hall angle, +-30 deg: the right polarity), g_hfi_theta0 for a calibration */
static __attribute__((noinline)) void start(uint8_t want, int acq, uint32_t th0, float i_a, float i_b) {
	uint8_t h = g_params.hfi_half;
	s_half = h < 2 ? 2 : h > HFI_HALF_MAX ? HFI_HALF_MAX
										  : h;
	s_gain = (float)s_half / (float)(s_half - 1);
	s_inv_n = 0.5f / (float)s_half;
	wave_restart(i_a, i_b);
	s_vel = 0.0f;
	s_amp = 0.0f;
	g_hfi_err_f = NAN; /* set from the first tracking cycle */
	if (acq) {
		s_ph = 0;
		s_acq = 0;
		s_state = S_ACQ;
	}
	else {
		s_ph = th0;
		s_state = (want == HFI_MAP) ? HFI_MAP : HFI_TRACK;
	}
}

hfi_out_t hfi_step(int run, int sensorless, int pause, float vel, float i_a, float i_b, uint32_t th_m, uint32_t th_ref, float* v_a, float* v_b) {
	*v_a = *v_b = 0.0f;
	uint8_t want = HFI_OFF;
	int autostart = 0;
	if (run) {
		want = g_hfi_cmd;
		/* autostart: sensorless and Hall + HFI always; hfi_mode 1 only in the encoder modes */
		const uint8_t src = g_params.ang_src;
		const int auto_on = sensorless || src == ANG_HALL_HFI || (src <= ANG_ENC_OBS && g_params.hfi_mode >= 1);
		if (want == HFI_OFF && auto_on && !g_busy && ready()) {
			want = HFI_TRACK;
			autostart = 1;
		}
	}
	if (want == HFI_OFF) {
		if (s_state != HFI_OFF) set_off();
		return HFI_IDLE;
	}

	if (s_state == HFI_OFF) start(want, autostart && sensorless, autostart ? th_ref : g_hfi_theta0, i_a, i_b);
	if (s_state == S_ACQ && ++s_acq >= ACQ_N) {
		s_pt = 0;
		s_sp = s_sn = 0.0f;
		s_pol_auto = 1;
		s_state = HFI_POL;
	}
	else if (want == HFI_POL && s_state == HFI_TRACK) {
		s_pt = 0;
		s_sp = s_sn = 0.0f;
		s_pol_auto = 0;
		s_state = HFI_POL;
	}
	else if ((want == HFI_TRACK || want == HFI_MAP) && (s_state == HFI_TRACK || s_state == HFI_MAP)) {
		s_state = want;
	}
	if (s_state == HFI_POL) return pol_step(i_a, i_b, v_a, v_b);

	/* at speed the observer has full weight: stop injecting and follow the commutation angle, so the
	 * injection restarts from the right angle when the speed drops */
	if (s_state == HFI_TRACK && pause) s_state = S_PAUSE;
	if (s_state == S_PAUSE) {
		if (pause) {
			s_vel = vel * (float)g_params.pole_pairs;
			s_ph = th_m + (uint32_t)(int32_t)(s_vel * (TS * Q32)); /* for the next sample, like the PLL */
			g_hfi_err = g_hfi_err_f = g_hfi_rpm = g_hfi_amp = NAN;
			return HFI_IDLE;
		}
		wave_restart(i_a, i_b); /* s_amp kept: the estimate stays usable while the wave restarts */
		s_state = HFI_TRACK;
	}

	/* demodulate the step since the previous sample */
	float da = i_a - s_ia, db = i_b - s_ib;
	s_ia = i_a;
	s_ib = i_b;
	const float w = 0.5f * (float)(s_h1 + s_h2);
	float eq = 0.0f;
	int info = 0;
	if (w != 0.0f) {
		float ed = w * (da * s_c1 + db * s_s1);
		eq = w * (db * s_c1 - da * s_s1);
		s_amp = (s_amp != 0.0f) ? s_amp + AMP_LP * (ed - s_amp) : ed;
		info = 1;
		if (g_hfi_acc_left) {
			g_hfi_acc_par += ed;
			g_hfi_acc_perp += eq;
			g_hfi_acc_n++;
		}
	}
	if (g_hfi_acc_left) g_hfi_acc_left--;
	g_hfi_amp = s_amp;

	if (s_state == HFI_TRACK || s_state == S_ACQ) {
		/* PLL, critically damped at hfi_bw; gains scaled up for the cycles that carry nothing */
		const float g = s_gain;
		const float wb = TWO_PI * g_params.hfi_bw;
		float e = 0.0f;
		if (info && s_amp > HFI_MIN_SIG) {
			e = eq / (2.0f * g_params.hfi_sal * s_amp) * (1.0f / TWO_PI); /* electrical turns */
			e = e > ERR_CLAMP ? ERR_CLAMP : e < -ERR_CLAMP ? -ERR_CLAMP
														   : e;
		}
		s_vel += wb * wb * TS * g * e;
		s_ph += (uint32_t)(int32_t)((s_vel + 2.0f * wb * g * e) * (TS * Q32));

		float err = (float)(int32_t)(s_ph - th_ref) * (1.0f / Q32);
		g_hfi_err = err;
		g_hfi_rpm = s_vel * 60.0f / (float)g_params.pole_pairs;
		if (s_report && s_state == HFI_TRACK) {
			s_report = 0;
			g_hfi_start_done = 1;
		}
		if (s_state == S_ACQ) {
			g_hfi_err_f = NAN; /* not locked yet (and maybe on -d): nothing for the watchdog */
		}
		else if (g_hfi_err_f != g_hfi_err_f) {
			g_hfi_err_f = err;
		}
		else {
			float d = err - g_hfi_err_f; /* low-pass the shortest way round */
			d -= (float)(int32_t)(d + (d >= 0.0f ? 0.5f : -0.5f));
			float f = g_hfi_err_f + ERR_LP * d;
			f -= (float)(int32_t)(f + (f >= 0.0f ? 0.5f : -0.5f));
			g_hfi_err_f = f;
		}
	}
	else {
		g_hfi_err = g_hfi_err_f = g_hfi_rpm = NAN;
	}

	/* next sign, on the estimated axis (or the map angle) */
	const uint32_t axis = (s_state == HFI_MAP) ? th_m + g_hfi_map_phi : s_ph;
	float sn, cs;
	sincos_q31(axis, &sn, &cs);
	const int8_t h = (s_cnt < s_half) ? 1 : -1;
	if (++s_cnt >= 2 * s_half) s_cnt = 0;
	s_h2 = s_h1;
	s_h1 = h;
	s_c1 = cs;
	s_s1 = sn;
	const float vh = (float)h * g_params.hfi_v;
	*v_a = vh * cs;
	*v_b = vh * sn;
	return (s_state == S_ACQ) ? HFI_HOLD : HFI_INJECT;
}

void hfi_reseed(uint32_t th, float vel) {
	if (s_state != HFI_TRACK) return;
	s_ph = th;
	s_vel = vel * (float)g_params.pole_pairs;
}

int hfi_estimate(uint32_t* th, float* vel) {
	*th = s_ph; /* advanced by one period at the end of the last hfi_step() */
	*vel = s_vel / (float)g_params.pole_pairs;
	return s_state == S_PAUSE || (s_state == HFI_TRACK && s_amp > HFI_MIN_SIG);
}

void hfi_filter(float* id, float* iq) {
	const int n = 2 * s_half;
	if (s_fill) {
		for (int k = 0; k < n; k++) {
			s_rd[k] = *id;
			s_rq[k] = *iq;
		}
		s_ri = 0;
		s_fill = 0;
	}
	s_rd[s_ri] = *id;
	s_rq[s_ri] = *iq;
	if (++s_ri >= n) s_ri = 0;
	float sd = 0.0f, sq = 0.0f; /* summed afresh: no drift from a running sum */
	for (int k = 0; k < n; k++) {
		sd += s_rd[k];
		sq += s_rq[k];
	}
	*id = sd * s_inv_n;
	*iq = sq * s_inv_n;
}
