#include "calib.h"

#include <math.h>
#include <string.h> /* memcpy */

#include "boot.h"		/* boot_reboot() */
#include "bridgeCtrl.h" /* checkAndClearBridgeFault(), disableBridge(), initBridge() */
#include "co402.h"
#include "co_app.h"
#include "co_store.h"
#include "tim.h"
#include "hall.h"
#include "hfi.h"
#include "mt6835.h"
#include "motorCtrl.h" /* g_loop_cyc */
#include "pi_ctrl.h"
#include "proto.h"

volatile ctrl_mode_t g_mode = CTRL_OFF;
volatile float g_vd_cmd, g_vq_cmd;
volatile uint32_t g_theta_cmd;
volatile float g_rec[REC_N];
volatile uint16_t g_rec_n;
volatile uint8_t g_rec_state, g_rec_use_q;
volatile pi_t g_pi_d, g_pi_q;
volatile float g_id_ref, g_iq_ref;
volatile loop_t g_loop = LOOP_CURRENT;
volatile pi_t g_pi_vel;
volatile float g_vel_ref;
volatile int64_t g_pos_ref_new;
volatile float g_pos_ff_new;
volatile int64_t g_pos_ff_q_new;
volatile uint16_t g_pos_ff_n_new;
volatile uint8_t g_pos_ref_req;
volatile uint8_t g_pos_here_req;

void pi_apply(void) {
	const float ts = 1.0f / CTRL_FS_HZ;
	g_pi_d.kp = g_params.kp_d;
	g_pi_d.ki_ts = g_params.ki_d * ts;
	g_pi_q.kp = g_params.kp_q;
	g_pi_q.ki_ts = g_params.ki_q * ts;
	g_pi_vel.kp = g_params.vel_kp;
	g_pi_vel.ki_ts = g_params.vel_ki / OUTER_FS_HZ;
}

/* ------------------------------------------------------------------ command origin */
/* commands arrive as binary frames (proto.c); the one being run has token s_tok, which its events carry */
static uint8_t s_tok;
volatile uint8_t g_busy;

void cmd_init(void) {
	proto_init();
}

/* event to the host about the command being run: EV(code, float, ...) */
#define EV(code, ...) proto_event(s_tok, code, sizeof((const float[]){__VA_ARGS__}) / sizeof(float), (const float[]){__VA_ARGS__})
#define EV0(code) proto_event(s_tok, code, 0, NULL)

/* set while CTRL_RUN only holds zero current until the motor is slow enough for CTRL_OFF (run_watchdog()) */
volatile uint8_t g_coasting;
volatile uint8_t g_can_owner;
volatile uint8_t g_stop_reason;

/* from CTRL_RUN: CTRL_OFF puts the zero vector on the motor, which short-brakes it (about lambda / L of
 * current at speed, far above imax). So switch off only below COAST_END_TURNS; faster, keep the current loop
 * at id = iq = 0 and let run_watchdog() switch off once the motor has slowed down. A run command takes over */
static void coast_or_off(void) {
	g_loop = LOOP_CURRENT;
	g_id_ref = 0.0f;
	g_iq_ref = 0.0f;
	if (g_mode == CTRL_RUN && !(fabsf(g_vel) < COAST_END_TURNS)) {
		g_coasting = 1;
		return;
	}
	g_coasting = 0;
	g_mode = CTRL_OFF;
}

/* speed/position mode: brake to (almost) zero speed under the speed loop first (the regen clamp keeps the
 * bus in check, the ISR trip the current). Then ramp the current references to zero and switch off, or coast
 * if the motor is still turning: current mode, or the braking took too long */
static void run_stop(void) {
	if (g_loop != LOOP_CURRENT) {
		g_vel_ref = 0.0f;
		g_loop = LOOP_VEL;
		uint32_t t0 = HAL_GetTick();
		while (fabsf(g_vel) > 0.2f && HAL_GetTick() - t0 < 3000) {
			HAL_Delay(1);
			if (g_trip) return; /* the ISR coasts at zero current, trip_watchdog() ends it */
		}
		g_loop = LOOP_CURRENT; /* currentLoop() leaves g_iq_ref at its last value */
	}
	for (int i = 0; i < 20; i++) {
		g_id_ref *= 0.8f;
		g_iq_ref *= 0.8f;
		HAL_Delay(5);
	}
	if (g_trip) return;
	coast_or_off();
}

static void finish(void) {
	g_hfi_cmd = HFI_OFF;
	if (g_trip && g_mode == CTRL_RUN) { /* coasting at zero current: no braking here, run_watchdog() ends it */
		g_vd_cmd = 0;
		g_vq_cmd = 0;
		return;
	}
	if (g_mode == CTRL_RUN) {
		run_stop(); /* CTRL_OFF, or coasting in CTRL_RUN */
		return;
	}
	for (int i = 0; i < 20; i++) { /* ramp voltage down instead of stepping it */
		g_vd_cmd *= 0.8f;
		g_vq_cmd *= 0.8f;
		HAL_Delay(5);
	}
	g_vd_cmd = 0;
	g_vq_cmd = 0;
	g_mode = CTRL_OFF;
	g_loop = LOOP_CURRENT;
}

/* ------------------------------------------------------------------ helpers */
#define HFI_POL_MIN 0.05f /* polarity contrast below this is a guess, not a measurement */

static st_t hfi_check(void);
static st_t hfi_lock_and_pol(float* before, float* after);

/* sensorless: ST_HFI_LOST once HFI has not tracked for SL_LOST_N cycles. Checked by chk() and the watchdog */
static st_t sl_lost(void) {
	return (g_params.ang_src == ANG_SENSORLESS && g_sl_lost_n >= SL_LOST_N) ? ST_HFI_LOST : ST_OK;
}

/* encoder modes: ST_ENC_LOST once no valid angle has come for ENC_LOST_N cycles. Checked by chk() and the watchdog */
static st_t enc_lost(void) {
	return (g_params.ang_src <= ANG_ENC_OBS && g_enc_lost_n >= ENC_LOST_N) ? ST_ENC_LOST : ST_OK;
}

/* Hall sources: ST_HALL_LOST once the state has been invalid for HALL_LOST_N cycles. A lost HFI angle in Hall + HFI
 * is not a stop: the Halls carry on */
static st_t hall_lost(void) {
	return (g_params.ang_src >= ANG_HALL && g_hall_bad_n >= HALL_LOST_N) ? ST_HALL_LOST : ST_OK;
}

/* returns ST_OK, or why the routine must stop. Serves the protocol in busy mode: READ works, ABORT and
 * STOP abort, anything else gets ST_BUSY */
static st_t chk(void) {
	if (proto_poll(1)) return ST_ABORTED;
	if (g_trip) return g_trip == TRIP_OV ? ST_VBUS_HIGH : ST_OVERCURRENT; /* the ISR trip (overcurrent too) */
	if (checkAndClearBridgeFault() != 0) return ST_BRIDGE_FAULT;
	if (g_vin < g_params.vbus_min) return ST_VBUS_LOW;
	if (g_params.vbus_max > 0.0f && g_vin > g_params.vbus_max) return ST_VBUS_HIGH;
	if (sl_lost() != ST_OK) return ST_HFI_LOST;
	if (enc_lost() != ST_OK) return ST_ENC_LOST;
	if (hall_lost() != ST_OK) return ST_HALL_LOST;
	return ST_OK;
}

static st_t precheck(void) {
	if (g_trip) return ST_TRIPPED;
	if (!(TIM1->BDTR & TIM_BDTR_MOE)) return ST_BRIDGE_OFF;
	if (g_params.rated_current <= 0.0f || g_params.max_current <= 0.0f) return ST_NEED_RATED;
	return chk();
}

/* signed difference of two 21-bit encoder readings */
static int32_t enc_delta(uint32_t now, uint32_t prev) {
	return (int32_t)((now - prev) << 11) >> 11;
}

static uint32_t enc_avg(int n, int ms) {
	uint32_t ref = hMT6835.angle;
	int64_t acc = 0;
	for (int i = 0; i < n; i++) {
		acc += enc_delta(hMT6835.angle, ref);
		HAL_Delay(ms);
	}
	return (ref + (uint32_t)(int32_t)(acc / n)) & 0x1FFFFFu;
}

static float avg_axis(int use_q, int n) {
	float a = 0.0f;
	for (int i = 0; i < n; i++) {
		a += use_q ? g_iq : g_id;
		HAL_Delay(1);
	}
	return a / (float)n;
}

/* raise Vd (20 mV every 5 ms) at theta = 0 until Id reaches i_target; rotor aligns on the way */
static st_t ramp_d_to_current(float i_target, float* v_out) {
	const float vmax = 0.8f * g_vin / SQRT3;
	float v = 0.0f;
	while (g_id < i_target) {
		v += 0.02f;
		if (v > vmax) return ST_VLIMIT;
		g_vd_cmd = v;
		HAL_Delay(5);
		st_t e = chk();
		if (e) return e;
	}
	*v_out = v;
	return ST_OK;
}

/* time between the 75% and 25% crossings of a decaying current is tau * ln(3);
 * measuring a difference makes the fixed one-period delay of the loop cancel out */
static float decay_tau(const volatile float* x, int n, float i0, float fs) {
	const float hi = 0.75f * i0, lo = 0.25f * i0;
	float ta = -1.0f;
	for (int k = 1; k < n; k++) {
		if (ta < 0.0f && x[k] <= hi && x[k - 1] > hi) ta = (float)(k - 1) + (x[k - 1] - hi) / (x[k - 1] - x[k]);
		if (ta >= 0.0f && x[k] <= lo && x[k - 1] > lo) {
			float tb = (float)(k - 1) + (x[k - 1] - lo) / (x[k - 1] - x[k]);
			return (tb - ta) / fs / 1.0986123f;
		}
	}
	return -1.0f;
}

/* ------------------------------------------------------------------ resistance */
static st_t cal_resistance(void) {
	st_t e;
	float v_hi;
	if ((e = precheck())) return e;

	g_mode = CTRL_CAL;
	g_theta_cmd = 0;
	g_vd_cmd = g_vq_cmd = 0;
	if ((e = ramp_d_to_current(0.4f * g_params.rated_current, &v_hi))) return e;
	HAL_Delay(500);
	float i_hi = avg_axis(0, 200);

	float v_lo = 0.5f * v_hi;
	g_vd_cmd = v_lo;
	HAL_Delay(300);
	float i_lo = avg_axis(0, 200);
	if ((e = chk())) return e;
	if (i_hi - i_lo < 0.05f) return ST_DI_SMALL;

	/* two points cancel the fixed dead-time / MOSFET drop */
	float r = (v_hi - v_lo) / (i_hi - i_lo);
	float v_off = v_hi - r * i_hi;
	g_params.rd = r;
	g_params.rq = r; /* surface PM: Rq = Rd. The rotor cannot be held for a q-axis DC test */
	g_params.flags |= PF_R_VALID;
	EV(EV_R, r, v_hi, i_hi, v_lo, i_lo, v_off);
	return ST_OK;
}

/* ------------------------------------------------------------------ inductance */
/* Flux linkage left at each sample of a decay at zero commanded voltage: L di/dt = -(R i + v_off), so
 * psi(i_j) = integral from t_j to the zero crossing of (R i + v_off) dt, and L = psi / i is the secant
 * inductance at i_j: what the observer uses (eta = x - L i). Needs no step timing and no fit. Segments
 * between two positive samples are integrated as exponentials, so the few samples per time constant
 * (L/R is ~3 PWM periods here) do not bias it */
static void print_flux_curve(const volatile float* x, int n, float i0, float r, float v_off) {
	const float h = 1.0f / CTRL_FS_HZ;
	int z = n - 1;
	float tz = (float)(n - 1);
	for (int k = 1; k < n; k++)
		if (x[k] <= 0.0f) {
			z = k;
			tz = (float)(k - 1) + x[k - 1] / (x[k - 1] - x[k]);
			break;
		}
	int j0 = 1;
	while (j0 < z && x[j0] > 0.97f * i0) j0++; /* first sample after the voltage step */
	EV(EV_LCURVE, i0, v_off);
	for (int j = j0; j < z && x[j] > 0.1f * i0; j++) {
		float area = 0.5f * x[z - 1] * (tz - (float)(z - 1)) * h; /* last bit, linear to zero */
		for (int k = j; k < z - 1; k++) {
			float a = x[k], b = x[k + 1];
			area += (a > b && b > 0.0f) ? h * (a - b) / logf(a / b) : 0.5f * h * (a + b);
		}
		float psi = r * area + v_off * (tz - (float)j) * h;
		EV(EV_LCURVE_PT, x[j], psi / x[j]);
	}
}

static float s_cal_arg; /* CAL argument: Ld/Lq test current or HFI map bias, 0 = default (and store) */

static st_t cal_inductance(int use_q) {
	st_t e;
	float v;
	if ((e = precheck())) return e;
	if (!(g_params.flags & PF_R_VALID)) return ST_NEED_R;
	const int report_only = s_cal_arg > 0.0f; /* an explicit current measures a point, it does not calibrate */
	const float amps = report_only ? s_cal_arg : 0.4f * g_params.rated_current;
	if (amps > 0.9f * g_params.max_current) return ST_I_TEST_HIGH;

	g_mode = CTRL_CAL;
	g_theta_cmd = 0;
	g_vd_cmd = g_vq_cmd = 0;
	if ((e = ramp_d_to_current(amps, &v))) return e;
	HAL_Delay(700); /* rotor settles on the d axis */

	if (use_q) {
		EV(EV_HOLD_SHAFT, 5.0f);
		for (int i = 0; i < 500; i++) {
			HAL_Delay(10);
			if ((e = chk())) return e;
		}
		g_vq_cmd = v; /* same voltage, now on the q axis */
		g_vd_cmd = 0;
		HAL_Delay(700);
	}

	float i0 = avg_axis(use_q, 200);
	if ((e = chk())) return e;
	if (i0 < 0.05f) return ST_I_LOW;

	/* drop the voltage to zero and record the decay */
	g_rec_state = 0;
	g_rec_use_q = (uint8_t)use_q;
	g_vd_cmd = 0;
	g_vq_cmd = 0;
	g_rec_state = 1;
	uint32_t t0 = HAL_GetTick();
	while (g_rec_state != 3)
		if (HAL_GetTick() - t0 > 500) return ST_REC_TIMEOUT;

	/* read g_rec in place: the ISR has stopped writing it (state 3) */
	print_flux_curve(g_rec, REC_N, i0, g_params.rd, v - g_params.rd * i0); /* v_off: what R*i0 does not explain */
	float tau = decay_tau(g_rec, REC_N, i0, CTRL_FS_HZ);
	if (tau < 0.0f) return ST_LR_SLOW;
	if (tau * CTRL_FS_HZ < 3.0f) return ST_LR_FAST;

	float L = tau * g_params.rd;
	if (report_only) {
		EV(EV_L, use_q, L, tau, i0, 0.0f);
		return ST_OK;
	}
	if (use_q) {
		g_params.lq = L;
	}
	else {
		g_params.ld = L;
		g_params.lq = L; /* surface PM default, overwrite with 'q' if you measure it */
		g_params.flags |= PF_L_VALID;
	}
	EV(EV_L, use_q, L, tau, i0, 1.0f);
	return ST_OK;
}

static st_t cal_ld(void) { return cal_inductance(0); }
static st_t cal_lq(void) { return cal_inductance(1); }

/* ------------------------------------------------------------------ encoder: direction, pole pairs, offset */
static st_t cal_encoder(void) {
	st_t e;
	float v;
	const int N_REVS = 20;	/* electrical revolutions to sweep */
	const float F_E = 2.0f; /* electrical Hz */
	if ((e = precheck())) return e;

	g_mode = CTRL_CAL;
	g_theta_cmd = 0;
	g_vd_cmd = g_vq_cmd = 0;
	EV0(EV_ALIGN);
	if ((e = ramp_d_to_current(0.5f * g_params.rated_current, &v))) return e;
	HAL_Delay(700);

	EV(EV_SWEEP, N_REVS);
	const uint32_t inc = (uint32_t)(F_E / 1000.0f * 4294967296.0f); /* per 1 ms */
	const int steps = (int)(N_REVS * 1000.0f / F_E);
	uint32_t theta = 0, prev = hMT6835.angle;
	int64_t total = 0; /* unwrapped encoder travel, counts */
	for (int s = 0; s < steps; s++) {
		theta += inc;
		g_theta_cmd = theta;
		HAL_Delay(1);
		uint32_t a = hMT6835.angle;
		total += enc_delta(a, prev);
		prev = a;
		if ((e = chk())) return e;
	}
	g_theta_cmd = 0; /* the sweep is an integer number of turns, so this is not a jump */

	float revs = fabsf((float)total) / 2097152.0f; /* mechanical revolutions */
	if (revs < 0.2f) return ST_NO_FOLLOW;
	int8_t dir = (total >= 0) ? 1 : -1;
	float ppf = (float)N_REVS / revs;
	int pp = (int)(ppf + 0.5f); /* ppf > 0 */
	if (pp < 1 || pp > 255 || fabsf(ppf - (float)pp) > 0.2f) {
		EV(EV_PP_BAD, ppf);
		return ST_PP_NONINT;
	}

	HAL_Delay(1000); /* settle on the d axis at theta = 0 */
	uint32_t mech = enc_avg(200, 2);
	uint32_t m = (mech << 11) * (uint32_t)pp;
	uint32_t ref = (dir > 0) ? (0u - m) : m; /* makes elec_angle() = 0 here: first guess */

	/* one point carries the encoder nonlinearity at that spot and the friction lag of the approach. Average
	 * offset = theta_cmd - encoder over one mechanical turn forward and one back: the nonlinearity
	 * (periodic in the mechanical angle) and the lag (sign follows the direction) both cancel */
	EV0(EV_OFFSET_AVG);
	const int turn_steps = (int)(pp * 1000.0f / F_E);
	const int skip = 100; /* ms after each start/reversal while the lag builds up */
	int64_t acc = 0;
	int n = 0;
	int32_t dmin = INT32_MAX, dmax = INT32_MIN;
	theta = 0;
	for (int pass = 0; pass < 2; pass++) {
		for (int s = 0; s < turn_steps; s++) {
			theta += pass ? (0u - inc) : inc;
			g_theta_cmd = theta;
			HAL_Delay(1);
			if ((e = chk())) return e;
			if (s < skip) continue;
			uint32_t me = (hMT6835.angle << 11) * (uint32_t)pp;
			int32_t d = (int32_t)(theta - ((dir > 0) ? me : 0u - me) - ref); /* this point's offset - first guess */
			acc += d;
			n++;
			if (d < dmin) dmin = d;
			if (d > dmax) dmax = d;
		}
	}
	g_theta_cmd = theta; /* back where the averaging started */
	uint32_t off = ref + (uint32_t)(int32_t)(acc / n);

	g_params.pole_pairs = (uint8_t)pp;
	g_params.enc_dir = dir;
	g_params.elec_offset = off;
	g_params.flags |= PF_ELEC_OFFSET_VALID;
	const float to_deg = 360.0f / 4294967296.0f;
	EV(EV_ENC, dir, pp, ppf, (float)(int32_t)(ref - off) * to_deg, (float)(dmax - dmin) * to_deg);
	return ST_OK;
}

/* ------------------------------------------------------------------ Hall table */
#define HALL_F_E 2.0f		/* electrical Hz of the sweep */
#define HALL_REVS 4			/* electrical revolutions measured each way, after one more to let the lag build up */
#define HALL_SPAN_TOL 25.0f /* deg electrical: each sector must be 60 +- this */

/* forced-angle sweep forward then back, like 'e'. Each crossing is logged at the commanded angle: forward that
 * leads the rotor by the lag, backward it trails by as much, and a sensor's hysteresis is the same both ways,
 * so the mean of the two is where the state really begins. Needs pole_pairs (set by hand without an encoder) */
static st_t cal_hall(void) {
	st_t e;
	float v;
	if ((e = precheck())) return e;

	g_mode = CTRL_CAL;
	g_theta_cmd = 0;
	g_vd_cmd = g_vq_cmd = 0;
	EV0(EV_ALIGN);
	if ((e = ramp_d_to_current(0.5f * g_params.rated_current, &v))) return e;
	HAL_Delay(700);

	EV(EV_SWEEP, HALL_REVS);
	const uint32_t inc = (uint32_t)(HALL_F_E / 1000.0f * 4294967296.0f); /* per 1 ms */
	const int rev_steps = (int)(1000.0f / HALL_F_E);
	uint8_t next[8] = {0};	 /* forward order, from the forward pass */
	uint32_t ref[8];		 /* first crossing seen, per state: the others are averaged as differences from it */
	int64_t acc[2][8] = {0}; /* [forward, backward][state]: crossings where the state begins */
	uint16_t n[2][8] = {0};
	int changes = 0, bad = 0;
	uint32_t theta = 0;
	for (int pass = 0; pass < 2; pass++) {
		uint8_t prev = g_hall_state;
		for (int s = 0; s < (HALL_REVS + 1) * rev_steps; s++) { /* whole revolutions: ends back on theta = 0 */
			theta += pass ? (0u - inc) : inc;
			g_theta_cmd = theta;
			HAL_Delay(1);
			if ((e = chk())) return e;
			const uint8_t st = g_hall_state;
			if (st == prev) continue;
			changes++;
			if (st < 1 || st > 6) bad = 1;
			else if (s >= rev_steps && prev >= 1 && prev <= 6) {
				/* forward, prev -> st crosses where st begins; backward, where prev begins */
				const uint8_t b = pass ? prev : st;
				if (!pass) {
					if (next[prev] && next[prev] != st) return ST_HALL_SEQ;
					next[prev] = st;
				}
				else if (next[st] != prev)
					return ST_HALL_SEQ; /* backward must retrace the forward order */
				if (!n[0][b] && !n[1][b]) ref[b] = theta;
				acc[pass][b] += (int32_t)(theta - ref[b]);
				n[pass][b]++;
			}
			prev = st;
		}
	}
	g_theta_cmd = theta;
	if (changes == 0) return ST_NO_FOLLOW;
	if (bad) return ST_HALL_SEQ;

	const float to_deg = 360.0f / 4294967296.0f;
	uint32_t begin[8] = {0};
	float hyst = 0.0f;
	for (int b = 1; b <= 6; b++) {
		if (!n[0][b] || !n[1][b]) return ST_HALL_SEQ; /* a state never seen both ways */
		const int32_t fw = (int32_t)(acc[0][b] / n[0][b]), bw = (int32_t)(acc[1][b] / n[1][b]);
		begin[b] = ref[b] + (uint32_t)(int32_t)(((int64_t)fw + bw) / 2);
		const float h = fabsf((float)(fw - bw) * to_deg);
		if (h > hyst) hyst = h;
	}
	/* each sector, from where it begins to where its successor begins, must be near 60 deg */
	for (int b = 1; b <= 6; b++) {
		const float span = (float)(begin[next[b]] - begin[b]) * to_deg;
		if (!(fabsf(span - 60.0f) <= HALL_SPAN_TOL)) {
			EV(EV_HALL, begin[1] * to_deg, begin[2] * to_deg, begin[3] * to_deg, begin[4] * to_deg, begin[5] * to_deg, begin[6] * to_deg, hyst);
			return ST_HALL_SPACING;
		}
	}
	for (int b = 1; b <= 6; b++) g_params.hall_edge[b] = (uint16_t)((begin[b] + 0x8000u) >> 16);
	g_params.flags |= PF_HALL_VALID;
	hall_table_apply();
	EV(EV_HALL, begin[1] * to_deg, begin[2] * to_deg, begin[3] * to_deg, begin[4] * to_deg, begin[5] * to_deg, begin[6] * to_deg, hyst);
	return ST_OK;
}

/* bench stand-in for Hall sensors: the MT6835's UVW outputs (wired to J13), at the motor's pole pairs. Register
 * 0x00B: Z_PHASE[7:6] kept, UVW_MUX (bit 5) 0 = UVW on the pins, UVW_OFF (bit 4) 0 = on, UVW_RES[3:0] = pp - 1.
 * Saved to the sensor's EEPROM (~6 s) so it survives a power cycle */
static st_t cal_uvw(void) {
	const int pp = s_cal_arg > 0.0f ? (int)(s_cal_arg + 0.5f) : (int)g_params.pole_pairs;
	if (pp < 1 || pp > 16) return ST_BAD_ARG;
	if (g_mode != CTRL_OFF) return ST_RUNNING;
	MT6835_StopBurst(&hMT6835);
	HAL_Delay(10);
	uint8_t r = 0, rb = 0;
	st_t e = ST_OK;
	if (MT6835_ReadRegister(&hMT6835, MT6835_ADDR_UVW, &r) != HAL_OK)
		e = ST_UVW_FAIL;
	else {
		const uint8_t w = (uint8_t)((r & 0xC0u) | (uint8_t)(pp - 1));
		if (MT6835_WriteRegister(&hMT6835, MT6835_ADDR_UVW, w) != HAL_OK || MT6835_ReadRegister(&hMT6835, MT6835_ADDR_UVW, &rb) != HAL_OK || rb != w)
			e = ST_UVW_FAIL;
		else {
			MT6835_SaveEEPROM(&hMT6835);
			EV(EV_EEPROM_WAIT, 6.0f);
			HAL_Delay(6000);
		}
	}
	MT6835_StartBurst(&hMT6835);
	EV(EV_UVW, pp, rb);
	return e;
}

/* ------------------------------------------------------------------ current PI autotune */
/* pole-zero cancellation: zero at R/L cancels the winding pole, closed loop becomes first order at wc */
static st_t pi_autotune(void) {
	const uint32_t need = PF_R_VALID | PF_L_VALID;
	if ((g_params.flags & need) != need) return ST_NEED_RL;
	if (!(g_params.cur_BW > 0.0f)) return ST_NEED_BW;

	const float bw_max = CTRL_FS_HZ / 20.0f;
	if (g_params.cur_BW > bw_max) {
		EV(EV_BW_CLAMPED, g_params.cur_BW, bw_max);
		g_params.cur_BW = bw_max;
	}
	const float wc = 2.0f * (float)M_PI * g_params.cur_BW;
	g_params.kp_d = g_params.ld * wc;
	g_params.ki_d = g_params.rd * wc;
	g_params.kp_q = g_params.lq * wc;
	g_params.ki_q = g_params.rq * wc;
	g_params.flags |= PF_PI_VALID;
	pi_apply();
	EV(EV_TUNE, g_params.cur_BW, g_params.kp_d, g_params.ki_d, g_params.kp_q, g_params.ki_q);
	return ST_OK;
}

/* ------------------------------------------------------------------ kv (back-EMF constant) */
#define KV_IQ_FRAC 0.25f	 /* spin-up current, fraction of rated */
#define KV_MAX_RPM 10000.0f /* spin-up stops here even if the voltage target is not reached */

/* electrical speed, rad/s, sign matches positive iq; dt from TIM2 (1 us free running) */
typedef struct {
	uint32_t ang, us;
} spd_t;

static void spd_init(spd_t* s) {
	s->ang = hMT6835.angle;
	s->us = TIM2->CNT;
}

static float spd_update(spd_t* s) {
	if (g_params.ang_src == ANG_SENSORLESS) { /* HFI speed (mechanical rpm), electrical rad/s; 0 while not tracking */
		const float r = g_hfi_rpm;
		return (r == r) ? r * (2.0f * (float)M_PI / 60.0f) * (float)g_params.pole_pairs : 0.0f;
	}
	if (g_params.ang_src >= ANG_HALL) /* the Hall speed (mechanical turn/s) */
		return g_vel * (2.0f * (float)M_PI) * (float)g_params.pole_pairs;
	uint32_t a = hMT6835.angle, t = TIM2->CNT;
	int32_t d = enc_delta(a, s->ang);
	float dt = (float)(t - s->us) * 1e-6f;
	s->ang = a;
	s->us = t;
	if (dt <= 0.0f) return 0.0f;
	return (float)(g_params.enc_dir * d) * (2.0f * (float)M_PI / 2097152.0f) * (float)g_params.pole_pairs / dt;
}

/* with iq held at zero, wait until the rotor has almost stopped (switching off while spinning
 * applies a zero vector, which short-brakes the motor) */
static st_t kv_coast(spd_t* s, float w_ref) {
	uint32_t t0 = HAL_GetTick();
	g_iq_ref = 0.0f;
	float w = w_ref;
	while (fabsf(w) > 0.05f * fabsf(w_ref) && HAL_GetTick() - t0 < 15000) {
		HAL_Delay(1); /* encoder delta must stay under half a turn per sample */
		w = spd_update(s);
		if (fabsf(g_id) > g_params.max_current || fabsf(g_iq) > g_params.max_current) return ST_OVERCURRENT;
		if (sl_lost() != ST_OK) return ST_HFI_LOST;
		if (hall_lost() != ST_OK) return ST_HALL_LOST;
	}
	return ST_OK;
}

/* spin up with constant iq, then let it coast with iq = 0: the PI output is then pure back-EMF,
 * vq = lambda * we. Fit vq - R*iq against we; the intercept absorbs dead-time error */
static st_t cal_kv(void) {
	st_t e;
	const int sl = g_params.ang_src == ANG_SENSORLESS;
	if (sl) {
		if ((e = hfi_check())) return e;
		if (g_params.hfi_sal == 0.0f) return ST_NEED_HMAP;
	}
	else if (g_params.ang_src >= ANG_HALL) { /* commutates on the Halls (HFI does not start during a calibration) */
		if (!(g_params.flags & PF_HALL_VALID)) return ST_NEED_HALL;
		if (!(g_params.flags & PF_R_VALID)) return ST_NEED_R;
		if (!(g_params.flags & PF_PI_VALID)) return ST_NEED_TUNE;
	}
	else {
		const uint32_t need = PF_ELEC_OFFSET_VALID | PF_PI_VALID | PF_R_VALID;
		if ((g_params.flags & need) != need) return ST_NEED_ERLT;
	}
	if ((e = precheck())) return e;

	EV(EV_UNLOAD_SHAFT, 2.0f);
	for (int i = 0; i < 200; i++) {
		HAL_Delay(10);
		if ((e = chk())) return e;
	}

	g_mode = CTRL_OFF;
	HAL_Delay(1);
	g_loop = LOOP_CURRENT;
	g_id_ref = 0.0f;
	g_iq_ref = 0.0f;
	g_mode = CTRL_RUN;
	if (sl) {
		/* sensorless: HFI speed needs the polarity right first, or the spin-up runs backwards */
		float before, after;
		if ((e = hfi_lock_and_pol(&before, &after))) return e;
		if (!(fabsf(g_hfi_pol_contrast) >= HFI_POL_MIN)) return ST_HFI_POL_WEAK;
	}
	g_id_ref = 0.0f; /* kv runs at id = 0: the d-axis flux term is part of vq */
	g_iq_ref = 0.0f;

	/* spin-up */
	const float vmax = 0.9f * g_vin / SQRT3;
	const float w_cap = KV_MAX_RPM / 60.0f * 2.0f * (float)M_PI * (float)g_params.pole_pairs;
	spd_t s;
	spd_init(&s);
	g_iq_ref = KV_IQ_FRAC * g_params.rated_current;
	float w = 0.0f, bemf = 0.0f;
	uint32_t t0 = HAL_GetTick();
	while (HAL_GetTick() - t0 < 3000) {
		HAL_Delay(1);
		w = 0.8f * w + 0.2f * spd_update(&s);
		bemf = g_vq_f - g_params.rq * g_iq_f;
		if ((e = chk())) {
			kv_coast(&s, w);
			return e;
		}
		if (bemf > 0.5f * vmax || fabsf(w) > w_cap) break;
	}
	if (bemf < 0.1f * vmax) {
		kv_coast(&s, w);
		return ST_KV_SLOW;
	}
	if (w <= 0.0f) {
		kv_coast(&s, w);
		return ST_KV_BACKWARDS;
	}

	/* coast-down: least squares y = lambda * x + c, x = we, y = vq - R*iq */
	const float w0 = w;
	g_iq_ref = 0.0f;
	HAL_Delay(20); /* current settles to zero */
	spd_init(&s);
	/* running means and centred sums (Welford): stays accurate in float, unlike n*sxx - sx*sx */
	int n = 0;
	float mx = 0.0f, my = 0.0f, cxx = 0.0f, cxy = 0.0f;
	t0 = HAL_GetTick();
	while (w > 0.3f * w0 && HAL_GetTick() - t0 < 5000) {
		HAL_Delay(1);
		w = spd_update(&s);
		float y = g_vq_f - g_params.rq * g_iq_f;
		n++;
		float dx = w - mx;
		mx += dx / (float)n;
		my += (y - my) / (float)n;
		cxx += dx * (w - mx);
		cxy += dx * (y - my);
		if ((e = chk())) {
			kv_coast(&s, w);
			return e;
		}
	}
	if ((e = kv_coast(&s, w0))) return e;

	if (n < 50 || !(cxx > 0.0f)) return ST_KV_FEW;
	float lambda = cxy / cxx; /* V*s/rad electrical, peak phase */
	float c = my - lambda * mx;
	if (lambda <= 0.0f) return ST_KV_SIGN;

	/* line-to-line peak convention: Kt = 1.5 * pp * lambda = 8.27 / kv */
	float kv = 60.0f / (2.0f * (float)M_PI * SQRT3 * (float)g_params.pole_pairs * lambda);
	g_params.kv = kv;
	EV(EV_KV, kv, lambda, (1.5f * (float)g_params.pole_pairs * lambda), c, n, (w0 / (2.0f * (float)M_PI * (float)g_params.pole_pairs) * 60.0f));
	return ST_OK;
}

/* ------------------------------------------------------------------ closed loop current run */
/* checks, then puts the current loop in CTRL_RUN (current mode, zero references) if it is not
 * running yet. Returns ST_OK if running */
/* puts the current loop in CTRL_RUN (zero references) if it is not running yet. Calibrations use it as is */
static st_t cal_enter(void) {
	st_t e = precheck();
	if (e) return e;
	g_coasting = 0; /* a coasting motor is picked up as it is: the current loop never stopped */
	if (g_mode != CTRL_RUN) {
		g_mode = CTRL_OFF; /* currentLoop() holds the integrators at zero */
		HAL_Delay(1);
		g_loop = LOOP_CURRENT;
		g_id_ref = 0.0f;
		g_iq_ref = 0.0f;
		g_mode = CTRL_RUN;
	}
	return ST_OK;
}

/* what the chosen angle source needs before a run (speed/position too when outer). Encoder modes need the
 * electrical offset from 'e'; sensorless needs no encoder at all, but R, L, the PI, kv, the saliency map and hv.
 * Hall: the Hall table and the PI. Hall + HFI: also R, L, the saliency map and hv (the Halls give the polarity) */
static st_t run_prereq(int outer) {
	const uint8_t src = g_params.ang_src;
	if (src >= ANG_HALL) {
		if (!(g_params.flags & PF_HALL_VALID)) return ST_NEED_HALL;
		if (!(g_params.flags & PF_PI_VALID)) return ST_NEED_TUNE;
		if (src == ANG_HALL_HFI) {
			const uint32_t rl = PF_R_VALID | PF_L_VALID;
			if ((g_params.flags & rl) != rl) return ST_NEED_RL;
			if (g_params.hfi_sal == 0.0f) return ST_NEED_HMAP;
			if (!(g_params.hfi_v > 0.0f)) return ST_NEED_HV;
		}
	}
	else if (src == ANG_SENSORLESS) {
		const uint32_t rl = PF_R_VALID | PF_L_VALID;
		if ((g_params.flags & rl) != rl) return ST_NEED_RL;
		if (!(g_params.flags & PF_PI_VALID)) return ST_NEED_TUNE;
		if (!(g_params.kv > 0.0f)) return ST_NEED_KV;
		if (g_params.hfi_sal == 0.0f) return ST_NEED_HMAP;
		if (!(g_params.hfi_v > 0.0f)) return ST_NEED_HV;
		if (!(g_params.hfi_pol_i > 0.0f)) return ST_I_LOW;
	}
	else {
		const uint32_t need = PF_ELEC_OFFSET_VALID | PF_PI_VALID;
		if ((g_params.flags & need) != need) return ST_NEED_ET;
	}
	if (outer && !(g_params.vel_kp > 0.0f)) return ST_NEED_VGAINS;
	if (outer && !(g_params.vel_lim > 0.0f)) return ST_NEED_VLIM;
	return ST_OK;
}

static st_t run_enter(int outer) {
	st_t e = run_prereq(outer);
	return e ? e : cal_enter();
}

/* d-axis current under the speed/position loops: the HFI bias while HFI runs (always when sensorless and in
 * Hall + HFI, with hfi_mode in the encoder modes, never with the Halls alone) */
static float id_bias(void) {
	const uint8_t src = g_params.ang_src;
	const int hfi_on = src == ANG_SENSORLESS || src == ANG_HALL_HFI || (src <= ANG_ENC_OBS && g_params.hfi_mode >= 1);
	return (hfi_on && g_params.hfi_sal != 0.0f) ? g_params.hfi_id : 0.0f;
}

/* speed mode, rpm */
static st_t run_vel(float rpm) {
	if (!(fabsf(rpm) < 1.0e7f)) return ST_BAD_ARG; /* also NaN */
	float lim = g_params.vel_lim * 60.0f;
	if (fabsf(rpm) > lim) {
		rpm = copysignf(lim, rpm);
		EV(EV_RPM_LIMITED, lim);
	}
	st_t e = run_enter(1);
	if (e) return e;
	g_id_ref = id_bias();
	g_vel_ref = rpm / 60.0f; /* reference first, then the switch: the ISR never sees a stale one */
	g_loop = LOOP_VEL;
	return ST_OK;
}

/* position mode, turns from home ('h') */
static st_t run_pos(pos_q32_t t) {
	if (g_params.ang_src == ANG_SENSORLESS) return ST_NEED_ENC_POS; /* turns come from the encoder */
	if (!(g_params.pos_kp > 0.0f)) return ST_NEED_PKP;
	st_t e = run_enter(1);
	if (e) return e;
	g_id_ref = id_bias();

	/* hand over the reference and wait until currentLoop() has latched it, then switch */
	g_pos_ref_new = t;
	g_pos_ff_new = 0.0f;
	g_pos_ff_n_new = 0;
	g_pos_ref_req = 1;
	uint32_t t0 = HAL_GetTick();
	while (g_pos_ref_req)
		if (HAL_GetTick() - t0 > 10) return ST_LOOP_DEAD;
	g_loop = LOOP_POS;
	return ST_OK;
}

/* current mode */
static st_t run_cur(float id, float iq) {
	if (!(fabsf(id) < 1.0e6f && fabsf(iq) < 1.0e6f)) return ST_BAD_ARG; /* also NaN */
	float mag = sqrtf(id * id + iq * iq);
	if (mag > g_params.rated_current) { /* scale the vector, keep its direction */
		float sc = g_params.rated_current / mag;
		id *= sc;
		iq *= sc;
		EV(EV_CUR_LIMITED, g_params.rated_current);
	}

	st_t e = run_enter(0);
	if (e) return e;
	g_loop = LOOP_CURRENT; /* switch first: from here on the ISR no longer writes g_iq_ref */
	g_id_ref = id;
	g_iq_ref = iq;
	return ST_OK;
}

/* ------------------------------------------------------------------ high-frequency injection */
/* docs/design-notes.md (HFI), hfi.c. Both routines commutate on the encoder and use it as the reference */
#define HMAP_N 24		  /* injection angles over 360 deg electrical: 2*phi steps by 30 deg */
#define HMAP_ACC 1000	  /* cycles summed per angle (40 ms) */
#define HFI_SIG_MIN 0.03f /* A per informative cycle, ~2 LSB */

static st_t hfi_check(void) {
	if (g_params.ang_src >= ANG_HALL) { /* the Hall angle is the reference: +-30 deg at standstill */
		if (!(g_params.flags & PF_HALL_VALID)) return ST_NEED_HALL;
		const uint32_t rl = PF_R_VALID | PF_L_VALID;
		if ((g_params.flags & rl) != rl) return ST_NEED_RL;
		if (!(g_params.flags & PF_PI_VALID)) return ST_NEED_TUNE;
	}
	else if (g_params.ang_src == ANG_SENSORLESS) {
		const uint32_t rl = PF_R_VALID | PF_L_VALID; /* no encoder offset needed */
		if ((g_params.flags & rl) != rl) return ST_NEED_RL;
		if (!(g_params.flags & PF_PI_VALID)) return ST_NEED_TUNE;
	}
	else {
		const uint32_t need = PF_ELEC_OFFSET_VALID | PF_PI_VALID | PF_R_VALID | PF_L_VALID;
		if ((g_params.flags & need) != need) return ST_NEED_ERLT;
	}
	if (!(g_params.hfi_v > 0.0f)) return ST_NEED_HV;
	return ST_OK;
}

static st_t wait_ms(int ms) {
	st_t e;
	for (int i = 0; i < ms; i++) {
		HAL_Delay(1);
		if ((e = chk())) return e;
	}
	return ST_OK;
}

/* inject at HMAP_N angles from the encoder d axis with id = bias (current loop on, iq = 0: no torque) and
 * fit 1/L(phi) = a0 + a2 cos 2phi + b2 sin 2phi. a2/a0 = (Lq-Ld)/(Lq+Ld) when the axes are where the
 * encoder says; b2 shows how far they are not (cross-saturation). The PC turns these into Ld, Lq, axis */
static st_t hfi_map_at(float bias, int store) {
	st_t e;
	g_id_ref = bias;
	g_hfi_map_phi = 0;
	g_hfi_cmd = HFI_MAP;
	if ((e = wait_ms(100))) return e;
	const float vts = g_params.hfi_v / CTRL_FS_HZ; /* volt-seconds behind one informative step */
	float a0 = 0.0f, a2 = 0.0f, b2 = 0.0f;
	float c2 = 1.0f, s2 = 0.0f; /* cos, sin of 2 phi, rotated by 30 deg per point */
	for (int k = 0; k < HMAP_N; k++) {
		g_hfi_map_phi = (uint32_t)((float)k * (4294967296.0f / HMAP_N));
		if ((e = wait_ms(20))) return e;
		g_hfi_acc_par = g_hfi_acc_perp = 0.0f;
		g_hfi_acc_n = 0;
		g_hfi_acc_left = HMAP_ACC;
		uint32_t t0 = HAL_GetTick();
		while (g_hfi_acc_left) {
			if (HAL_GetTick() - t0 > 200) return ST_LOOP_DEAD;
			if ((e = wait_ms(1))) return e;
		}
		const float n = (float)g_hfi_acc_n;
		const float par = g_hfi_acc_par / n, perp = g_hfi_acc_perp / n; /* A per informative cycle */
		if (!(par > HFI_SIG_MIN)) return ST_HFI_NO_SIGNAL;				 /* also n = 0 */
		const float y = par / vts;										 /* 1/L along phi */
		a0 += y;
		a2 += y * c2;
		b2 += y * s2;
		EV(EV_HFI_PT, bias, (float)k * (360.0f / HMAP_N), 1.0f / y, perp / par);
		const float c = c2 * 0.8660254f - s2 * 0.5f;
		s2 = s2 * 0.8660254f + c2 * 0.5f;
		c2 = c;
	}
	a0 *= 1.0f / HMAP_N;
	a2 *= 2.0f / HMAP_N;
	b2 *= 2.0f / HMAP_N;
	float sal = sqrtf(a2 * a2 + b2 * b2) / a0;
	/* the low-inductance axis is nearer q than d. Sensorless has no reference angle, so the fit's phase says
	 * nothing about d: it stores the magnitude, taking the low-inductance axis as d (Ld < Lq) */
	if (a2 < 0.0f && g_params.ang_src != ANG_SENSORLESS) sal = -sal;
	if (store) g_params.hfi_sal = sal;
	EV(EV_HFI_FIT, bias, a0, a2, b2, sal, store);
	return ST_OK;
}

/* saliency map (the go/no-go for HFI). arg 0: biases 0, 0.25 and 0.5 x rated and hfi_id, stores
 * hfi_sal from the hfi_id one. arg > 0: that bias only, nothing stored */
static st_t cal_hfi_map(void) {
	st_t e;
	if ((e = hfi_check())) return e;
	const float r = g_params.rated_current;
	float bias[4] = {0.0f, 0.25f * r, 0.5f * r, g_params.hfi_id};
	int n = 4, store = 3;
	if (s_cal_arg > 0.0f) {
		bias[0] = s_cal_arg;
		n = 1;
		store = -1;
	}
	else {
		for (int i = 0; i < 3; i++)
			if (fabsf(bias[i] - g_params.hfi_id) < 1.0e-3f) {
				store = i;
				n = 3;
			}
	}
	for (int i = 0; i < n; i++)
		if (fabsf(bias[i]) > 0.9f * g_params.max_current) return ST_I_TEST_HIGH;
	if ((e = cal_enter())) return e;
	g_iq_ref = 0.0f; /* no torque: the rotor stays where it is, so the map needs no encoder and no held shaft */
	g_hfi_theta0 = 0; /* sensorless: the reference the injection angles are measured from, frozen during the map */
	for (int i = 0; i < n; i++)
		if ((e = hfi_map_at(bias[i], i == store))) return e;
	return ST_OK;
}

/* lock from g_hfi_theta0 (a random angle), then the polarity pulses. before, after: HFI - encoder (deg elec)
 * just before and after the pulses. The pulses decide the polarity by themselves, so this runs without an
 * encoder too (the caller reads the contrast) */
static st_t hfi_lock_and_pol(float* before, float* after) {
	st_t e;
	g_iq_ref = 0.0f;
	g_id_ref = g_params.hfi_id;
	g_hfi_theta0 = DWT->CYCCNT * 2654435761u; /* random start: either polarity can come out */
	g_hfi_cmd = HFI_TRACK;
	if ((e = wait_ms(300))) return e;
	if (!(g_hfi_amp > HFI_SIG_MIN)) return ST_HFI_NO_SIGNAL;
	*before = g_hfi_err_f * 360.0f;

	g_id_ref = 0.0f; /* the pulses start from no current */
	if ((e = wait_ms(30))) return e;
	g_hfi_cmd = HFI_POL;
	uint32_t t0 = HAL_GetTick();
	while (g_hfi_cmd == HFI_POL) { /* ~35 ms; the ISR sets HFI_TRACK when done */
		if (HAL_GetTick() - t0 > 200) return ST_LOOP_DEAD;
		if ((e = wait_ms(1))) return e;
	}
	g_id_ref = g_params.hfi_id;
	if ((e = wait_ms(100))) return e;
	*after = g_hfi_err_f * 360.0f;
	return ST_OK;
}

/* polarity check (run it from many rotor positions, it must come out right every time).
 * Encoder: the lock must be within 45 deg of d or -d, and the flip must land on d. Sensorless: the pulse
 * contrast must be clear enough to trust (the same bar as the sensorless start) */
static st_t cal_hfi_pol(void) {
	st_t e;
	if (g_params.ang_src >= ANG_HALL) return ST_BAD_ARG; /* the Halls give the polarity: nothing to check */
	if ((e = hfi_check())) return e;
	if (g_params.hfi_sal == 0.0f) return ST_NEED_HMAP;
	if (!(g_params.hfi_pol_i > 0.0f)) return ST_I_LOW;
	if (g_params.hfi_pol_i > 0.9f * g_params.max_current || g_params.hfi_id > 0.9f * g_params.max_current) return ST_I_TEST_HIGH;
	if ((e = cal_enter())) return e;
	float before = NAN, after = NAN;
	if ((e = hfi_lock_and_pol(&before, &after))) return e;
	if (g_params.ang_src == ANG_SENSORLESS) {
		EV(EV_HFI_START, g_hfi_pol_contrast, g_hfi_pol_flip, after);
		return fabsf(g_hfi_pol_contrast) >= HFI_POL_MIN ? ST_OK : ST_HFI_POL_WEAK;
	}
	EV(EV_HFI_POL, before, after, g_hfi_pol_flip, g_hfi_pol_contrast);

	float ax = fabsf(before); /* distance from the nearer of d and -d */
	if (ax > 90.0f) ax = 180.0f - ax;
	if (!(ax <= 45.0f)) return ST_HFI_NO_LOCK; /* also NaN */
	if (!(fabsf(after) <= 90.0f)) return ST_HFI_POL_WRONG;
	return ST_OK;
}

/* the ISR trips (g_trip), in any mode: report once, then end the zero-current coast once the motor is slow
 * (or at once when the bridge is off). Returns 1 while a trip is latched */
static int trip_watchdog(void) {
	static uint8_t reported;
	static uint32_t resumes_seen;
	const uint32_t r = g_ov_resumes;
	if (r != resumes_seen) { /* the ISR ended an OV coast by itself (lower: the counter was cleared) */
		if (r > resumes_seen) {
			const float v[2] = {g_vin, (float)r};
			proto_event(0, EV_RESUMED, 2, v);
		}
		resumes_seen = r;
	}
	if (!g_trip) {
		reported = 0;
		return 0;
	}
	if (!reported) {
		reported = 1;
		g_stop_reason = (uint8_t)(g_trip == TRIP_OV ? ST_VBUS_HIGH : ST_OVERCURRENT);
		const float v[3] = {(float)g_stop_reason, g_id, g_iq};
		proto_event(0, EV_STOPPED, 3, v); /* unsolicited: tok 0 */
	}
	if (g_mode == CTRL_RUN && !g_trip_moe && !(fabsf(g_vel) < COAST_END_TURNS)) return 1; /* still coasting */
	g_coasting = 0;
	if (g_mode != CTRL_OFF) {
		g_mode = CTRL_OFF;
		g_hfi_cmd = HFI_OFF;
		g_loop = LOOP_CURRENT;
		g_id_ref = 0.0f;
		g_iq_ref = 0.0f;
	}
	return 1;
}

/* called from cmd_poll(): the trips, then while running: stop on fault, low bus, overspeed or lost HFI, and
 * end a coast (g_coasting) once the motor is slow. A stop never short-brakes: it coasts at zero current
 * (coast_or_off()), or switches the bridge off when the angle is gone or the bridge already is */
static void run_watchdog(void) {
	if (trip_watchdog()) return;
	if (g_mode != CTRL_RUN) {
		g_coasting = 0;
		return;
	}
	st_t e = ST_OK;
	if (g_hfi_start_done) { /* report the sensorless start (its polarity pulses); refuse to run on a coin toss */
		g_hfi_start_done = 0;
		const float s[3] = {g_hfi_pol_contrast, g_hfi_pol_flip, g_hfi_err * 360.0f};
		proto_event(0, EV_HFI_START, 3, s);
		if (!(fabsf(g_hfi_pol_contrast) >= HFI_POL_MIN)) e = ST_HFI_POL_WEAK;
	}
	if (e) /* the start verdict above */
		;
	else if (!(TIM1->BDTR & TIM_BDTR_MOE) || checkAndClearBridgeFault() != 0) e = ST_BRIDGE_FAULT;
	else if (g_vin < g_params.vbus_min) e = ST_VBUS_LOW;
	else if (g_params.vel_lim > 0.0f && fabsf(g_vel) > 1.5f * g_params.vel_lim) e = ST_OVERSPEED;
	else if (sl_lost() != ST_OK) e = ST_HFI_LOST; /* sensorless: HFI stopped tracking */
	else if (enc_lost() != ST_OK) e = ST_ENC_LOST; /* encoder modes: no valid angle */
	else if (hall_lost() != ST_OK) e = ST_HALL_LOST; /* Hall sources: sensor gone */
	if (g_coasting && (e == ST_VBUS_LOW || e == ST_OVERSPEED)) e = ST_OK; /* the coast already is the reaction */
	if (!e) {
		if (g_coasting && fabsf(g_vel) < COAST_END_TURNS) {
			g_coasting = 0;
			g_mode = CTRL_OFF;
		}
		return;
	}

	/* immediately, no ramp: the outer loops are not in control any more */
	g_hfi_cmd = HFI_OFF;
	if (e == ST_HFI_LOST || e == ST_ENC_LOST || e == ST_HALL_LOST) TIM1->BDTR &= ~TIM_BDTR_MOE; /* no angle for the current loop: bridge off, it coasts */
	if (TIM1->BDTR & TIM_BDTR_MOE)
		coast_or_off();
	else { /* off already: checkAndClearBridgeFault() re-arms it once the zero vector is harmless */
		g_loop = LOOP_CURRENT;
		g_id_ref = 0.0f;
		g_iq_ref = 0.0f;
		g_coasting = 0;
		g_mode = CTRL_OFF;
	}
	g_stop_reason = (uint8_t)e;
	const float v[5] = {(float)e, g_id, g_iq, g_vin, e == ST_BRIDGE_FAULT ? (float)bridge_fault_info() : 0.0f};
	proto_event(0, EV_STOPPED, 5, v); /* unsolicited: tok 0 */
}

/* the motor params (pages 62-63) and/or the CANopen comm record (pages 60-61, co_store.c). CO_RQ_* bits */
static HAL_StatusTypeDef save_records(uint8_t what) {
	disableBridge(); /* the CPU stalls ~20 ms during each flash erase */
	const uint32_t late0 = hMT6835.late; /* the stall delays the encoder read (vector table in flash): not a timing fault */
	HAL_StatusTypeDef st = HAL_OK;
	if (what & CO_RQ_SAVE_APP) st = params_save();
	if (st == HAL_OK) st = co_store_save(what); /* any request: the record holds the CAN settings and both OD parts */
	hMT6835.late = late0;
	initBridge(); /* same bring-up as at boot: CCRs, short-circuit reference, gate-driver fault clear */
	return st;
}

static HAL_StatusTypeDef save_params(void) {
	return save_records(CO_RQ_SAVE_APP | CO_RQ_SAVE_COMM);
}

void reset_encoder(void) {
	g_mode = CTRL_OFF;
	MT6835_StopBurst(&hMT6835);
	HAL_Delay(10);
	MT6835_WriteRegister(&hMT6835, MT6835_ADDR_ROT, 0x07);
	MT6835_ResetZero(&hMT6835);
	MT6835_SaveEEPROM(&hMT6835);
	EV(EV_EEPROM_WAIT, 6.0f);
	HAL_Delay(6000);

	MT6835_StartBurst(&hMT6835);
}

static st_t cal_zero(void) {
	reset_encoder();
	return ST_OK;
}

/* zero offsets of the three current-sense channels, at zero volts with the rotor still; stored on success */
static st_t cal_cur_off(void) {
	st_t e = precheck();
	if (e) return e;
	int16_t m[3], sp[3];
	const int ok = cs_measure_offsets(m, sp);
	if (ok) {
		memcpy(g_params.cur_off, m, sizeof m);
		g_params.flags |= PF_CUR_CAL_VALID;
		cs_set_offsets(m);
		g_cs_off_src = CS_OFF_SRC_STORED;
	}
	EV(EV_CUR_OFF, m[0], m[1], m[2], sp[0], sp[1], sp[2], ok);
	return ok ? ST_OK : ST_CUR_OFF_BAD;
}

static void home(void) {
	if (g_mode == CTRL_RUN) run_stop(); /* a moving zero would kick the position loop */
	g_home_pos = 0;
	g_home_req = 1; /* currentLoop() takes the present position as 0 */
}

/* ------------------------------------------------------------------ CiA 402 drive profile (co402.c, main loop) */
/* ready to run now, without blocking and without proto_poll() (unlike chk()) */
static st_t ready_check(void) {
	if (g_trip) return ST_TRIPPED;
	if (!(TIM1->BDTR & TIM_BDTR_MOE)) return ST_BRIDGE_OFF;
	if (g_params.rated_current <= 0.0f || g_params.max_current <= 0.0f) return ST_NEED_RATED;
	if (g_vin < g_params.vbus_min) return ST_VBUS_LOW;
	if (g_params.vbus_max > 0.0f && g_vin > g_params.vbus_max) return ST_VBUS_HIGH;
	return ST_OK;
}

st_t drive_check(loop_t loop) {
	if (g_busy) return ST_BUSY;
	if (g_mode != CTRL_OFF && !g_can_owner) return ST_RUNNING; /* the USB side has the motor */
	if (loop == LOOP_POS && g_params.ang_src == ANG_SENSORLESS) return ST_NEED_ENC_POS;
	if (loop == LOOP_POS && !(g_params.pos_kp > 0.0f)) return ST_NEED_PKP;
	const st_t e = run_prereq(loop != LOOP_CURRENT);
	return e ? e : ready_check();
}

st_t drive_enter(loop_t loop) {
	const st_t e = drive_check(loop);
	if (e) return e;
	g_coasting = 0; /* a coasting motor is picked up as it is */
	if (g_mode != CTRL_RUN) {
		g_mode = CTRL_OFF; /* currentLoop() holds the integrators at zero */
		HAL_Delay(1);
		g_loop = LOOP_CURRENT;
		g_id_ref = 0.0f;
		g_iq_ref = 0.0f;
		g_mode = CTRL_RUN;
	}
	g_id_ref = id_bias();
	if (loop == LOOP_VEL) {
		g_vel_ref = g_vel; /* reference first, then the switch: bumpless from the present speed */
		g_loop = LOOP_VEL;
	}
	else if (loop == LOOP_POS) {
		g_pos_here_req = 1; /* the position reference becomes the present position, then the switch */
		const uint32_t t0 = HAL_GetTick();
		while (g_pos_here_req)
			if (HAL_GetTick() - t0 > 10) return ST_LOOP_DEAD;
		g_loop = LOOP_POS;
	}
	else
		g_loop = LOOP_CURRENT; /* iq stays where it is: bumpless */
	return ST_OK;
}

void drive_coast(void) {
	coast_or_off();
}

int drive_trip_reset(void) {
	if (!g_trip) return 1;
	if (g_mode != CTRL_OFF) return 0; /* still coasting */
	g_trip_moe = 0;
	g_trip = TRIP_NONE;
	return 1;
}

/* ------------------------------------------------------------------ binary commands (proto.c) */
static st_t (*const k_cal[CAL_N])(void) = {
	[CAL_ENC] = cal_encoder,
	[CAL_R] = cal_resistance,
	[CAL_LD] = cal_ld,
	[CAL_LQ] = cal_lq,
	[CAL_TUNE] = pi_autotune,
	[CAL_KV] = cal_kv,
	[CAL_ZERO] = cal_zero,
	[CAL_HFI_MAP] = cal_hfi_map,
	[CAL_HFI_POL] = cal_hfi_pol,
	[CAL_CUR_OFF] = cal_cur_off,
	[CAL_HALL] = cal_hall,
	[CAL_UVW] = cal_uvw,
};

/* ACK first, then the routine blocks here (chk() keeps serving READ/ABORT/STOP), EV_DONE at the end */
static void cal_message(const uint8_t* p, uint8_t len) {
	float arg = 0.0f;
	const uint8_t which = len == 5 ? p[0] : 0;
	if (len == 5) memcpy(&arg, p + 1, 4);
	st_t e = ST_OK;
	if (which < 1 || which >= CAL_N || !(arg >= 0.0f && arg < 1000.0f)) e = ST_BAD_ARG;
	else if (g_mode == CTRL_RUN) e = ST_RUNNING;
	proto_reply_st(MSG_CAL, s_tok, e);
	if (e) return;
	g_busy = which;
	s_cal_arg = arg;
	e = k_cal[which]();
	finish();
	g_busy = 0;
	EV(EV_DONE, e, which);
}

void cmd_message(uint8_t type, uint8_t tok, const uint8_t* p, uint8_t len) {
	float a, b;
	pos_q32_t t;
	st_t e = ST_OK;
	s_tok = tok; /* EV() reports to this request */
	if (g_trip && type != MSG_STOP && type != MSG_REBOOT) { /* SAVE would re-arm the bridge (initBridge()) */
		proto_reply_st(type, tok, ST_TRIPPED);
		s_tok = 0;
		return;
	}
	if (g_can_owner && (type == MSG_CAL || type == MSG_SAVE || type == MSG_RUN_CUR || type == MSG_RUN_VEL ||
						type == MSG_RUN_POS || type == MSG_HOME || type == MSG_REBOOT)) {
		proto_reply_st(type, tok, ST_CAN_ACTIVE); /* STOP still works: a local stop, the CANopen side goes to fault */
		s_tok = 0;
		return;
	}
	switch (type) {
		case MSG_CAL:
			cal_message(p, len);
			s_tok = 0;
			return;
		case MSG_SAVE:
			if (g_mode != CTRL_OFF) e = ST_RUNNING; /* 'w' would cut the bridge under a running motor */
			else if (save_params() != HAL_OK) e = ST_SAVE_FAILED;
			break;
		case MSG_RUN_CUR:
			if (len != 8) e = ST_BAD_ARG;
			else {
				memcpy(&a, p, 4);
				memcpy(&b, p + 4, 4);
				e = run_cur(a, b);
			}
			break;
		case MSG_RUN_VEL:
			if (len != 4) e = ST_BAD_ARG;
			else {
				memcpy(&a, p, 4);
				e = run_vel(a);
			}
			break;
		case MSG_RUN_POS:
			if (len != 8) e = ST_BAD_ARG;
			else {
				memcpy(&t, p, 8);
				e = run_pos(t);
			}
			break;
		case MSG_HOME:
			home();
			finish();
			break;
		case MSG_STOP:
			if (g_trip) { /* reset the trip, once the coast has ended; the bridge re-arms if it was switched off */
				if (!drive_trip_reset()) e = ST_TRIPPED;
				break;
			}
			if (g_can_owner) { /* take the motor from the CANopen master first: the tick stops writing references */
				g_can_owner = 0;
				g_stop_reason = ST_ABORTED;
			}
			finish(); /* ramps down and switches off */
			break;
		case MSG_REBOOT:
			if (len != 1 || p[0] > 1) e = ST_BAD_ARG;
			else if (g_mode != CTRL_OFF) e = ST_RUNNING; /* a reset cuts the bridge under a running motor */
			proto_reply_st(type, tok, e);
			s_tok = 0;
			if (e == ST_OK) {
				disableBridge();
				boot_reboot(p[0]);
			}
			return;
		default:
			e = ST_BAD_TYPE;
			break;
	}
	/* a run command that failed stops */
	if (e && (type == MSG_RUN_CUR || type == MSG_RUN_VEL || type == MSG_RUN_POS)) finish();
	proto_reply_st(type, tok, e);
	s_tok = 0;
}

/* a sensor that lost its supply (or was unplugged) wakes up out of burst mode and never streams again by itself:
 * with the motor off, send the burst command again every ENC_RETRY_MS until angles come. The turns counter
 * carries on, but the rotor may have moved meanwhile: home again for an exact position */
#define ENC_RETRY_MS 100
static void enc_recover(void) {
	static uint32_t last;
	if (g_mode != CTRL_OFF || g_enc_lost_n < ENC_LOST_N) return; /* also after a StartBurst that timed out */
	if (g_params.ang_src >= ANG_HALL) return;					  /* the Hall sources do without one */
	const uint32_t now = HAL_GetTick();
	if (now - last < ENC_RETRY_MS) return;
	last = now;
	MT6835_StopBurst(&hMT6835);
	MT6835_StartBurst(&hMT6835);
}

void cmd_poll(void) {
	run_watchdog();
	enc_recover();
	proto_poll(0); /* frames -> objdict / cmd_message() */
	co_app_poll(); /* CANopen communication / node resets */
	co402_poll();  /* CiA 402 state machine */
	const uint8_t rq = co_store_take_request(); /* 0x1010 / 0x1011 over SDO, acknowledged already */
	if (rq && g_mode == CTRL_OFF && !g_trip) save_records(rq); /* a trip: initBridge() would re-arm it */
}
