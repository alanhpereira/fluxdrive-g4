#include "motorCtrl.h"

#include <math.h>
#include <string.h>

#include "i2c.h"
#include "usart.h"
#include "cordicMgr.h"
#include "mt6835.h"
#include "params.h"
#include "calib.h"
#include "pi_ctrl.h"

#define SQRT3_2 0.8660254f
#define INV_SQRT3 0.57735027f
#define POLE_PAIRS 7
#define ELEC_OFFSET 0

static uint16_t seq;
typedef struct {
	uint8_t sync[2];  // 0xA5, 0x5A
	uint16_t seq;
	float v[16];
	uint32_t sum;  // frame_sum() of the 17 words before it: lets the receiver confirm a frame by itself
} Frame;		   // offsets: seq at 2, v at 4, sum at 68, so natural alignment, no padding
_Static_assert(sizeof(Frame) == 72, "frame size");

/* rotate-and-add over the first 17 little-endian words; must match software/plotter.py */
static uint32_t frame_sum(const Frame* f) {
	uint32_t s = 0x5A5AA5A5u;
	for (int i = 0; i < 17; i++) {
		uint32_t w;
		memcpy(&w, (const uint8_t*)f + 4 * i, 4);
		s = ((s << 1) | (s >> 31)) + w;
	}
	return s;
}

Frame telemetry;

static float dc_float;
static float center_f;

volatile float g_iq, g_id;
volatile float g_vq_f, g_iq_f;
volatile float g_vel, g_pos, g_rpm;
volatile uint8_t g_home_req;
volatile uint32_t g_loop_cyc, g_loop_cyc_max; /* currentLoop() run time, CPU cycles */

volatile uint8_t control_enabled = 0;

static int16_t cycles_since_last_frame = 0;

void disableMotor() {
	disableCS();
	disableBridge();
}

HAL_StatusTypeDef initMotor() {
	HAL_StatusTypeDef status;
	cordic_init();
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /* DWT cycle counter, for the loop timing */
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	status = initCS();
	if (status != HAL_OK) {
		disableMotor();
		return status;
	}
	status = initBridge();
	if (status != HAL_OK) {
		disableMotor();
		return status;
	}
	dc_float = g_max_duty_cycle * INV_SQRT3;
	center_f = (float)g_half_duty_cycle + 0.5f;
	MT6835_StartBurst(&hMT6835);
	telemetry.sync[0] = 0xA5;
	telemetry.sync[1] = 0x5A;
	control_enabled = 1;

	return HAL_OK;
}
static inline void clarke(float ia, float ib, float ic, float* al, float* be) {
	*al = (2.0f * ia - ib - ic) * (1.0f / 3.0f);
	*be = (ib - ic) * INV_SQRT3;
}

static inline void park(float al, float be, float s, float c, float* d, float* q) {
	*d = al * c + be * s;
	*q = -al * s + be * c;
}

static inline void ipark(float d, float q, float s, float c, float* al, float* be) {
	*al = d * c - q * s;
	*be = d * s + q * c;
}
/* returns 0 when nothing was written (the previous duties stay on) */
int SVPWM(float alpha, float beta) {
	if (HAL_GetTick() < g_lastFaultCleared + FAULT_CLEAR_MIN_TIME) return 0;
	float base = -0.5f * alpha;
	float height = SQRT3_2 * beta;
	// calculate raw voltages
	float va = alpha;
	float vb = base + height;
	float vc = base - height;
	float min_v = va > vb ? (vb > vc ? vc : vb) : (va > vc ? vc : va);
	float max_v = va < vb ? (vb < vc ? vc : vb) : (va < vc ? vc : va);
	float off = (min_v + max_v) * -0.5f;
	int32_t out_a = (int32_t)((va + off) * dc_float + center_f);
	int32_t out_b = (int32_t)((vb + off) * dc_float + center_f);
	int32_t out_c = (int32_t)((vc + off) * dc_float + center_f);
	out_a = out_a < 0 ? 0 : out_a > g_max_duty_cycle ? g_max_duty_cycle
													 : out_a;
	out_b = out_b < 0 ? 0 : out_b > g_max_duty_cycle ? g_max_duty_cycle
													 : out_b;
	out_c = out_c < 0 ? 0 : out_c > g_max_duty_cycle ? g_max_duty_cycle
													 : out_c;
	setDuty(out_a, out_b, out_c);
	return 1;
}

/* ------------------------------------------------------------------ position / speed estimate */
/* PLL: critically damped, kp = 2*wb, ki = wb^2. 2000 rad/s (~320 Hz) leaves room for a speed loop of ~100 Hz */
#define PLL_BW 2000.0f
#define PLL_KP (2.0f * PLL_BW)
#define PLL_KI_TS (PLL_BW * PLL_BW / CTRL_FS_HZ)

static int64_t s_home;		/* counts, motor direction */
static pos_q32_t s_pos;		/* from home, motor direction */
static pos_q32_t s_pos_ref; /* latched from g_pos_ref_new */
static uint32_t s_pll_ph;	/* Q32 = one turn, motor direction */
static float s_pll_vel;		/* turn/s */
static uint8_t s_pll_init;
static float s_vel;				  /* turn/s, encoder PLL blended with the observer PLL: what the loops use */
static float s_vel_ref_eff = NAN; /* speed reference actually fed to the speed PI, for telemetry */

/* age_s: how long ago the sensor sampled ang. fresh = 0 when there was no new sample (stream stalled,
 * CRC): the PLL then only predicts */
static void motion_update(uint32_t ang, int fresh, float age_s) {
	/* turns and angle are only written by MT6835_Sample(), earlier in this same ISR */
	int64_t raw = (int64_t)hMT6835.turns * 2097152 + (int64_t)ang;
	int64_t p = (g_params.enc_dir > 0) ? raw : -raw;
	if (g_home_req) {
		s_home = p;
		g_home_req = 0;
	}
	if (g_pos_ref_req) {
		s_pos_ref = g_pos_ref_new;
		g_pos_ref_req = 0;
	}
	s_pos = (p - s_home) * 2048; /* counts -> Q32.32 */
	g_pos = (float)(int32_t)(s_pos >> 32) + (float)(uint32_t)s_pos * (float)(1.0 / POS_Q32_ONE);

	uint32_t ph = (uint32_t)p << 11; /* low 21 bits as a Q32 phase */
	if (!s_pll_init) {
		if (!fresh) return; /* no encoder frame yet: nothing to lock onto */
		s_pll_ph = ph;
		s_pll_vel = 0.0f;
		s_pll_init = 1;
	}
	/* compare with where the PLL says the rotor was when the sensor sampled, not where it is now */
	uint32_t pred = s_pll_ph - (uint32_t)(int32_t)(s_pll_vel * age_s * 4294967296.0f);
	float err = fresh ? (float)(int32_t)(ph - pred) * (1.0f / 4294967296.0f) : 0.0f; /* turns, wraps correctly */
	s_pll_vel += PLL_KI_TS * err;
	s_pll_ph += (uint32_t)(int32_t)((s_pll_vel + PLL_KP * err) * (4294967296.0f / CTRL_FS_HZ));
}

/* ------------------------------------------------------------------ speed / position loops */
/* The speed reference is slewed at g_params.acc_lim (rpm/s, 0 = off). Measured: ~22000 rpm/s per amp, so
 * 60000 rpm/s is ~3 A plus friction. That keeps the observer's load-dependent flux error small (it drops
 * to ~0.8 of lambda at 8 A) and stays clear of the ~8 A where the encoder mount slips */

/* in CTRL_RUN with g_loop != LOOP_CURRENT: writes g_iq_ref at OUTER_FS_HZ */
static void outer_loops(ctrl_mode_t mode) {
	static loop_t prev = LOOP_CURRENT;
	static int div;
	static float vramp; /* slewed speed reference, turn/s */
	loop_t lp = (mode == CTRL_RUN) ? g_loop : LOOP_CURRENT;

	if (lp == LOOP_CURRENT) {
		pi_reset(&g_pi_vel);
		s_vel_ref_eff = NAN;
		div = OUTER_DIV; /* run on the first cycle after switching in */
	}
	else {
		if (prev == LOOP_CURRENT) {
			g_pi_vel.integ = g_iq_ref; /* bumpless: carry on from the present current */
			vramp = s_vel;			   /* and ramp from the present speed */
		}
		if (++div >= OUTER_DIV) {
			div = 0;
			float lim = g_params.vel_lim;
			float vref = g_vel_ref;
			if (lp == LOOP_POS) {
				/* exact integer difference; only the (small) error becomes a float */
				float err = (float)(s_pos_ref - s_pos) * (float)(1.0 / POS_Q32_ONE);
				vref = g_params.pos_kp * err;
			}
			if (vref > lim)
				vref = lim;
			else if (vref < -lim)
				vref = -lim;
			const float dv = g_params.acc_lim * (1.0f / 60.0f) * (float)OUTER_DIV / CTRL_FS_HZ;
			if (!(dv > 0.0f))
				vramp = vref;
			else if (vref > vramp + dv)
				vramp += dv;
			else if (vref < vramp - dv)
				vramp -= dv;
			else
				vramp = vref;
			s_vel_ref_eff = vramp;
			g_iq_ref = pi_run(&g_pi_vel, vramp - s_vel, g_params.rated_current);
		}
	}
	prev = lp;
}

/* ------------------------------------------------------------------ angle latency compensation */
/* The encoder age relative to the current sample comes from MT6835_Sample() every cycle (~4 us: the
 * last group of the read that ends just before the ADC). ENC_EXTRA_S is for delay the sensor adds
 * itself (its filter, BW register), unknown so far */
#define ENC_EXTRA_S 0.0f
/* the duty written now loads at the next bottom and is centred on the following top */
#define OUT_DELAY_S (1.0f / CTRL_FS_HZ + 0.44e-6f) /* ~40.4 us, current sample -> centre of the applied voltage */
#define ANGLE_COMP 1							   /* 0: use the raw encoder angle for both transforms */

/* ------------------------------------------------------------------ back-EMF (flux) observer */
/* Nonlinear flux observer (Ortega et al. 2011, the one VESC/ODrive use), stationary frame:
 *   x' = v - R*i + k * eta * (lambda^2 - |eta|^2),   eta = x - L*i = rotor flux,  k = gain / (2 lambda^2)
 * The angle of eta is the electrical angle at the current sample, the same instant theta_m refers to.
 * With g_params.obs_blend it replaces the encoder for commutation and speed above obs_lo_v of back-EMF,
 * with a linear blend up to obs_hi_v. Position always comes from the encoder. Tuning (params, 's'):
 * obs_gain (rad/s), obs_flux_tol, obs_err_max (deg electrical: leaves room for the encoder's own error,
 * nonlinearity ~+-5 and a mount slip of ~13 deg have been seen) */
#define OBS_LOCK_N 250		/* cycles (10 ms) both checks must hold before it gets any weight */
/* Once locked, a failing check only drops the observer when it persists: each check keeps a leaky count
 * (+1 per failing cycle, -1 per passing one) and drops at these levels. The flux check is the observer's
 * own (and the only one left without an encoder); the encoder check is slow on purpose, because the
 * encoder is not always the one that is wrong (mount slip, nonlinearity) */
#define OBS_FLUX_DROP_N 125 /* ~5 ms net of flux out of tolerance */
#define OBS_ENC_DROP_N 1250 /* ~50 ms net of disagreement with the encoder */

static float s_obs_xa, s_obs_xb;	/* stator flux estimate, Wb */
static float s_obs_ia, s_obs_ib;	/* currents at the previous sample, A */
static float s_obs_va, s_obs_vb;	/* last voltage written to the bridge, V */
static float s_obs_va1, s_obs_vb1;	/* the one written before it */
static float s_obs_kv = -1.0f;		/* kv and gain the constants below were computed from */
static float s_obs_gain = -1.0f;
static float s_obs_lambda, s_obs_k; /* Wb peak phase (as cal_kv()), correction gain */
static uint16_t s_obs_good;			/* consecutive cycles both checks passed (up to OBS_LOCK_N) */
static uint16_t s_obs_flux_bad;		/* leaky counts of failing cycles, see OBS_FLUX_DROP_N */
static uint16_t s_obs_enc_bad;
static float s_obs_ratio = NAN;		/* |eta| / lambda, telemetry */
static float s_obs_err = NAN;		/* observer - encoder, turns electrical, telemetry */
static float s_obs_w;				/* blend weight, telemetry */

/* one step with the currents sampled now; returns 0 (and no angle) without R, L and kv */
static int obs_update(float i_a, float i_b, uint32_t* theta) {
	const uint32_t need = PF_R_VALID | PF_L_VALID;
	if ((g_params.flags & need) != need || !(g_params.kv > 0.0f)) {
		s_obs_ratio = NAN;
		return 0;
	}
	if (g_params.kv != s_obs_kv) {
		s_obs_kv = g_params.kv;
		s_obs_lambda = 60.0f / (2.0f * (float)M_PI * SQRT3 * (float)g_params.pole_pairs * s_obs_kv);
		s_obs_xa = s_obs_xb = 0.0f;
		s_obs_good = 0;
		s_obs_gain = -1.0f;
	}
	if (g_params.obs_gain != s_obs_gain) {
		s_obs_gain = g_params.obs_gain;
		s_obs_k = 0.5f * s_obs_gain / (s_obs_lambda * s_obs_lambda);
	}
	float r = 0.5f * (g_params.rd + g_params.rq);
	float l = 0.5f * (g_params.ld + g_params.lq);
	/* the interval since the previous sample ran on the previous command for about half of it and on the
	 * one before for the other half (a duty loads at the bottom and the samples are near the top) */
	float va = 0.5f * (s_obs_va + s_obs_va1) - r * 0.5f * (i_a + s_obs_ia);
	float vb = 0.5f * (s_obs_vb + s_obs_vb1) - r * 0.5f * (i_b + s_obs_ib);
	s_obs_ia = i_a;
	s_obs_ib = i_b;

	float ea = s_obs_xa - l * i_a, eb = s_obs_xb - l * i_b;
	float c = s_obs_k * (s_obs_lambda * s_obs_lambda - (ea * ea + eb * eb));
	s_obs_xa += (va + c * ea) * (1.0f / CTRL_FS_HZ);
	s_obs_xb += (vb + c * eb) * (1.0f / CTRL_FS_HZ);
	ea = s_obs_xa - l * i_a;
	eb = s_obs_xb - l * i_b;
	s_obs_ratio = sqrtf(ea * ea + eb * eb) / s_obs_lambda;
	if (!(s_obs_ratio < 3.0f)) {
		/* far off (or NaN) the cubic correction overshoots in discrete time and runs away: restart from eta = 0 */
		s_obs_xa = l * i_a;
		s_obs_xb = l * i_b;
	}
	*theta = atan2_q32(eb, ea);
	return 1;
}

/* weight of the observer angle against the encoder one. diff = observer - encoder (Q32 electrical),
 * we = electrical speed in Q32 per second */
static float obs_weight(int valid, uint32_t diff, float we) {
	if (!valid) {
		s_obs_good = 0;
		s_obs_flux_bad = s_obs_enc_bad = 0;
		s_obs_err = NAN;
		s_obs_w = 0.0f;
		return 0.0f;
	}
	float d = (float)(int32_t)diff * (1.0f / 4294967296.0f);
	s_obs_err = d;
	int flux_ok = fabsf(s_obs_ratio - 1.0f) < g_params.obs_flux_tol;
	int enc_ok = fabsf(d) * 360.0f < g_params.obs_err_max;
	if (flux_ok) {
		if (s_obs_flux_bad) s_obs_flux_bad--;
	}
	else if (s_obs_flux_bad < UINT16_MAX)
		s_obs_flux_bad++;
	if (enc_ok) {
		if (s_obs_enc_bad) s_obs_enc_bad--;
	}
	else if (s_obs_enc_bad < UINT16_MAX)
		s_obs_enc_bad++;

	if (!(TIM1->BDTR & TIM_BDTR_MOE) || s_obs_flux_bad >= OBS_FLUX_DROP_N || s_obs_enc_bad >= OBS_ENC_DROP_N) {
		/* with MOE off the terminal voltage is not what was commanded: drop at once */
		s_obs_good = 0;
		s_obs_flux_bad = s_obs_enc_bad = 0; /* relocking needs OBS_LOCK_N clean cycles anyway */
	}
	else if (s_obs_good < OBS_LOCK_N) {
		s_obs_good = (flux_ok && enc_ok) ? s_obs_good + 1 : 0; /* locking: strict */
	}
	float w = 0.0f;
	if (g_params.obs_blend && s_obs_good >= OBS_LOCK_N) {
		float emf = s_obs_lambda * fabsf(we) * (2.0f * (float)M_PI / 4294967296.0f); /* V peak */
		float span = g_params.obs_hi_v - g_params.obs_lo_v;
		if (span > 0.0f)
			w = (emf - g_params.obs_lo_v) / span;
		else
			w = (emf >= g_params.obs_lo_v) ? 1.0f : 0.0f;
		w = w < 0.0f ? 0.0f : w > 1.0f ? 1.0f
									   : w;
	}
	s_obs_w = w;
	return w;
}

/* Speed from the observer alone: a PLL (same bandwidth as the encoder one) on the observer's electrical
 * angle. It needs nothing from the encoder except the starting speed, so it is what an encoderless mode
 * would run on. Blended into the speed with the same weight as the angle */
static uint32_t s_opll_ph; /* Q32 electrical */
static float s_opll_vel;   /* electrical turn/s */
static uint8_t s_opll_init;
static float s_obs_vel = NAN; /* turn/s mechanical, telemetry */

static void obs_pll(int valid, uint32_t th_obs) {
	if (!valid) {
		s_opll_init = 0;
		s_obs_vel = NAN;
		return;
	}
	if (!s_opll_init) {
		s_opll_ph = th_obs;
		s_opll_vel = s_pll_vel * (float)g_params.pole_pairs; /* encoderless: start from 0 and let it pull in */
		s_opll_init = 1;
	}
	float err = (float)(int32_t)(th_obs - s_opll_ph) * (1.0f / 4294967296.0f); /* electrical turns */
	s_opll_vel += PLL_KI_TS * err;
	s_opll_ph += (uint32_t)(int32_t)((s_opll_vel + PLL_KP * err) * (4294967296.0f / CTRL_FS_HZ));
	s_obs_vel = s_opll_vel / (float)g_params.pole_pairs;
}

/* time since the ADC sampled the currents (CC4 on the way up), from the TIM1 counter */
static inline float since_current_sample_s(void) {
	int32_t arr = (int32_t)TIM1->ARR, smp = (int32_t)TIM1->CCR4, cnt = (int32_t)TIM1->CNT;
	int32_t d = (TIM1->CR1 & TIM_CR1_DIR) ? (arr - smp) + (arr - cnt) : cnt - smp;
	return (float)d * (1.0f / 170e6f);
}

void currentLoop(void) {
	static float age_cur; /* encoder sample -> current sample */
	uint32_t c0 = DWT->CYCCNT;
	float age;
	int fresh = MT6835_Sample(&hMT6835, &age); /* last good group of this period's burst read */
	age_cur = fresh ? age + ENC_EXTRA_S : age_cur + 1.0f / CTRL_FS_HZ;
	uint32_t ang = hMT6835.angle;								   /* one snapshot for the whole cycle */
	motion_update(ang, fresh, age_cur + since_current_sample_s()); /* the PLL works in "now" time */

	float i_alpha, i_beta, id, iq;
	clarke(g_cur[0], g_cur[1], g_cur[2], &i_alpha, &i_beta);
	uint32_t th_obs = 0;
	int obs = obs_update(i_alpha, i_beta, &th_obs); /* runs in every mode so it is converged when needed */
	obs_pll(obs, th_obs);

	/* encoder angle, advanced by the known delays to the current sample. Blended with the observer by
	 * speed, it gives theta_m = rotor angle when the currents were sampled (Park); theta_o = rotor angle
	 * while the new voltage is applied (inverse Park). Forced during calibration */
	float pp = (float)g_params.pole_pairs;
	float we = s_pll_vel * pp * 4294967296.0f; /* electrical Q32 per second, encoder */
	uint32_t th_enc = elec_angle(ang);
#if ANGLE_COMP
	th_enc += (uint32_t)(int32_t)(we * age_cur);
#endif
	float w = obs_weight(obs, th_obs - th_enc, we);
	s_vel = (w > 0.0f) ? s_pll_vel + w * (s_obs_vel - s_pll_vel) : s_pll_vel; /* s_obs_vel is NaN when invalid */
	g_vel = s_vel;
	g_rpm = s_vel * 60.0f;
#if ANGLE_COMP
	uint32_t out_adv = (uint32_t)(int32_t)(s_vel * pp * 4294967296.0f * OUT_DELAY_S);
#else
	uint32_t out_adv = 0;
#endif
	uint32_t theta_m, theta_o;
	if (g_mode == CTRL_CAL) {
		theta_m = theta_o = g_theta_cmd;
	}
	else {
		theta_m = th_enc + (uint32_t)(int32_t)(w * (float)(int32_t)(th_obs - th_enc)); /* shortest way round */
		theta_o = theta_m + out_adv;
	}
	float sin_ang, cos_ang, sin_o, cos_o;
	sincos_q31(theta_m, &sin_ang, &cos_ang);
	sincos_q31(theta_o, &sin_o, &cos_o);

	/* measurement */
	park(i_alpha, i_beta, sin_ang, cos_ang, &id, &iq);
	g_id = id;
	g_iq = iq;

	/* voltage command in volts */
	float vin = g_vin > 1.0f ? g_vin : 1.0f;
	float v_d = 0.0f, v_q = 0.0f; /* CTRL_OFF: zero volts, bridge stays enabled */
	ctrl_mode_t mode = g_mode;
	outer_loops(mode);
	if (mode == CTRL_RUN) {
		pi_current_step(id, iq, 0.9f * vin / SQRT3, &v_d, &v_q);
	}
	else {
		pi_reset(&g_pi_d); /* start from zero on the next CTRL_RUN */
		pi_reset(&g_pi_q);
		if (mode == CTRL_CAL) {
			v_d = g_vd_cmd;
			v_q = g_vq_cmd;
		}
	}
	/* ~0.4 ms low-pass, read by slow routines in main ('k' back-EMF) */
	g_vq_f += 0.1f * (v_q - g_vq_f);
	g_iq_f += 0.1f * (iq - g_iq_f);

	/* volts -> per-unit of Vdc/sqrt(3), then limit the vector (keeps its direction) */
	float k = SQRT3 / vin;
	float u_d = v_d * k, u_q = v_q * k;
	/* beyond 1 the phases clip and the observer would integrate a voltage that was never applied */
	float m2 = u_d * u_d + u_q * u_q;
	if (m2 > 1.0f) {
		float sc = 1.0f / sqrtf(m2); /* build with -fno-math-errno so this is one vsqrt */
		u_d *= sc;
		u_q *= sc;
	}

	float v_alpha, v_beta;
	ipark(u_d, u_q, sin_o, cos_o, &v_alpha, &v_beta);
	int written = SVPWM(v_alpha, v_beta); /* apply first, log afterwards: less delay between measurement and output */
	s_obs_va1 = s_obs_va;				  /* when nothing was written the previous duties stay on */
	s_obs_vb1 = s_obs_vb;
	if (written) {
		s_obs_va = v_alpha * vin * INV_SQRT3;
		s_obs_vb = v_beta * vin * INV_SQRT3;
	}

	/* decay/step recorder used by the 'l' and 'q' commands */
	if (g_rec_state == 1) {
		g_rec_n = 0;
		g_rec_state = 2;
	}
	if (g_rec_state == 2) {
		g_rec[g_rec_n++] = g_rec_use_q ? iq : id;
		if (g_rec_n >= REC_N) g_rec_state = 3;
	}

	/* telemetry */
	if (++cycles_since_last_frame >= 10) {
		cycles_since_last_frame = 0;
		seq++;
		if (g_telem_en && huart1.gState == HAL_UART_STATE_READY) {
			telemetry.seq = seq;
			telemetry.v[0] = (float)ang * (360.0f / 2097152.0f);
			telemetry.v[1] = g_vin;
			telemetry.v[2] = g_cur[0];
			telemetry.v[3] = g_cur[1];
			telemetry.v[4] = g_cur[2];
			telemetry.v[5] = id;
			telemetry.v[6] = iq;
			telemetry.v[7] = g_rpm;
			telemetry.v[8] = s_vel_ref_eff * 60.0f; /* NaN unless a speed/position loop is active */
			telemetry.v[9] = g_pos;
			telemetry.v[10] = (mode == CTRL_RUN && g_loop == LOOP_POS) ? (float)(int32_t)(s_pos_ref >> 32) + (float)(uint32_t)s_pos_ref * (float)(1.0 / POS_Q32_ONE) : NAN;
			telemetry.v[11] = s_obs_err * 360.0f; /* observer - encoder, deg electrical; NaN without R, L, kv */
			telemetry.v[12] = s_obs_ratio;		  /* |rotor flux| / lambda, 1 when converged */
			telemetry.v[13] = s_obs_w;			  /* observer weight in the commutation angle and the speed */
			telemetry.v[14] = s_obs_vel * 60.0f;  /* rpm from the observer PLL alone; NaN without the observer */
			telemetry.v[15] = s_pll_vel * 60.0f;  /* rpm from the encoder PLL alone (v[7] is the blend) */
			telemetry.sum = frame_sum(&telemetry);
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&telemetry, sizeof(telemetry));
		}
	}

	uint32_t dc = DWT->CYCCNT - c0;
	g_loop_cyc = dc;
	if (dc > g_loop_cyc_max) g_loop_cyc_max = dc;
}