#include "motorCtrl.h"

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
void SVPWM(float alpha, float beta) {
	if (HAL_GetTick() < g_lastFaultCleared + FAULT_CLEAR_MIN_TIME) return;
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
	g_vel = s_pll_vel;
	g_rpm = s_pll_vel * 60.0f;
}

/* ------------------------------------------------------------------ speed / position loops */
/* in CTRL_RUN with g_loop != LOOP_CURRENT: writes g_iq_ref at OUTER_FS_HZ */
static void outer_loops(ctrl_mode_t mode) {
	static loop_t prev = LOOP_CURRENT;
	static int div;
	loop_t lp = (mode == CTRL_RUN) ? g_loop : LOOP_CURRENT;

	if (lp == LOOP_CURRENT) {
		pi_reset(&g_pi_vel);
		s_vel_ref_eff = NAN;
		div = OUTER_DIV; /* run on the first cycle after switching in */
	}
	else {
		if (prev == LOOP_CURRENT) g_pi_vel.integ = g_iq_ref; /* bumpless: carry on from the present current */
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
			s_vel_ref_eff = vref;
			g_iq_ref = pi_run(&g_pi_vel, vref - s_pll_vel, g_params.rated_current);
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
#define ANGLE_COMP 1										 /* 0: use the raw encoder angle for both transforms */

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
	uint32_t ang = hMT6835.angle; /* one snapshot for the whole cycle */
	motion_update(ang, fresh, age_cur + since_current_sample_s()); /* the PLL works in "now" time */

	/* angle: forced during calibration, encoder otherwise. With the encoder, advance it by the known
	 * delays: theta_m = rotor angle when the currents were sampled (Park), theta_o = rotor angle while the
	 * new voltage is applied (inverse Park) */
	uint32_t theta_m, theta_o;
	if (g_mode == CTRL_CAL) {
		theta_m = theta_o = g_theta_cmd;
	}
	else {
		uint32_t e = elec_angle(ang);
#if ANGLE_COMP
		float we = s_pll_vel * (float)g_params.pole_pairs * 4294967296.0f; /* electrical Q32 per second */
		theta_m = e + (uint32_t)(int32_t)(we * age_cur);
		theta_o = e + (uint32_t)(int32_t)(we * (age_cur + OUT_DELAY_S));
#else
		theta_m = theta_o = e;
#endif
	}
	float sin_ang, cos_ang, sin_o, cos_o;
	sincos_q31(theta_m, &sin_ang, &cos_ang);
	sincos_q31(theta_o, &sin_o, &cos_o);

	/* measurement */
	float i_alpha, i_beta, id, iq;
	clarke(g_cur[0], g_cur[1], g_cur[2], &i_alpha, &i_beta);
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
	// float m2 = u_d * u_d + u_q * u_q;
	//  if (m2 > 1.0f) {
	//  maybe use cordic
	//  float sc = 1.0f / sqrtf(m2); /* build with -fno-math-errno so this is one vsqrt */
	//  u_d *= sc;
	//  u_q *= sc;
	// }

	float v_alpha, v_beta;
	ipark(u_d, u_q, sin_o, cos_o, &v_alpha, &v_beta);
	SVPWM(v_alpha, v_beta); /* apply first, log afterwards: less delay between measurement and output */

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
			telemetry.sum = frame_sum(&telemetry);
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&telemetry, sizeof(telemetry));
		}
	}

	uint32_t dc = DWT->CYCCNT - c0;
	g_loop_cyc = dc;
	if (dc > g_loop_cyc_max) g_loop_cyc_max = dc;
}