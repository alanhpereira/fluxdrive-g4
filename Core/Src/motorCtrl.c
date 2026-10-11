#include "motorCtrl.h"

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
	float v[17];
} Frame;  // offsets: seq at 2, v at 4, so natural alignment, no padding
_Static_assert(sizeof(Frame) == 72, "frame size");

Frame telemetry;

static float dc_float;
static float center_f;

volatile float g_iq, g_id;
volatile float g_vq_f, g_iq_f;
volatile float g_vel, g_pos, g_rpm;
volatile uint8_t g_home_req;

volatile uint8_t control_enabled = 0;

static int16_t cycles_since_last_frame = 0;

void disableMotor() {
	disableCS();
	disableBridge();
}

HAL_StatusTypeDef initMotor() {
	HAL_StatusTypeDef status;
	cordic_init();
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

static int64_t s_home;	   /* counts, motor direction */
static pos_q32_t s_pos;	   /* from home, motor direction */
static pos_q32_t s_pos_ref; /* latched from g_pos_ref_new */
static uint32_t s_pll_ph;  /* Q32 = one turn, motor direction */
static float s_pll_vel;	   /* turn/s */
static uint8_t s_pll_init;
static float s_vel_ref_eff = NAN; /* speed reference actually fed to the speed PI, for telemetry */

static void motion_update(uint32_t ang) {
	/* this ISR and the SPI DMA callback share priority 0, so turns and ang belong together */
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
		s_pll_ph = ph;
		s_pll_vel = 0.0f;
		s_pll_init = 1;
	}
	float err = (float)(int32_t)(ph - s_pll_ph) * (1.0f / 4294967296.0f); /* turns, wraps correctly */
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
			if (vref > lim) vref = lim;
			else if (vref < -lim) vref = -lim;
			s_vel_ref_eff = vref;
			g_iq_ref = pi_run(&g_pi_vel, vref - s_pll_vel, g_params.rated_current);
		}
	}
	prev = lp;
}

void currentLoop(void) {
	uint32_t ang = hMT6835.angle; /* one snapshot for the whole cycle */
	motion_update(ang);

	/* angle: forced during calibration, encoder otherwise */
	uint32_t theta = (g_mode == CTRL_CAL) ? g_theta_cmd : elec_angle(ang);
	float sin_ang, cos_ang;
	sincos_q31(theta, &sin_ang, &cos_ang);

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
	ipark(u_d, u_q, sin_ang, cos_ang, &v_alpha, &v_beta);
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
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&telemetry, sizeof(telemetry));
		}
	}
}