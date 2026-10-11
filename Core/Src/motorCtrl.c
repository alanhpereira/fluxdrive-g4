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

void currentLoop(void) {
	uint32_t ang = hMT6835.angle; /* one snapshot for the whole cycle */

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
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&telemetry, sizeof(telemetry));
		}
	}
}