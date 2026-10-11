#include "motorCtrl.h"

#include "i2c.h"
#include "usart.h"
#include "cordicMgr.h"
#include "mt6835.h"

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
	MT6835_WriteRegister(&hMT6835, MT6835_ADDR_ROT, 0x0f);
	uint32_t last_fault = HAL_GetTick();
	control_enabled = 0;
	g_vin = 0;
	while (HAL_GetTick() < last_fault + 200) {
		if (g_vin > 0.1) {
			SVPWM(5 / g_vin * ((HAL_GetTick() - last_fault) / 200.0f), 0);
		}
		else {
			SVPWM(0, 0);
			last_fault = HAL_GetTick();
		}
		if (checkAndClearBridgeFault() != 0) {
			SVPWM(0, 0);
			last_fault = HAL_GetTick();
		}
	}
	HAL_Delay(100);
	MT6835_SetZero(&hMT6835);
	SVPWM(0, 0);
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

void currentLoop() {
	float alpha, beta;
	uint32_t ang = hMT6835.angle;
	if (!control_enabled) return;
	float sin_ang, cos_ang;
	sincos_q31((ang << 11) * POLE_PAIRS + ELEC_OFFSET, &sin_ang, &cos_ang);

	float al, be, iq, id;
	clarke(g_cur[0], g_cur[1], g_cur[2], &al, &be);
	park(al, be, sin_ang, cos_ang, &id, &iq);
	g_id = id;
	g_iq = iq;

	if (cycles_since_last_frame >= 10) {
		cycles_since_last_frame = 0;
		seq++;
		if (huart1.gState == HAL_UART_STATE_READY) {
			telemetry.seq = seq;
			telemetry.v[0] = (ang * 360.0) / (1 << 21);
			telemetry.v[1] = g_vin;
			telemetry.v[2] = g_cur[0];
			telemetry.v[3] = g_cur[1];
			telemetry.v[4] = g_cur[2];
			telemetry.v[5] = g_id;
			telemetry.v[6] = g_iq;
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&telemetry, sizeof(telemetry));
		}
	}

	cycles_since_last_frame++;
	ipark(0, 15, sin_ang, cos_ang, &alpha, &beta);
	//  sincos_q31(HAL_GetTick() * (0xffffffff / 1000 * 7 * 0), &beta, &alpha);
	// alpha = 3;
	// beta = 0;
	float inv_vin = 1 / g_vin;
	alpha *= inv_vin;
	beta *= inv_vin;
	SVPWM(alpha, beta);
	// SVPWM(0.4f, 0.0f);
}