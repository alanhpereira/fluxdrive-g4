#include "motorCtrl.h"

#include "i2c.h"
#include "usart.h"
#include "cordicMgr.h"
#include "mt6835.h"

#define SQRT3_2 0.8660254f
#define INV_SQRT3 0.57735027f

void disableMotor() {
	disableCS();
	disableBridge();
}
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

static int16_t cycles_since_last_frame = 0;

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
	HAL_Delay(10);
	SVPWM(3 / g_vin, 0);
	HAL_Delay(10);
	MT6835_SetZero(&hMT6835);
	MT6835_StartBurst(&hMT6835);
	dc_float = g_max_duty_cycle * INV_SQRT3;
	center_f = (float)g_half_duty_cycle + 0.5f;
	telemetry.sync[0] = 0xA5;
	telemetry.sync[1] = 0x5A;
	return HAL_OK;
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
	sincos_q31(HAL_GetTick() * (0xffffffff / 1000 * 7), &beta, &alpha);
	float inv_vin = 1 / g_vin;
	alpha *= inv_vin;
	beta *= inv_vin;
	if (cycles_since_last_frame >= 10) {
		cycles_since_last_frame = 0;
		seq++;
		if (huart1.gState == HAL_UART_STATE_READY) {
			telemetry.seq = seq;
			telemetry.v[0] = g_vin;
			telemetry.v[1] = g_cur[0];
			telemetry.v[2] = g_cur[1];
			telemetry.v[3] = g_cur[2];
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&telemetry, sizeof(telemetry));
		}
	}
	cycles_since_last_frame++;
	SVPWM(alpha, beta);
}