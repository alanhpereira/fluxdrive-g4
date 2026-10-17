/* ADC callback -> currentLoop(): -O2 even in a size-optimized (-Os) build */
#pragma GCC optimize("O2")

#include "curSnsCtrl.h"
#include "adc.h"
#include "dac.h"
#include "opamp.h"
#include "params.h"

volatile int16_t cur_raw[3];
volatile uint16_t vin_raw;
volatile float g_cur[3];
volatile float g_vin;

static volatile uint8_t convCptlMask = 0;
static int16_t cur_offset[3]; /* ADC counts subtracted in the callback: 2048 + g_cs_off[i] */
int16_t g_cs_off[3];
volatile uint8_t g_cs_off_src;

float g_curMult;
float g_vinMult;

extern void currentLoop();

void disableCS() {
	HAL_ADCEx_InjectedStop_IT(&hadc1);
	HAL_ADCEx_InjectedStop_IT(&hadc2);
}

void cs_set_offsets(const int16_t off[3]) {
	for (int i = 0; i < 3; i++) {
		g_cs_off[i] = off[i];
		cur_offset[i] = (int16_t)(2048 + off[i]); /* one 16-bit store each: the callback never sees half of it */
	}
}

/* the stored offsets once CAL_CUR_OFF has measured them, else the bench board's */
static void cs_fallback(void) {
	static const int16_t k_def[3] = CS_OFF_DEFAULT;
	const int stored = (g_params.flags & PF_CUR_CAL_VALID) != 0;
	cs_set_offsets(stored ? g_params.cur_off : k_def);
	g_cs_off_src = stored ? CS_OFF_SRC_STORED : CS_OFF_SRC_DEFAULT;
}

int cs_measure_offsets(int16_t out[3], int16_t spread[3]) {
	int32_t acc[3] = {0, 0, 0};
	int16_t lo[3] = {INT16_MAX, INT16_MAX, INT16_MAX}, hi[3] = {INT16_MIN, INT16_MIN, INT16_MIN};
	for (int n = 0; n < CS_OFF_N; n++) {
		HAL_Delay(1);
		for (int i = 0; i < 3; i++) {
			int16_t r = (int16_t)(cur_raw[i] + cur_offset[i]); /* back to the raw ADC value */
			acc[i] += r;
			if (r < lo[i]) lo[i] = r;
			if (r > hi[i]) hi[i] = r;
		}
	}
	int ok = 1;
	for (int i = 0; i < 3; i++) {
		int32_t a = acc[i] + (acc[i] >= 0 ? CS_OFF_N / 2 : -CS_OFF_N / 2); /* rounded */
		out[i] = (int16_t)(a / CS_OFF_N - 2048);
		spread[i] = (int16_t)(hi[i] - lo[i]);
		if (out[i] > CS_OFF_ABS_MAX || out[i] < -CS_OFF_ABS_MAX || spread[i] > CS_OFF_SPREAD_MAX) ok = 0;
	}
	return ok;
}

void cs_boot_offsets(void) {
	int16_t m[3], sp[3];
	if (!cs_measure_offsets(m, sp)) return;
	for (int i = 0; i < 3; i++) {
		int d = m[i] - g_cs_off[i];
		if (d > CS_OFF_DEV_MAX || d < -CS_OFF_DEV_MAX) return; /* rotor turning, or a bad fallback: keep it */
	}
	cs_set_offsets(m);
	g_cs_off_src = CS_OFF_SRC_BOOT;
}

HAL_StatusTypeDef initCS() {
	HAL_StatusTypeDef status;
	cs_fallback();
	g_curMult = -VDDA / (2.0f * 4095.0f * CS_GAIN * CS_SHUNT);
	g_vinMult = VDDA * (VIN_RL + VIN_RH) / (4095.0f * VIN_RL);

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);

	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, 0x800);
	status = HAL_ADC_Start(&hadc1);
	if (status != HAL_OK) {
		disableCS();
		return status;
	}
	status = HAL_ADC_Start(&hadc2);
	if (status != HAL_OK) {
		disableCS();
		return status;
	}
	HAL_OPAMP_Start(&hopamp1);
	if (status != HAL_OK) {
		disableCS();
		return status;
	}
	HAL_OPAMP_Start(&hopamp2);
	if (status != HAL_OK) {
		disableCS();
		return status;
	}
	HAL_OPAMP_Start(&hopamp3);
	if (status != HAL_OK) {
		disableCS();
		return status;
	}
	status = HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	if (status != HAL_OK) {
		disableCS();
		return status;
	}
	status = HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	if (status != HAL_OK) {
		disableCS();
		return status;
	}

	status = HAL_ADCEx_InjectedStart_IT(&hadc1);
	if (status != HAL_OK) {
		disableCS();
		return status;
	}
	status = HAL_ADCEx_InjectedStart_IT(&hadc2);
	if (status != HAL_OK) {
		disableCS();
		return status;
	}
	return HAL_OK;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc == &hadc1) {
		convCptlMask |= 1;
		cur_raw[0] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1) - cur_offset[0];
		cur_raw[2] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2) - cur_offset[2];
	}
	if (hadc == &hadc2) {
		convCptlMask |= 2;
		cur_raw[1] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1) - cur_offset[1];
		vin_raw = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
	}
	if (convCptlMask == 3) {
		convCptlMask = 0;
		g_cur[0] = cur_raw[0] * g_curMult;
		g_cur[1] = cur_raw[1] * g_curMult;
		g_cur[2] = cur_raw[2] * g_curMult;
		g_vin = vin_raw * g_vinMult;
		currentLoop();
	}
}