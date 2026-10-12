/* ADC callback -> currentLoop(): -O2 even in a size-optimized (-Os) build */
#pragma GCC optimize("O2")

#include "curSnsCtrl.h"
#include "adc.h"
#include "dac.h"
#include "opamp.h"

volatile int16_t cur_raw[3];
volatile uint16_t vin_raw;
volatile float g_cur[3];
volatile float g_vin;

static volatile uint8_t convCptlMask = 0;
static int16_t cur_offset[3];

float g_curMult;
float g_vinMult;

extern void currentLoop();

void disableCS() {
	HAL_ADCEx_InjectedStop_IT(&hadc1);
	HAL_ADCEx_InjectedStop_IT(&hadc2);
}

HAL_StatusTypeDef initCS() {
	HAL_StatusTypeDef status;
	cur_offset[0] = 2048 - 18;
	cur_offset[1] = 2048 - 10;
	cur_offset[2] = 2048 - 6;
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