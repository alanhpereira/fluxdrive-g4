#include "curSnsCtrl.h"
#include "adc.h"
#include "dac.h"
#include "opamp.h"

volatile int16_t cur_raw[3];
volatile uint16_t vin_raw;

static volatile uint8_t convCptlMask = 0;
static int16_t cur_offset;

extern void currentLoop();

void disableCS() {
	HAL_ADCEx_InjectedStop_IT(&hadc1);
	HAL_ADCEx_InjectedStop_IT(&hadc2);
}

HAL_StatusTypeDef initCS() {
	HAL_StatusTypeDef status;
	cur_offset = 0x800;
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
		cur_raw[0] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1) - cur_offset;
		cur_raw[2] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2) - cur_offset;
	}
	if (hadc == &hadc2) {
		convCptlMask |= 2;
		cur_raw[1] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1) - cur_offset;
		vin_raw = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
	}
	if (convCptlMask == 3) {
		convCptlMask = 0;
		currentLoop();
	}
}