#include "bridgeCtrl.h"

#include "tim.h"
#include "adc.h"
#include "i2c.h"

static uint16_t pwm_res;
static volatile int16_t cur_raw[3];
static volatile uint16_t vin_raw;

static volatile uint8_t convCptlMask = 0;
static int16_t cur_offset;
static uint8_t fault_reg;

void runControl() {
	if ((HAL_GetTick() / 1000) & 1) {
		htim1.Instance->CCR1 = pwm_res - ZERO_TIME;
		htim1.Instance->CCR2 = 0;
		htim1.Instance->CCR3 = 0;
	}
	else {
		htim1.Instance->CCR1 = 0;
		htim1.Instance->CCR2 = 0;
		htim1.Instance->CCR3 = 0;
	}
}

void disableBridge() {
	htim1.Instance->CCR1 = 0;
	htim1.Instance->CCR2 = 0;
	htim1.Instance->CCR3 = 0;
	htim1.Instance->CCR4 = 0;
	/*HAL_ADCEx_InjectedStop_IT(&hadc1);
	HAL_ADCEx_InjectedStop_IT(&hadc2);
	HAL_TIM_Base_Stop(&htim1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);*/
}

HAL_StatusTypeDef initBridge() {
	HAL_StatusTypeDef status;
	pwm_res = htim1.Instance->ARR;
	htim1.Instance->CCR1 = 0;
	htim1.Instance->CCR2 = 0;
	htim1.Instance->CCR3 = 0;
	htim1.Instance->CCR4 = pwm_res - ZERO_TIME;
	cur_offset = 0x400;
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
	status = HAL_ADCEx_InjectedStart_IT(&hadc1);
	if (status != HAL_OK) {
		disableBridge();
		return status;
	}
	status = HAL_ADCEx_InjectedStart_IT(&hadc2);
	if (status != HAL_OK) {
		disableBridge();
		return status;
	}
	status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	if (status != HAL_OK) {
		disableBridge();
		return status;
	}
	status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	if (status != HAL_OK) {
		disableBridge();
		return status;
	}
	status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	if (status != HAL_OK) {
		disableBridge();
		return status;
	}
	status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	if (status != HAL_OK) {
		disableBridge();
		return status;
	}
	status = HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	if (status != HAL_OK) {
		disableBridge();
		return status;
	}
	status = HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	if (status != HAL_OK) {
		disableBridge();
		return status;
	}
	status = HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
	if (status != HAL_OK) {
		disableBridge();
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
		runControl();
	}
}

void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef* htim) {
	HAL_I2C_Mem_Read(&hi2c3, 0x47 << 1, 0x80, 1, &fault_reg, 1, 10);
}