#include "bridgeCtrl.h"

#include "tim.h"
#include "dac.h"
#include "i2c.h"

static uint16_t pwm_res;
static uint8_t fault_reg;

#define PW 100

void disableBridge() {
	htim1.Instance->CCR1 = 0;
	htim1.Instance->CCR2 = 0;
	htim1.Instance->CCR3 = 0;
	htim1.Instance->CCR4 = 0;
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_4);
	HAL_TIM_Base_Stop(&htim1);
}

HAL_StatusTypeDef initBridge() {
	HAL_StatusTypeDef status;
	pwm_res = htim1.Instance->ARR;
	htim1.Instance->CCR1 = 0;
	htim1.Instance->CCR2 = 0;
	htim1.Instance->CCR3 = 0;
	htim1.Instance->CCR4 = pwm_res - ZERO_TIME;

	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
	// ser scref for shutting drivers down to SCREF MV
	HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_2, DAC_ALIGN_12B_R, 0xFFF * SCREF_MV / 3300);
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

void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef* htim) {
	HAL_I2C_Mem_Read(&hi2c3, 0x47 << 1, 0x80, 1, &fault_reg, 1, 10);
}