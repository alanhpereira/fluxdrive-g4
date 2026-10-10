#include "bridgeCtrl.h"

#include "tim.h"
#include "dac.h"
#include "i2c.h"

static uint8_t fault_reg;
int32_t g_pwm_res;
int32_t g_max_duty_cycle;
int32_t g_half_duty_cycle;
uint32_t g_lastFaultCleared;

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
	g_pwm_res = htim1.Instance->ARR;
	g_max_duty_cycle = g_pwm_res - ZERO_TIME;
	g_half_duty_cycle = g_max_duty_cycle / 2;
	htim1.Instance->CCR1 = 0;
	htim1.Instance->CCR2 = 0;
	htim1.Instance->CCR3 = 0;
	htim1.Instance->CCR4 = g_pwm_res - 75;

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

int32_t checkAndClearBridgeFault() {
	HAL_StatusTypeDef rc = HAL_I2C_Mem_Read(&hi2c3, 0x47 << 1, 0x80, I2C_MEMADD_SIZE_8BIT, &fault_reg, 1, 10);

	uint32_t bdtr = TIM1->BDTR;
	uint32_t sr = TIM1->SR;

	int moe_off = (bdtr & TIM_BDTR_MOE) == 0;
	int brk_set = (sr & (TIM_SR_BIF | TIM_SR_B2IF)) != 0;

	if (rc == HAL_OK && (moe_off || brk_set)) {
		if (fault_reg & 0x03) {
			return -(fault_reg & 3);
		}
		else {
			if (fault_reg & 0x0C) {	 // latched RESET / VDS_P: clear the driver latch
				uint8_t v = 0xFF;
				HAL_I2C_Mem_Write(&hi2c3, 0x47 << 1, 0x09, I2C_MEMADD_SIZE_8BIT, &v, 1, 10);
			}
			if (GPIOE->IDR & GPIO_PIN_15) {	 // nFAULT released → break input inactive
				setDuty(0, 0, 0);
				TIM1->EGR = TIM_EGR_UG;	 // load zero duty now (also resets the counter)
				__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK | TIM_FLAG_BREAK2);
				__HAL_TIM_MOE_ENABLE(&htim1);
				g_lastFaultCleared = HAL_GetTick();
				return 1;
			}
			return -8;
		}
	}
	return 0;
}

void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef* htim) {
	HAL_I2C_Mem_Read(&hi2c3, 0x47 << 1, 0x80, 1, &fault_reg, 1, 10);
}