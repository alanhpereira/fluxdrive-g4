#include "bridgeCtrl.h"

#include "tim.h"
#include "dac.h"
#include "i2c.h"

#include <math.h>

#include "calib.h" /* g_vel, COAST_END_TURNS, SL_REARM_MS */
#include "hall.h"  /* g_hall_bad_n */

static uint8_t fault_reg;
int32_t g_pwm_res;
int32_t g_max_duty_cycle;
int32_t g_half_duty_cycle;
uint32_t g_lastFaultCleared;
volatile uint8_t g_trip_moe;

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
	// short-circuit reference: the gate driver shuts down above SCREF_MV
	HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_2, DAC_ALIGN_12B_R, 0xFFF * SCREF_MV / 3300);

	/* cold power-up: the driver supply comes up after the MCU. Give it up to 100 ms to report ready,
	 * then clear what it latched on the way up (RESET, VCC_UVLO); checkAndClearBridgeFault() retries */
	uint32_t t0 = HAL_GetTick();
	while (HAL_GPIO_ReadPin(DRV_RDY_GPIO_Port, DRV_RDY_Pin) == GPIO_PIN_RESET && HAL_GetTick() - t0 < 100) {
	}
	uint8_t clr = 0xFF;
	HAL_I2C_Mem_Write(&hi2c3, DRV_I2C_ADDR, DRV_REG_CLEAR, I2C_MEMADD_SIZE_8BIT, &clr, 1, 10);

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

HAL_StatusTypeDef readBridgeStatus(uint8_t* status) {
	return HAL_I2C_Mem_Read(&hi2c3, DRV_I2C_ADDR, DRV_REG_STATUS, I2C_MEMADD_SIZE_8BIT, status, 1, 10);
}

/* Returns 0 if the bridge is running, 1 if a fault was just cleared, negative while still faulted
 * (BRIDGE_ERR_* or -(status fault bits)). The I2C traffic only happens while faulted, and at most
 * every FAULT_RETRY_MS, so calling this from every loop is cheap. */
/* re-arming loads zero duty (and SVPWM() waits FAULT_CLEAR_MIN_TIME): the zero vector, which short-brakes a
 * spinning motor. Encoder modes: only below COAST_END_TURNS. Sensorless cannot see the speed with the bridge
 * off: wait SL_REARM_MS. Until MOE has been on once (latched power-up faults) it does not wait */
static uint8_t s_was_on;	  /* MOE has been on since boot */
static uint8_t s_fault_seen; /* driver status fault bits cleared by checkAndClearBridgeFault() since the last bridge_fault_info() */

uint32_t bridge_fault_info(void) {
	uint8_t st = 0;
	uint32_t r = s_fault_seen;
	s_fault_seen = 0;
	if (readBridgeStatus(&st) == HAL_OK)
		r |= st & DRV_ST_FAULTS;
	else
		r |= BFI_I2C;
	if (!(GPIOE->IDR & GPIO_PIN_15)) r |= BFI_NFAULT;
	if (TIM1->SR & TIM_SR_BIF) r |= BFI_BRK;
	if (TIM1->SR & TIM_SR_B2IF) r |= BFI_BRK2;
	return r;
}

static int rearm_safe(uint32_t off_ms) {
	if (!s_was_on) return 1;
	/* without a valid angle g_vel decays to zero while the rotor may still turn: wait, like sensorless. The Halls
	 * see the speed with the bridge off, the encoder does too */
	const uint8_t src = g_params.ang_src;
	if (src == ANG_SENSORLESS || (src <= ANG_ENC_OBS && g_enc_lost_n >= ENC_LOST_N) || (src >= ANG_HALL && g_hall_bad_n >= HALL_LOST_N))
		return off_ms >= SL_REARM_MS;
	return fabsf(g_vel) < COAST_END_TURNS;
}

int32_t checkAndClearBridgeFault() {
	static uint32_t last_try, off_since;
	static uint8_t i2c_fails, was_off;

	int moe_off = (TIM1->BDTR & TIM_BDTR_MOE) == 0;
	int brk_set = (TIM1->SR & (TIM_SR_BIF | TIM_SR_B2IF)) != 0;
	if (!moe_off && !brk_set) {
		was_off = 0;
		s_was_on = 1;
		return 0;
	}
	if (!was_off) {
		was_off = 1;
		off_since = HAL_GetTick();
	}
	if (g_trip_moe) return BRIDGE_ERR_TRIP; /* re-arming would short-brake the coasting motor */

	uint32_t now = HAL_GetTick();
	if (now - last_try < FAULT_RETRY_MS) return BRIDGE_ERR_WAIT;
	last_try = now;

	if (readBridgeStatus(&fault_reg) != HAL_OK) {
		/* driver not answering yet, or the HAL/bus got stuck: start the peripheral over */
		if (++i2c_fails >= 5) {
			i2c_fails = 0;
			HAL_I2C_DeInit(&hi2c3);
			MX_I2C3_Init();
		}
		return BRIDGE_ERR_I2C;
	}
	i2c_fails = 0;

	/* any latched fault, including VCC_UVLO from a slow power-up, must be cleared explicitly.
	 * If the condition is still there the bit comes straight back and we retry next time */
	if (fault_reg & DRV_ST_FAULTS) {
		s_fault_seen |= fault_reg & DRV_ST_FAULTS; /* for bridge_fault_info(): this write clears them */
		uint8_t v = 0xFF;
		HAL_I2C_Mem_Write(&hi2c3, DRV_I2C_ADDR, DRV_REG_CLEAR, I2C_MEMADD_SIZE_8BIT, &v, 1, 10);
		return -(int32_t)(fault_reg & DRV_ST_FAULTS);
	}

	if (!(GPIOE->IDR & GPIO_PIN_15)) return BRIDGE_ERR_NFAULT; /* nFAULT still low: break input active */
	if (!rearm_safe(now - off_since)) return BRIDGE_ERR_SPIN;

	setDuty(0, 0, 0);
	/* a trip in currentLoop() since the check above (the I2C read takes ~50 us) must win: check again with the
	 * interrupts off, so the ISR cannot switch MOE off between that check and the enable */
	const uint32_t pm = __get_PRIMASK();
	__disable_irq();
	if (g_trip_moe) {
		__set_PRIMASK(pm);
		return BRIDGE_ERR_TRIP;
	}
	TIM1->EGR = TIM_EGR_UG; // load zero duty now (also resets the counter)
	__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK | TIM_FLAG_BREAK2);
	__HAL_TIM_MOE_ENABLE(&htim1);
	__set_PRIMASK(pm);
	g_lastFaultCleared = HAL_GetTick();
	return 1;
}