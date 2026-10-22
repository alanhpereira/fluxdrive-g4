#ifndef TEMP_H
#define TEMP_H

#include "stm32g4xx_hal.h"

/* Temperatures: the board NTC (TH1, PC4 = ADC2_IN5), the motor NTC on J12 (PC3 = ADC1_IN9) and the MCU die
 * (ADC1 internal sensor). Regular conversions, one per channel every TEMP_POLL_MS, started at the TIM1 counter
 * bottom (temp_isr_start() in TIM1_UP_TIM16_IRQHandler) so they end long before the CC4 current sample */

/* NTC: ANTC1608-103F3450FB, 10 k at 25 deg C, B 3450 K, in a divider with a 10 k 0.1 % resistor.
 * Board: 10 k from 3.3 V, NTC to GND. Motor (J12): NTC from 3.3 V, 10 k (R18) to GND */
#define TEMP_R_PULL 10000.0f
#define NTC_R25 10000.0f
#define NTC_B 3450.0f
#define TEMP_N_MIN 100	/* counts: below / above these the NTC is shorted or open (about 157 / -50 deg C) */
#define TEMP_N_MAX 4000

#define TEMP_POLL_MS 10
#define TEMP_DERATE_BAND 10.0f /* deg C below a limit where the current cap falls from max_current to zero */
#define TEMP_HYST 10.0f		   /* deg C below a limit before a run is allowed again after an over-temperature stop */
#define TEMP_DIE_MAX 110.0f	   /* deg C, MCU die (same package as the gate driver) */

enum { TEMP_OK = 0, TEMP_DERATING, TEMP_OVER, TEMP_SENSOR_BAD };

extern volatile float g_temp_board; /* deg C, NaN: not measured yet or sensor open/shorted */
extern volatile float g_temp_die;	/* deg C, NaN: sensor path not available */
extern volatile float g_temp_motor; /* deg C, NaN: no motor sensor (mtemp_max = 0) or open/shorted */
extern volatile uint16_t g_temp_motor_raw; /* filtered counts of the J12 input */
extern volatile float g_i_temp;	   /* A, current cap from the temperatures: read by the ISR */
extern volatile float g_i_vel;	   /* A, the speed loop's limit: rated_current, capped by g_i_temp */
extern volatile uint8_t g_temp_state; /* TEMP_* */
extern volatile uint8_t g_temp_start; /* main arms a conversion, the TIM1 bottom interrupt starts it */

/* before the injected conversions are started: sample times (SMPR may only change while JADSTART = 0) */
void temp_init(void);
/* main loop (cmd_poll(), chk()): reads the last conversions, filters, converts, updates g_i_temp */
void temp_poll(void);
/* ST_OK, ST_OVERTEMP, ST_MOTOR_OVERTEMP or ST_TEMP_SENSOR (an st_t). *val: the temperature over its limit,
 * or for a sensor fault 0 = board, 2 = motor */
int temp_fault(float* val);

/* TIM1 counter bottom (mt6835.c), 19.6 us before the current sample: the conversions take 1.4 us (dividers)
 * and 6.1 us (die sensor). Clears the rs bits in the write like LL_ADC_REG_StartConversion(). always_inline:
 * included above mt6835.c's O2 pragma, -Og would leave it out of line in flash, called from CCM through a veneer */
static inline __attribute__((always_inline)) void temp_isr_start(void) {
	if (g_temp_start) {
		g_temp_start = 0;
		ADC1->CR = (ADC1->CR & ~ADC_CR_BITS_PROPERTY_RS) | ADC_CR_ADSTART;
		ADC2->CR = (ADC2->CR & ~ADC_CR_BITS_PROPERTY_RS) | ADC_CR_ADSTART;
	}
}

#endif
