#include "temp.h"

#include <math.h>

#include "calib.h"		/* g_mode, g_loop */
#include "curSnsCtrl.h" /* g_vdda */
#include "pi_ctrl.h"	/* g_iq_ref */
#include "params.h"
#include "proto.h" /* st_t */

volatile float g_temp_board = NAN;
volatile float g_temp_die = NAN;
volatile float g_temp_motor = NAN;
volatile uint16_t g_temp_motor_raw;
volatile float g_i_temp; /* no current until the first evaluation (~20 ms after boot) */
volatile float g_i_vel;
volatile uint8_t g_temp_state;
volatile uint8_t g_temp_start;

/* ADC1 alternates between the motor input and the die sensor; ADC2 always converts the board NTC */
#define CH_MOTOR 9u	 /* ADC1_IN9, PC3 */
#define CH_DIE 16u	 /* ADC1 internal temperature sensor */
#define CH_BOARD 5u	 /* ADC2_IN5, PC4 */
#define SMP_47 4u	 /* 47.5 cycles: 1.4 us per conversion. The divider caps (C26, C19) hold the input */
#define SMP_247 6u	 /* 247.5 cycles = 5.8 us: the die sensor needs >= 5 us */
#define FILT_K_1 0.025f /* IIR per sample: ~0.4 s at one sample per TEMP_POLL_MS (board) */
#define FILT_K_2 0.05f	/* ~0.4 s at one sample every second poll (motor, die) */

static float s_n_board, s_n_motor, s_n_die; /* filtered counts, 0 = no sample yet */
static uint8_t s_die_ch;					/* 1: ADC1 is converting the die sensor */
static uint8_t s_ts_ok;						/* sensor path switched on (cs_vdda()) */
static uint8_t s_over_board, s_over_die, s_over_motor; /* latched until TEMP_HYST below the limit */
static uint8_t s_armed;

static void adc1_channel(uint32_t ch) {
	ADC1->SQR1 = ch << ADC_SQR1_SQ1_Pos; /* L = 0: one conversion. ADSTART is 0 here */
}

void temp_init(void) {
	s_ts_ok = (ADC12_COMMON->CCR & ADC_CCR_VSENSESEL) != 0;
	ADC1->SMPR1 = (ADC1->SMPR1 & ~ADC_SMPR1_SMP9_Msk) | (SMP_47 << ADC_SMPR1_SMP9_Pos);
	ADC1->SMPR2 = (ADC1->SMPR2 & ~ADC_SMPR2_SMP16_Msk) | (SMP_247 << ADC_SMPR2_SMP16_Pos);
	ADC2->SMPR1 = (ADC2->SMPR1 & ~ADC_SMPR1_SMP5_Msk) | (SMP_47 << ADC_SMPR1_SMP5_Pos);
	ADC2->SQR1 = CH_BOARD << ADC_SQR1_SQ1_Pos;
	adc1_channel(CH_MOTOR);
	ADC1->ISR = ADC_ISR_EOC; /* write 1 to clear */
	ADC2->ISR = ADC_ISR_EOC;
	/* the TIM1 update interrupt starts the conversions, with or without the encoder burst (mt6835.c owns it) */
	TIM1->DIER |= TIM_DIER_UIE;
	HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
}

static void filt(float* f, uint32_t n, float k) {
	*f = (*f == 0.0f) ? (float)n + 0.001f : *f + k * ((float)n - *f); /* the first sample seeds it */
}

/* Beta model; NaN outside the valid count window */
static float ntc_c(float n, int low_side) {
	if (!(n > TEMP_N_MIN && n < TEMP_N_MAX)) return NAN;
	const float r = low_side ? TEMP_R_PULL * n / (4095.0f - n) : TEMP_R_PULL * (4095.0f - n) / n;
	return 1.0f / (1.0f / 298.15f + logf(r / NTC_R25) / NTC_B) - 273.15f;
}

static float die_c(float n) {
	const float cal1 = (float)*TEMPSENSOR_CAL1_ADDR, cal2 = (float)*TEMPSENSOR_CAL2_ADDR;
	if (!(cal2 > cal1)) return NAN;
	const float n3 = n * g_vdda * (1000.0f / (float)TEMPSENSOR_CAL_VREFANALOG); /* as read at the calibration's 3.0 V */
	return (float)TEMPSENSOR_CAL1_TEMP + (n3 - cal1) * (float)(TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) / (cal2 - cal1);
}

/* cap factor 1..0 across the band below lim, and the latched over flag. Returns 1 when the reading is invalid */
static int limit(float t, float lim, uint8_t* over, float* k) {
	if (t != t) return 1;
	if (t >= lim) *over = 1;
	else if (t < lim - TEMP_HYST) *over = 0;
	float f = (lim - t) * (1.0f / TEMP_DERATE_BAND);
	f = f > 1.0f ? 1.0f : f < 0.0f ? 0.0f : f;
	if (f < *k) *k = f;
	return 0;
}

static void evaluate(void) {
	g_temp_board = ntc_c(s_n_board, 1);
	g_temp_die = (s_ts_ok && s_n_die > 0.0f) ? die_c(s_n_die) : NAN;
	g_temp_motor = g_params.mtemp_max ? ntc_c(s_n_motor, 0) : NAN;
	g_temp_motor_raw = (uint16_t)s_n_motor;

	float k = 1.0f;
	int bad = 0;
	if (g_params.temp_max > 0.0f) bad |= limit(g_temp_board, g_params.temp_max, &s_over_board, &k);
	else s_over_board = 0;
	if (g_temp_die == g_temp_die) limit(g_temp_die, TEMP_DIE_MAX, &s_over_die, &k);
	if (g_params.mtemp_max) bad |= limit(g_temp_motor, (float)g_params.mtemp_max, &s_over_motor, &k);
	else s_over_motor = 0;
	if (bad) k = 0.0f;
	const float it = k * g_params.max_current;
	g_i_temp = it;
	g_i_vel = it < g_params.rated_current ? it : g_params.rated_current; /* the speed loop (outer_loops()) */
	/* current mode from USB (and an overvoltage resume) writes g_iq_ref once: cap it here. CST caps its own each tick
	 * (co402.c), the speed loop through g_i_vel. A tick in between costs at most one stale value */
	if (g_mode == CTRL_RUN && g_loop == LOOP_CURRENT && fabsf(g_iq_ref) > it) g_iq_ref = copysignf(it, g_iq_ref);
	g_temp_state = bad ? TEMP_SENSOR_BAD : (s_over_board || s_over_die || s_over_motor) ? TEMP_OVER : k < 1.0f ? TEMP_DERATING : TEMP_OK;
}

void temp_poll(void) {
	static uint32_t last;
	const uint32_t now = uwTick; /* not HAL_GetTick(): it lives in CCM, and a call from flash adds a veneer there */
	if (now - last < TEMP_POLL_MS) return;
	last = now;
	if (s_armed) {
		static uint8_t waits;
		if (g_temp_start || !(ADC1->ISR & ADC_ISR_EOC) || !(ADC2->ISR & ADC_ISR_EOC)) {
			if (++waits < 10) return;
			/* no result for 100 ms (TIM1 stopped?): the readings go invalid rather than stale */
			waits = 0;
			s_n_board = s_n_motor = s_n_die = 0.0f;
			evaluate();
			s_armed = 0;
			g_temp_start = 0;
			ADC1->ISR = ADC_ISR_EOC;
			ADC2->ISR = ADC_ISR_EOC;
			return;
		}
		waits = 0;
		const uint32_t n1 = ADC1->DR, n2 = ADC2->DR; /* reading DR clears EOC */
		filt(&s_n_board, n2, FILT_K_1);
		filt(s_die_ch ? &s_n_die : &s_n_motor, n1, FILT_K_2);
		s_armed = 0;
		evaluate();
	}
	if (s_ts_ok) {
		s_die_ch ^= 1;
		adc1_channel(s_die_ch ? CH_DIE : CH_MOTOR);
	}
	s_armed = 1;
	g_temp_start = 1;
}

int temp_fault(float* val) {
	float v = 0.0f;
	st_t e = ST_OK;
	if (g_temp_state == TEMP_SENSOR_BAD) {
		e = ST_TEMP_SENSOR;
		v = (g_params.temp_max > 0.0f && g_temp_board != g_temp_board) ? 0.0f : 2.0f;
	}
	else if (s_over_motor) {
		e = ST_MOTOR_OVERTEMP;
		v = g_temp_motor;
	}
	else if (s_over_board || s_over_die) {
		e = ST_OVERTEMP;
		v = s_over_board ? g_temp_board : g_temp_die;
	}
	if (val) *val = v;
	return e;
}
