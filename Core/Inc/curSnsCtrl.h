#ifndef CURSNSCTRL_H
#define CURSNSCTRL_H

#include "stm32g4xx_hal.h"

#define VDDA 3.3f
#define CS_SHUNT 0.002f
#define CS_GAIN 16.0f
#define VIN_RL 10.0f
#define VIN_RH 270.0f

/* current-sense zero offsets, counts relative to mid-scale (2048): ADC zero = 2048 + off.
 * CS_OFF_DEFAULT is the bench board's, used until CAL_CUR_OFF stores g_params.cur_off */
#define CS_OFF_DEFAULT {-18, -10, -6}
#define CS_OFF_N 64			  /* samples of the boot / CAL_CUR_OFF measurement, one per ms */
#define CS_OFF_ABS_MAX 150	  /* a channel further than this from 2048 is broken, not offset */
#define CS_OFF_SPREAD_MAX 40  /* max - min over the samples: more means current flowed (rotor turning) */
#define CS_OFF_DEV_MAX 25	  /* boot: a measurement further than this from the fallback is not trusted (~0.3 A) */
enum { CS_OFF_SRC_DEFAULT = 0, CS_OFF_SRC_STORED, CS_OFF_SRC_BOOT };

extern volatile int16_t cur_raw[3];
extern volatile uint16_t vin_raw;
extern volatile float g_cur[3];
extern volatile float g_vin;
extern int16_t g_cs_off[3];			/* offsets in use, counts from 2048 */
extern volatile uint8_t g_cs_off_src; /* CS_OFF_SRC_* */

void disableCS();
HAL_StatusTypeDef initCS();
/* averages the raw ADC zero over CS_OFF_N ms (main context, zero current: bridge at zero volts or off,
 * rotor still). Returns 1 if the result is plausible; out = offsets from 2048, spread = max - min */
int cs_measure_offsets(int16_t out[3], int16_t spread[3]);
void cs_set_offsets(const int16_t off[3]);
/* boot: measure, and keep the result if it is close to the stored/default offsets */
void cs_boot_offsets(void);
#endif