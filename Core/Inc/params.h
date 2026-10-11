#ifndef PARAMS_H
#define PARAMS_H

#include <stdint.h>
#include "stm32g4xx_hal.h"

/* flags: what has actually been measured/calibrated (refuse to run FOC until the needed ones are set) */
#define PF_ELEC_OFFSET_VALID (1u << 0)
#define PF_CUR_CAL_VALID (1u << 1)
#define PF_R_VALID (1u << 2)
#define PF_L_VALID (1u << 3)
#define PF_PI_VALID (1u << 4) /* current PI gains set by autotune ('t') */

#define PARAM_MAGIC 0x4D505231u /* 'MPR1' */
#define PARAM_VERSION 2u		/* bump when the struct layout changes */

typedef struct {
	/* header */
	uint32_t magic;
	uint16_t version;
	uint16_t size;
	uint32_t seq; /* increments on every save, picks the newest slot */
	uint32_t flags;

	/* motor */
	float kv;			  /* rpm/V */
	float rd, rq;		  /* ohm */
	float ld, lq;		  /* henry */
	float rated_current;  /* A */
	float max_current;	  /* A, hard limit used by the controller */
	uint32_t elec_offset; /* electrical angle offset, 2^32 = 360 deg */
	uint8_t pole_pairs;
	int8_t enc_dir; /* +1 or -1 */
	uint8_t reserved0[2];

	/* sensing */
	float cur_scale;	/* A per count, sign included */
	int16_t cur_off[3]; /* per-channel offset, counts */
	uint8_t reserved1[2];
	float vin_scale; /* V per count */

	/* controller */
	float kp_d, ki_d, kp_q, ki_q;
	float cur_BW; /* current loop bandwidth, Hz (autotune input) */
	/* limits */
	float vbus_min, vbus_max; /* V */
	float temp_max;			  /* deg C */

	uint32_t crc; /* CRC32 of everything above; keep last */
} motor_params_t;

_Static_assert(sizeof(motor_params_t) % 8 == 0, "flash programs 64-bit words: size must be a multiple of 8");

extern motor_params_t g_params;

/* Returns 0 if a valid copy was loaded from flash, 1 if defaults are in use. */
int params_load(void);

/* Writes g_params to the inactive slot. The bridge must be disabled (MOE off), because the CPU
 * stalls for ~20 ms during the page erase. */
HAL_StatusTypeDef params_save(void);

#endif
