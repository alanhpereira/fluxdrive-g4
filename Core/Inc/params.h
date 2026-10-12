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
#define PARAM_VERSION 5u /* bump when the struct layout changes: older copies are rejected, defaults load */

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

	/* outer loops */
	float vel_kp;  /* A per turn/s */
	float vel_ki;  /* A per turn */
	float pos_kp;  /* turn/s per turn */
	float vel_lim; /* turn/s, position mode speed limit and overspeed trip (x1.5) */

	/* back-EMF observer and speed ramp */
	float obs_gain;		/* rad/s, flux magnitude correction */
	float obs_lo_v;		/* peak back-EMF where the observer starts to get weight, V */
	float obs_hi_v;		/* ...and where it has all of it */
	float obs_flux_tol; /* |flux| / lambda must stay within 1 +- tol */
	float obs_err_max;	/* deg electrical: sustained observer - encoder difference above this drops it */
	float acc_lim;		/* rpm/s, speed reference ramp in the speed/position loops; 0 = off */
	uint8_t obs_blend;	/* 1: blend the observer into angle and speed, 0: run and log it only */

	/* high-frequency injection (hfi.c) */
	uint8_t hfi_mode; /* 0 off, 1 estimate only (track and log while running), 2 sensorless (HFI + observer, encoderless start) */
	uint8_t hfi_half; /* PWM periods per half wave of the injection, 2..HFI_HALF_MAX */
	uint8_t reserved2;
	float hfi_v;	 /* injection amplitude, V */
	float hfi_bw;	 /* angle PLL bandwidth, Hz */
	float hfi_id;	 /* d-axis bias current for HFI (saturation saliency), A */
	float hfi_sal;	 /* saliency (Lq-Ld)/(Lq+Ld) at hfi_id, from the saliency map; 0 = not measured */
	float hfi_pol_i; /* polarity check pulse current, A */

	uint32_t crc; /* CRC32 of everything above; keep last */
} motor_params_t;

_Static_assert(sizeof(motor_params_t) % 8 == 0, "flash programs 64-bit words: size must be a multiple of 8");

extern motor_params_t g_params;

/* Returns 0 if a valid copy was loaded from flash, 1 if defaults are in use. */
int params_load(void);

/* Writes g_params to the inactive slot. The bridge must be disabled (MOE off), because the CPU
 * stalls for ~20 ms during the page erase. */
HAL_StatusTypeDef params_save(void);

/* 1 if g_params differs from the newest copy in flash (or there is none): unsaved changes */
int params_dirty(void);

#endif
