#ifndef PARAMS_H
#define PARAMS_H

#include <stddef.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"

/* flags: what has actually been measured/calibrated (refuse to run FOC until the needed ones are set) */
#define PF_ELEC_OFFSET_VALID (1u << 0)
#define PF_CUR_CAL_VALID (1u << 1)
#define PF_R_VALID (1u << 2)
#define PF_L_VALID (1u << 3)
#define PF_PI_VALID (1u << 4) /* current PI gains set by autotune ('t') */
#define PF_HALL_VALID (1u << 5) /* Hall table (hall_edge) from CAL_HALL */

/* angle source (ang_src): what commutation, speed and the observer's references come from */
#define ANG_ENCODER 0u   /* encoder only; the observer and HFI run and log, nothing blends in */
#define ANG_ENC_OBS 1u   /* encoder, with the observer blended in above obs_lo_v (obs_blend) */
#define ANG_SENSORLESS 2u /* HFI, with the observer above obs_lo_v; no encoder in the loop, position mode refused */
#define ANG_HALL 3u		  /* Hall sensors (interpolated), with the observer above obs_lo_v (obs_blend); no encoder */
#define ANG_HALL_HFI 4u	  /* HFI at low speed, guarded and started by the Halls; Halls from hyb_lo..hyb_hi up; observer above obs_lo_v */

#define PARAM_MAGIC 0x4D505231u /* 'MPR1' */
#define PARAM_VERSION 8u /* bump when the struct layout changes: older copies are rejected, defaults load */

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
	uint8_t ang_src; /* ANG_* */
	uint8_t reserved0;

	/* sensing */
	int16_t cur_off[3]; /* per-channel offset, counts */
	uint8_t reserved1[2];

	/* controller */
	float kp_d, ki_d, kp_q, ki_q;
	float cur_BW; /* current loop bandwidth, Hz (autotune input) */
	/* limits */
	float vbus_min, vbus_max; /* V */
	float temp_max;			  /* deg C */
	float vbus_clamp;		  /* V below vbus_max where the regen clamp starts cutting braking current; 0 = off */
	float vbus_resume;		  /* V below vbus_max the bus must fall to before an OV coast resumes */

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
	uint8_t hfi_mode; /* 0 off, 1 track and log while running in the encoder modes (sensorless always injects) */
	uint8_t hfi_half; /* PWM periods per half wave of the injection, 2..HFI_HALF_MAX */
	uint8_t reserved2;
	float hfi_v;	 /* injection amplitude, V */
	float hfi_bw;	 /* angle PLL bandwidth, Hz */
	float hfi_id;	 /* d-axis bias current for HFI (saturation saliency), A */
	float hfi_sal;	 /* saliency (Lq-Ld)/(Lq+Ld) at hfi_id, from the saliency map; 0 = not measured */
	float hfi_pol_i; /* polarity check pulse current, A */

	/* Hall sensors (hall.c) */
	uint16_t hall_edge[8]; /* Q16 electrical angle where state s (1..6) begins going forward; [0], [7] unused */
	float hyb_lo, hyb_hi;  /* rpm: Hall + HFI hands over from HFI to the Halls between these speeds */

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

/* shared with the CANopen comm record (co_store.c, pages 60-61) */
uint32_t flash_crc32(const void* p, size_t n); /* CRC-32 (IEEE), as stored in the records */
/* erases one 2 KB page and programs len bytes (a multiple of 8) from its start. Refused (HAL_BUSY) while
 * MOE is set: the erase stalls the CPU ~20 ms */
HAL_StatusTypeDef flash_write_page(uint32_t page, const void* data, uint32_t len);
/* same, with the data from get(off, 8 bytes out, ctx) for off = 0, 8, 16...: no RAM copy of the whole page */
HAL_StatusTypeDef flash_write_page_from(uint32_t page, uint32_t len, void (*get)(uint32_t off, uint8_t* dst8, void* ctx), void* ctx);

#endif
