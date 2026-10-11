#ifndef CALIB_H
#define CALIB_H

#include <stdint.h>
#include "params.h"

#define SQRT3 1.7320508f

/* position as Q32.32 in an int64: upper 32 bits = whole turns, lower 32 bits = fraction of a turn.
 * Same resolution everywhere in +-2^31 turns; one encoder count (2^-21 turn) = 2^11 */
typedef int64_t pos_q32_t;
#define POS_Q32_ONE 4294967296.0
#define CTRL_FS_HZ 25000.0f /* rate at which currentLoop() actually runs, measure it */
#define OUTER_DIV 10						  /* speed/position loops run every OUTER_DIV current cycles */
#define OUTER_FS_HZ (CTRL_FS_HZ / OUTER_DIV) /* 2500 Hz */

/* ---- shared with currentLoop() (all written/read across ISR and main: keep volatile) ---- */
typedef enum { CTRL_OFF = 0,
			   CTRL_CAL,
			   CTRL_RUN,
} ctrl_mode_t;

extern volatile ctrl_mode_t g_mode;

/* what feeds the current loop in CTRL_RUN */
typedef enum { LOOP_CURRENT = 0, /* g_id_ref/g_iq_ref set by the 'g' command */
			   LOOP_VEL,		  /* speed PI -> g_iq_ref */
			   LOOP_POS,		  /* position P -> speed PI -> g_iq_ref */
} loop_t;
extern volatile loop_t g_loop;
extern volatile float g_vd_cmd, g_vq_cmd; /* volts, used in CTRL_CAL */
extern volatile uint32_t g_theta_cmd;	  /* forced electrical angle in CTRL_CAL, 2^32 = 360 deg */
extern volatile uint8_t g_telem_en;		  /* 0 while a command is talking, so text and frames never mix */

/* step/decay recorder, filled by currentLoop() */
#define REC_N 600
extern volatile float g_rec[REC_N];
extern volatile uint16_t g_rec_n;
extern volatile uint8_t g_rec_state; /* 0 idle, 1 armed, 2 recording, 3 done */
extern volatile uint8_t g_rec_use_q; /* record iq instead of id */

/* defined elsewhere in your code: must be volatile float, defined exactly once */
extern volatile float g_id, g_iq, g_vin;
extern volatile float g_vq_f, g_iq_f; /* low-passed vq command and iq, from currentLoop() */

/* from currentLoop(): motor direction (positive = positive iq), relative to home */
extern volatile float g_vel;		   /* turn/s, PLL estimate */
extern volatile float g_pos;		   /* turns, display/telemetry only: the loop uses pos_q32_t */
extern volatile float g_rpm;		   /* g_vel * 60 */
extern volatile uint8_t g_home_req; /* set to 1: currentLoop() makes the present position 0 */

/* implement in main.c:  uint32_t enc_angle21(void) { return hMT6835.angle; } */
uint32_t enc_angle21(void);

/* electrical angle for normal (non-calibration) operation */
static inline uint32_t elec_angle(uint32_t angle21) {
	uint32_t m = (angle21 << 11) * (uint32_t)g_params.pole_pairs;
	return (g_params.enc_dir > 0 ? m : 0u - m) + g_params.elec_offset;
}

void cmd_init(void); /* call once after MX_USART1_UART_Init() */
void cmd_poll(void); /* call from the main loop */
void pi_apply(void); /* copy PI gains from g_params into the controllers; after params_load() and any change */

#endif
