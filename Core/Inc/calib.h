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
typedef enum { LOOP_CURRENT = 0, /* g_id_ref/g_iq_ref set by MSG_RUN_CUR */
			   LOOP_VEL,		  /* speed PI -> g_iq_ref */
			   LOOP_POS,		  /* position P -> speed PI -> g_iq_ref */
} loop_t;
extern volatile loop_t g_loop;
extern volatile float g_vd_cmd, g_vq_cmd; /* volts, used in CTRL_CAL */
extern volatile uint32_t g_theta_cmd;	  /* forced electrical angle in CTRL_CAL, 2^32 = 360 deg */

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
#define SL_LOST_N 1250 /* sensorless: HFI not tracking for this many cycles (50 ms) stops the motor */
#define SL_AGREE_MAX_TURNS 10.0f /* sensorless: above this speed (turn/s, ~600 rpm) HFI is not checked against the observer */
extern volatile uint16_t g_sl_lost_n; /* consecutive such cycles, from currentLoop() */

/* trips, detected in currentLoop(). In CTRL_RUN the ISR coasts at zero current (id = iq = 0, outer loops
 * held): no torque, no regen into the bus, no short-brake. Outside CTRL_RUN, or when the current keeps
 * rising anyway, it switches the bridge off (bridge_trip_off()). The watchdog ends the coast (CTRL_OFF)
 * below COAST_END_TURNS; STOP resets the trip after that */
enum { TRIP_NONE = 0,
	   TRIP_OV, /* g_vin above vbus_max for OV_TRIP_N cycles */
	   TRIP_OC, /* |id| or |iq| above max_current for OC_TRIP_N cycles */
};
#define OV_TRIP_N 25	   /* 1 ms */
#define OC_TRIP_N 3		   /* 120 us: rides through single-sample spikes */
#define OC_HARD 1.5f	   /* x max_current during the coast: the loop lost control, bridge off */
#define OC_LOST_N 125	   /* ...or still above max_current this many cycles (5 ms) into the coast */
#define COAST_END_TURNS 0.5f /* turn/s (30 rpm): the coast ends, the zero vector is harmless from here */
extern volatile uint8_t g_trip; /* TRIP_*, set by currentLoop(), cleared by STOP (or by itself, see below) */
extern volatile uint8_t g_coasting; /* a stop is coasting at zero current in CTRL_RUN until COAST_END_TURNS */
/* sensorless: after the bridge went off (break, lost HFI) it is re-armed only after this long, since the speed
 * cannot be measured without it and re-arming a spinning motor short-brakes it. Encoder modes wait for
 * |speed| < COAST_END_TURNS instead. The first re-arm after boot does not wait */
#define SL_REARM_MS 3000

/* regen clamp: in CTRL_RUN, braking current (iq against the speed) is scaled down linearly from full at
 * vbus_max - vbus_clamp (param) to zero at vbus_max, so a deceleration only goes as fast as the bus can take
 * the energy. Motoring current is never limited. The OV trip stays as the backstop (external drive, a step) */
/* an OV coast (bridge still on, CTRL_RUN) resumes by itself after the bus has been below
 * vbus_max - vbus_resume (param) for OV_RESUME_N cycles; the references are restored. Overcurrent never resumes */
#define OV_RESUME_N 250 /* 10 ms */
extern volatile uint32_t g_ov_resumes; /* count of those resumes, object 0x4002:11 (write 0 clears) */
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
