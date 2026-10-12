#ifndef PI_CTRL_H
#define PI_CTRL_H

#include <math.h>

/* discrete PI, forward Euler integrator. ki_ts = ki * Ts, precomputed by pi_apply() */
typedef struct {
	float kp;
	float ki_ts;
	float integ; /* volts */
} pi_t;

/* defined in calib.c; gains written from main (pi_apply), integrators owned by currentLoop() */
extern volatile pi_t g_pi_d, g_pi_q;
extern volatile float g_id_ref, g_iq_ref; /* A */
extern volatile pi_t g_pi_vel;			  /* speed loop, output A, runs at OUTER_FS_HZ */
extern volatile float g_vel_ref;		  /* turn/s, LOOP_VEL */
/* LOOP_POS reference, turns from home. 64-bit writes are not atomic on the M4, so main hands it over:
 * write g_pos_ref_new, set g_pos_ref_req = 1, wait until currentLoop() clears it */
extern volatile int64_t g_pos_ref_new;
extern volatile uint8_t g_pos_ref_req;

static inline void pi_reset(volatile pi_t* c) {
	c->integ = 0.0f;
}

/* output limited to +-lim. Conditional integration: while saturated, the integrator only
 * moves if the error pulls the output back out of saturation */
static inline float pi_run(volatile pi_t* c, float err, float lim) {
	float i_new = c->integ + c->ki_ts * err;
	float u = c->kp * err + i_new;
	if (u > lim) {
		u = lim;
		if (err < 0.0f) c->integ = i_new;
	}
	else if (u < -lim) {
		u = -lim;
		if (err > 0.0f) c->integ = i_new;
	}
	else {
		c->integ = i_new;
	}
	/* a shrinking limit (bus sag, d taking more of the circle) must not leave a stale integrator */
	if (c->integ > lim) c->integ = lim;
	else if (c->integ < -lim) c->integ = -lim;
	return u;
}

/* d and q current loops sharing a voltage circle of radius vmax; d gets priority, q gets what is left */
static inline void pi_current_step(float id, float iq, float vmax, float* vd, float* vq) {
	float d = pi_run(&g_pi_d, g_id_ref - id, vmax);
	float r2 = vmax * vmax - d * d;
	float vq_max = r2 > 0.0f ? sqrtf(r2) : 0.0f;
	*vd = d;
	*vq = pi_run(&g_pi_q, g_iq_ref - iq, vq_max);
}

#endif
