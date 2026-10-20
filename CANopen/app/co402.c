#include "co402.h"

#include <math.h>
#include <string.h>

#include "CANopen.h"
#include "OD.h"
#include "calib.h"
#include "co_app.h"
#include "objdict.h" /* od_write(): the parameter views go through the same checks as the USB protocol */
#include "pi_ctrl.h" /* g_vel_ref, g_iq_ref */

#define INC_PER_TURN 65536.0f /* 0x608F: 2^16 increments per motor revolution, Q16.16 turns (g_pos_cnt) */
#define INC_TO_Q32 65536	  /* increments -> Q32.32 turns */
#define TICK_S 0.001f
#define STOP_WAIT_MS 1000 /* after a stop ramp reached zero: coast anyway if the motor is not slow by then */

/* statusword bits (CiA 402) */
#define SW_VE 0x0010u	  /* voltage enabled */
#define SW_WARN 0x0080u	  /* warning */
#define SW_REMOTE 0x0200u /* controlword is processed */
#define SW_REACHED 0x0400u
#define SW_LIMIT 0x0800u  /* internal limit active */
#define SW_OMS12 0x1000u  /* cyclic modes: drive follows the command value */
#define SW_OMS13 0x2000u  /* position modes: following error */

volatile uint8_t g_co402_state = FSA_NOT_READY;
volatile uint8_t g_co402_refused;
volatile uint8_t g_co402_fault;

/* main -> tick */
static volatile int8_t s_mode;	   /* mode in effect (0x6061) */
static volatile uint8_t s_stream;  /* the tick writes references for s_mode */
static volatile uint8_t s_ramp;	   /* stop ramp in the tick: 0 none, 1 profile deceleration (6084), 2 quick stop (6085) */
static volatile uint8_t s_warn;	   /* statusword warning: the last switch on / enable was refused */
/* tick -> main */
static volatile float s_rv;		  /* speed reference of the ramp, turn/s */
static volatile uint8_t s_ramp_zero; /* the ramp reached zero */
static volatile uint8_t s_limited;	 /* the last reference was clamped */
static volatile uint8_t s_ferr;		 /* following error for longer than 0x6066 */
/* CSP: the tick's own */
static int64_t s_act64;		  /* actual position, counts, unwrapped from the int32 g_pos_cnt */
static int32_t s_cnt_prev;
static int64_t s_tgt64;		  /* last target handed to the ISR, counts */
static int32_t s_tgt_last32;  /* ...as the master sent it (0x607A) */
static volatile uint8_t s_tgt_rx; /* 0x607A was written (RPDO or SDO) since the last tick, unchanged values too */
static volatile uint8_t s_mode_init; /* main: the mode (re)starts, from the present position and speed */
static uint16_t s_ferr_ms;
/* PV, HM, PP: the tick's own */
static uint16_t s_cw_tick;	/* controlword at the last tick: bit 4 edges */
static uint16_t s_win_ms;	/* time inside the velocity (PV) or position (PP) window */
static uint16_t s_thr_ms;	/* PV: time below the velocity threshold */
static float s_vf;			/* actual speed through a 10 ms low-pass, inc/s: the PV window and threshold */
static volatile uint8_t s_hm; /* HM: 0 not started, 1 homing, 2 attained, 3 error */
static int64_t s_pp_pos;	/* PP profile position, counts (+ s_pp_frac) */
static float s_pp_frac;
static float s_pp_v;		/* PP profile velocity, counts/s */
static int64_t s_pp_tgt;	/* PP target in progress */
static int64_t s_pp_next;	/* PP buffered set-point */
static uint8_t s_pp_busy, s_pp_pend, s_pp_ack;
static uint8_t s_csp_halt; /* CSP: halted, the PP profile brings the reference to rest */
static float s_csp_v;	   /* CSP: rate of the last segment, counts/s (where a halt starts braking from) */
/* main only */
enum { AFTER_SO = 1, AFTER_RTSO, AFTER_SOD, AFTER_HOLD };
static uint8_t s_after; /* where a stop ramp ends: switched on, ready, switch on disabled, or hold in quick stop */
static uint32_t s_zero_ms;
static uint16_t s_cw_prev;
static uint8_t s_st_prev;
static uint8_t s_nmt_op;  /* NMT was operational while the master had the motor */
static uint8_t s_hb_prev; /* heartbeat consumer timeout seen */
static uint8_t s_fault_bit;
static int8_t s_want_refused; /* 6060 value a mode change in operation was refused for, 0 = none */

/* ------------------------------------------------------------------ faults -> 0x603F, EMCY, 0x1001 */
static const struct {
	uint8_t st, bit; /* bit: CANopenNode error status bit (0x40..: see CO_CONFIG_ERR_CONDITION_* in CO_driver_target.h) */
	uint16_t code;	 /* CiA 301 / 402 error code */
} k_fault[] = {
	{ST_OVERCURRENT, 0x40, 0x2310},	 /* continuous over current */
	{ST_VBUS_HIGH, 0x41, 0x3210},	 /* DC link over-voltage */
	{ST_VBUS_LOW, 0x42, 0x3220},	 /* DC link under-voltage */
	{ST_BRIDGE_FAULT, 0x43, 0x5400}, /* power section (gate driver fault, break) */
	{ST_OVERSPEED, 0x44, 0x8400},	 /* velocity speed controller */
	{ST_HFI_LOST, 0x45, 0x7300},	 /* sensor (sensorless angle lost) */
	{ST_HFI_POL_WEAK, 0x45, 0x7300},
	{ST_ENC_LOST, 0x45, 0x7305}, /* incremental sensor 1 fault */
	{ST_HALL_LOST, 0x45, 0x7306}, /* incremental sensor 2 fault (the Halls) */
	{ST_ABORTED, 0x47, 0xFF01},	 /* manufacturer: stopped from the USB/serial link */
	{ST_CAN_LOST, 0x48, 0x8130}, /* heartbeat / life guard error: master lost */
	{ST_FOLLOWING_ERROR, 0x46, 0x8611},
};
#define FAULT_OTHER_BIT 0x49
#define FAULT_OTHER_CODE 0x1000 /* generic error */

static CO_EM_t* em(void) {
	CO_t* co = co_app_co();
	return co ? co->em : NULL;
}

static loop_t loop_of(int8_t mode) {
	return mode == MODE_CST ? LOOP_CURRENT : (mode == MODE_CSP || mode == MODE_PP) ? LOOP_POS : LOOP_VEL;
}

static int supported(int8_t mode) {
	return mode == MODE_PP || mode == MODE_PV || mode == MODE_HM || mode == MODE_CSP || mode == MODE_CSV || mode == MODE_CST;
}

/* the master lets go: no more references, the speed ramp back to acc_lim, USB may run again */
static void release(void) {
	s_stream = 0;
	s_ramp = 0;
	g_ramp_bypass = 0;
	g_can_owner = 0;
}

/* zero current now (CTRL_OFF once slow: never the zero vector at speed) */
static void stop_now(void) {
	s_stream = 0;
	s_ramp = 0;
	if (g_mode == CTRL_RUN && !g_coasting && !g_trip) drive_coast();
}

static void fault(uint8_t st) {
	uint8_t bit = FAULT_OTHER_BIT;
	uint16_t code = FAULT_OTHER_CODE;
	for (unsigned i = 0; i < sizeof k_fault / sizeof k_fault[0]; i++)
		if (k_fault[i].st == st) {
			bit = k_fault[i].bit;
			code = k_fault[i].code;
		}
	stop_now(); /* the watchdog has coasted already when it reported this one */
	g_co402_fault = st;
	OD_RAM.x603F_errorCode = code;
	s_fault_bit = bit;
	if (em()) CO_errorReport(em(), bit, code, st);
	g_co402_state = FSA_FAULT_REACTION_ACTIVE;
}

void co402_comm_reset(void) {
	/* the reset re-initialised the emergency object: an active drive fault would be gone from 0x1001 / 0x1003 */
	if (s_fault_bit && em()) CO_errorReport(em(), s_fault_bit, OD_RAM.x603F_errorCode, g_co402_fault);
}

/* ramp the speed to zero in the tick, then go to `after`. Needs the speed loop: from CST it switches over
 * bumplessly, or coasts if the speed gains are not set */
static void start_ramp(int quick, uint8_t after) {
	if (g_loop != LOOP_VEL && drive_enter(LOOP_VEL) != ST_OK) {
		stop_now();
		g_co402_state = after == AFTER_SO ? FSA_SWITCHED_ON : after == AFTER_RTSO ? FSA_READY_TO_SWITCH_ON : FSA_SWITCH_ON_DISABLED;
		return;
	}
	s_rv = g_vel_ref;
	s_ramp_zero = 0;
	s_zero_ms = 0;
	s_after = after;
	s_ramp = quick ? 2 : 1;
}

/* quick stop command or reaction, 0x605A */
static void quick_stop(void) {
	const int16_t o = OD_PERSIST_APP.x605A_quickStopOptionCode;
	if (o <= 0) {
		stop_now();
		g_co402_state = FSA_SWITCH_ON_DISABLED;
		return;
	}
	g_co402_state = FSA_QUICK_STOP_ACTIVE;
	const int quick = !(o == 1 || o == 5); /* 1, 5: profile deceleration; 2, 6 (and the current-limit ones): quick stop */
	start_ramp(quick, o >= 5 ? AFTER_HOLD : AFTER_SOD);
}

static void refuse(st_t e) {
	g_co402_refused = (uint8_t)e;
	s_warn = 1;
}

/* ------------------------------------------------------------------ main loop */
void co402_poll(void) {
	uint8_t st = g_co402_state;
	const uint16_t cw = OD_RAM.x6040_controlword;
	const int fr_edge = (cw & 0x80) && !(s_cw_prev & 0x80);
	const int cw_new = cw != s_cw_prev;
	const int fresh = cw_new || st != s_st_prev; /* switch on / enable: tried once per command or state */
	s_cw_prev = cw;
	s_st_prev = st; /* the state this poll started in: "switch on + enable" in one command takes two polls */
	const int b7 = (cw & 0x80) != 0;
	const int dv = !b7 && !(cw & 0x02);					/* disable voltage */
	const int qs = !b7 && (cw & 0x06) == 0x02;			/* quick stop */
	const int sd = !b7 && (cw & 0x07) == 0x06;			/* shutdown */
	const int so = !b7 && (cw & 0x0F) == 0x07;			/* switch on / disable operation */
	const int eo = !b7 && (cw & 0x0F) == 0x0F;			/* enable operation (also switch on + enable) */
	const int remote = !(g_busy || (g_mode != CTRL_OFF && !g_can_owner));
	const int8_t want = OD_RAM.x6060_modesOfOperation;

	/* stops the master did not ask for: watchdog trips and stops, a local STOP, a lost master */
	const uint8_t why = g_stop_reason;
	if (why) g_stop_reason = 0;
	if (st == FSA_OPERATION_ENABLED || st == FSA_QUICK_STOP_ACTIVE) {
		const uint8_t nmt = co_app_nmt_state();
		const uint8_t hb = em() && CO_isError(em(), CO_EM_HEARTBEAT_CONSUMER);
		const int lost = (s_nmt_op && nmt != 5) || (hb && !s_hb_prev); /* NMT left operational, or heartbeat timeout */
		s_hb_prev = hb;
		if (nmt == 5) s_nmt_op = 1; /* enabled in pre-operational, then started: a later drop (bus-off) counts too */
		if (!g_can_owner) g_ramp_bypass = 0; /* a local STOP took it: a USB run that follows slews at acc_lim again */
		if (why)
			fault(why);
		else if (!g_can_owner)
			fault(ST_ABORTED); /* a local STOP took the motor */
		else if (g_mode != CTRL_RUN)
			fault(ST_LOOP_DEAD);
		else if (s_ferr) {
			s_ferr = 0;
			fault(ST_FOLLOWING_ERROR);
		}
		else if (lost) {
			s_nmt_op = 0;
			switch (OD_PERSIST_APP.x6007_abortConnectionOptionCode) { /* 0x6007 */
				case 0: break;
				case 2: stop_now(); g_co402_state = FSA_SWITCH_ON_DISABLED; break;
				case 3: if (st == FSA_OPERATION_ENABLED) quick_stop(); break;
				default: fault(ST_CAN_LOST); break;
			}
		}
		st = g_co402_state;
	}
	else if (g_trip && st != FSA_FAULT_REACTION_ACTIVE && st != FSA_FAULT) {
		/* a trip outside operation (the coast after disable voltage, an overvoltage with the bridge idle): only a fault
		 * reset clears it over CANopen, so it must show as a fault, or every switch on is refused with ST_TRIPPED */
		fault(why ? why : g_trip == TRIP_OV ? ST_VBUS_HIGH : ST_OVERCURRENT);
		st = g_co402_state;
	}

	/* fault reaction: the coast is the reaction; Fault once the motor is off */
	if (st == FSA_FAULT_REACTION_ACTIVE && g_mode == CTRL_OFF) {
		release();
		st = g_co402_state = FSA_FAULT;
	}
	if (fr_edge) s_warn = 0;

	switch (st) {
		case FSA_NOT_READY:
			st = FSA_SWITCH_ON_DISABLED; /* nothing to initialise: the motor control is up before CANopen */
			break;
		case FSA_SWITCH_ON_DISABLED:
			if (sd) st = FSA_READY_TO_SWITCH_ON;
			break;
		case FSA_READY_TO_SWITCH_ON:
			if (dv || qs)
				st = FSA_SWITCH_ON_DISABLED;
			else if ((so || eo) && fresh) {
				const st_t e = !remote ? ST_RUNNING : !supported(want) ? ST_BAD_ARG : drive_check(loop_of(want));
				if (e)
					refuse(e);
				else {
					g_co402_refused = 0;
					s_warn = 0;
					st = FSA_SWITCHED_ON;
				}
			}
			break;
		case FSA_SWITCHED_ON:
			if (dv || qs)
				st = FSA_SWITCH_ON_DISABLED;
			else if (sd)
				st = FSA_READY_TO_SWITCH_ON;
			else if (eo && fresh) {
				const st_t e = !remote ? ST_RUNNING : !supported(want) ? ST_BAD_ARG : drive_enter(loop_of(want));
				if (e)
					refuse(e);
				else {
					g_co402_refused = 0;
					s_warn = 0;
					g_can_owner = 1;
					g_ramp_bypass = 1;
					s_mode = want;
					s_rv = g_vel_ref;
					s_ramp = 0;
					s_mode_init = 1;
					s_ferr = 0;
					s_want_refused = 0;
					s_stream = 1;
					s_nmt_op = co_app_nmt_state() == 5;
					s_hb_prev = em() && CO_isError(em(), CO_EM_HEARTBEAT_CONSUMER);
					st = FSA_OPERATION_ENABLED;
				}
			}
			break;
		case FSA_OPERATION_ENABLED:
			if (dv) {
				stop_now();
				st = FSA_SWITCH_ON_DISABLED;
			}
			else if (qs) {
				quick_stop();
				st = g_co402_state;
			}
			else if (sd && !s_ramp) {
				if (OD_PERSIST_APP.x605B_shutdownOptionCode >= 1)
					start_ramp(0, AFTER_RTSO);
				else {
					stop_now();
					st = FSA_READY_TO_SWITCH_ON;
				}
				if (g_co402_state != FSA_OPERATION_ENABLED) st = g_co402_state; /* the ramp could not start */
			}
			else if (so && !s_ramp) {
				if (OD_PERSIST_APP.x605C_disableOperationOptionCode >= 1)
					start_ramp(0, AFTER_SO);
				else {
					stop_now();
					st = FSA_SWITCHED_ON;
				}
				if (g_co402_state != FSA_OPERATION_ENABLED) st = g_co402_state;
			}
			else if (eo) {
				if (s_ramp) { /* the master changed its mind during a disable / shutdown ramp: back to the mode */
					/* no references until the mode has restarted: once drive_enter() has switched the loop, the tick
					   would stream from stale CSP / PP state (a PP profile left where the ramp began) */
					s_stream = 0;
					if (g_loop == loop_of(s_mode) || drive_enter(loop_of(s_mode)) == ST_OK) {
						s_ramp = 0;
						s_rv = g_vel_ref;
						s_mode_init = 1;
					} /* else the ramp carries on: cancelled without its loop it would hold the speed of the moment */
					s_stream = 1;
				}
				if (want != s_want_refused) s_want_refused = 0; /* 6060 changed since: the next value is tried */
				if (!s_ramp && want != s_mode && supported(want) && want != s_want_refused) {
					s_stream = 0; /* switch the loop bumplessly, then stream the new mode */
					const st_t e = drive_enter(loop_of(want));
					if (e == ST_OK) {
						s_rv = g_vel_ref;
						s_mode_init = 1;
						s_mode = want;
						g_co402_refused = 0;
						s_warn = 0;
					}
					else { /* once per 6060 value: the old mode carries on, 6061 shows it, warning + 0x4042:2 say why */
						refuse(e);
						s_want_refused = want;
					}
					s_stream = 1;
				}
			}
			break;
		case FSA_QUICK_STOP_ACTIVE:
			if (dv) {
				stop_now();
				st = FSA_SWITCH_ON_DISABLED;
			}
			/* transition 16: options 5..8 may go back to operation, on a new controlword only: after a quick stop for a
			   lost master (6007 = 3) its last one is still enable operation, and must not undo the stop */
			else if (eo && cw_new && s_after == AFTER_HOLD) {
				s_stream = 0; /* as above: nothing streamed from stale state while the loop switches */
				if (g_loop == loop_of(s_mode) || drive_enter(loop_of(s_mode)) == ST_OK) {
					s_ramp = 0;
					s_rv = g_vel_ref;
					s_mode_init = 1;
					st = FSA_OPERATION_ENABLED;
				} /* else it stays in quick stop, holding zero speed */
				s_stream = 1;
			}
			break;
		case FSA_FAULT:
			if (fr_edge && drive_trip_reset()) {
				if (em() && s_fault_bit) CO_errorReset(em(), s_fault_bit, 0);
				s_fault_bit = 0;
				g_co402_fault = 0;
				OD_RAM.x603F_errorCode = 0;
				st = FSA_SWITCH_ON_DISABLED;
			}
			break;
		default:
			break;
	}
	g_co402_state = st;

	/* a stop ramp reached zero: hold (quick stop options 5..8) or coast and finish the transition */
	if (s_ramp && s_ramp_zero) {
		const uint32_t now = HAL_GetTick();
		if (!s_zero_ms) s_zero_ms = now ? now : 1;
		if (s_after != AFTER_HOLD && (fabsf(g_vel) < COAST_END_TURNS || now - s_zero_ms > STOP_WAIT_MS)) {
			stop_now();
			g_co402_state = s_after == AFTER_SO ? FSA_SWITCHED_ON : s_after == AFTER_RTSO ? FSA_READY_TO_SWITCH_ON : FSA_SWITCH_ON_DISABLED;
		}
	}

	/* the master has the motor until it is off again */
	st = g_co402_state;
	if (g_can_owner && st != FSA_OPERATION_ENABLED && st != FSA_QUICK_STOP_ACTIVE && st != FSA_FAULT_REACTION_ACTIVE && g_mode == CTRL_OFF)
		release();
}

/* ------------------------------------------------------------------ CANopen tick */
static int16_t sat16(float x) {
	return x > 32767.0f ? 32767 : x < -32768.0f ? -32768 : (int16_t)x;
}

static float clampf(float x, float lim, uint8_t* hit) {
	if (x > lim) {
		*hit = 1;
		return lim;
	}
	if (x < -lim) {
		*hit = 1;
		return -lim;
	}
	return x;
}

/* 0x60C2: value x 10^index s, the period CSP interpolates each new target over */
static float interp_period(void) {
	float t = (float)OD_PERSIST_APP.x60C2_interpolationTimePeriod.interpolationTimePeriodValue;
	for (int8_t i = OD_PERSIST_APP.x60C2_interpolationTimePeriod.interpolationTimeIndex; i < 0; i++) t *= 0.1f;
	for (int8_t i = OD_PERSIST_APP.x60C2_interpolationTimePeriod.interpolationTimeIndex; i > 0; i--) t *= 10.0f;
	return t < 0.0004f ? 0.001f : t > 0.1f ? 0.1f : t; /* 0 or out of range: 1 ms */
}

/* position modes: a segment for the ISR, ending at the demand s_tgt64 + dc after T. The ISR goes there from where
 * its reference is, so a segment that comes early or late stays continuous, and carries on at the same rate for up
 * to one more period while the next one is late (motion_update()). dc = 0 is "hold here": PP posts it every tick at
 * rest, CSP for every target received, so neither coasts past its end. Returns 0 while the ISR has not taken the
 * last one yet (try again next tick) */
static int pos_step(int32_t dc, float T, uint8_t* lim) {
	if (g_pos_ref_req) return 0;
	g_pos_ref_new = (s_tgt64 + dc) * INC_TO_Q32; /* the segment's end, counts -> Q32.32 */
	const uint16_t n = (uint16_t)(T * CTRL_FS_HZ + 0.5f);
	const float ff = (float)dc * (1.0f / INC_PER_TURN) / T;
	if (fabsf(ff) > g_params.vel_lim * 1.001f) *lim = 1; /* the speed loop clamps it to vlim */
	g_pos_ff_new = ff;
	g_pos_ff_q_new = (int64_t)dc * INC_TO_Q32 / n; /* per cycle; the ISR recomputes it from where its reference is */
	g_pos_ff_n_new = n;
	g_pos_ref_req = 1;
	s_tgt64 += dc;
	return 1;
}

/* PP: one tick of the trapezoid profile (0x6086 = 0) to s_pp_tgt, at 6081 (<= 607F, vlim), 6083 up, 6084 down.
 * Replans every tick from where it is, so a set-point changed on the way (bit 5) is just a new target. Halt:
 * down to zero at 6084 (605D = 2: 6085) and hold; released, it carries on to the target */
static void pp_profile(int halt) {
	float vmax = fminf((float)OD_PERSIST_APP.x6081_profileVelocity, (float)OD_PERSIST_APP.x607F_maxProfileVelocity);
	vmax = fminf(vmax, g_params.vel_lim * INC_PER_TURN); /* counts/s */
	const float acc = fmaxf((float)OD_PERSIST_APP.x6083_profileAcceleration, 1.0f);
	const float dec = fmaxf((float)(halt && OD_PERSIST_APP.x605D_haltOptionCode == 2 ? OD_PERSIST_APP.x6085_quickStopDeceleration
																					: OD_PERSIST_APP.x6084_profileDeceleration),
							1.0f);
	const float d = (float)(s_pp_tgt - s_pp_pos) - s_pp_frac; /* counts to go */
	float v = s_pp_v, vt = 0.0f;
	if (!halt && s_pp_busy) vt = copysignf(fminf(vmax, sqrtf(2.0f * dec * fabsf(d))), d); /* stops at the target at dec */
	if (v * vt > 0.0f && fabsf(v) > fabsf(vt)) { /* slower, same direction */
		if (fabsf(v) - fabsf(vt) <= 2.0f * dec * TICK_S)
			v = vt; /* on the braking curve */
		else
			v -= copysignf(dec * TICK_S, v); /* too fast for it (a nearer set-point, a lower 6081): overshoot */
	}
	else { /* faster, or turning round: slow down first at dec */
		const float a = (v * vt < 0.0f || vt == 0.0f) ? dec : acc;
		v = vt > v + a * TICK_S ? v + a * TICK_S : vt < v - a * TICK_S ? v - a * TICK_S : vt;
	}
	if (!halt && s_pp_busy && fabsf(d) <= fmaxf(fabsf(v) * TICK_S, 0.5f) && fabsf(v) <= 2.0f * dec * TICK_S + 1.0f) {
		s_pp_pos = s_pp_tgt; /* there */
		s_pp_frac = 0.0f;
		s_pp_v = 0.0f;
		s_pp_busy = s_pp_pend;
		s_pp_tgt = s_pp_pend ? s_pp_next : s_pp_tgt; /* the buffered set-point starts */
		s_pp_pend = 0;
		return;
	}
	const float f = s_pp_frac + v * TICK_S;
	const int32_t i = (int32_t)f;
	s_pp_frac = f - (float)i;
	s_pp_pos += i;
	s_pp_v = v;
}

static const uint16_t k_sw_state[] = {
	[FSA_NOT_READY] = 0x0000,
	[FSA_SWITCH_ON_DISABLED] = 0x0040,
	[FSA_READY_TO_SWITCH_ON] = 0x0021,
	[FSA_SWITCHED_ON] = 0x0023,
	[FSA_OPERATION_ENABLED] = 0x0027,
	[FSA_QUICK_STOP_ACTIVE] = 0x0007,
	[FSA_FAULT_REACTION_ACTIVE] = 0x000F,
	[FSA_FAULT] = 0x0008,
};

void co402_tick(void) {
	const float rated = g_params.rated_current;
	const float pm = rated > 0.0f ? 1000.0f / rated : 0.0f; /* A -> per mille of rated */
	const uint8_t st = g_co402_state;
	const uint16_t cw = OD_RAM.x6040_controlword;
	const int halt = (cw & 0x0100) && st == FSA_OPERATION_ENABLED;

	/* actual values */
	const int32_t cnt = g_pos_cnt;
	const int32_t ref = g_pos_ref_cnt; /* read together: at most one ISR cycle apart */
	s_act64 += (int32_t)((uint32_t)cnt - (uint32_t)s_cnt_prev); /* int32 delta: unwraps past +-32768 turns */
	s_cnt_prev = cnt;
	OD_RAM.x6064_positionActualValue = cnt;
	OD_RAM.x606C_velocityActualValue = (int32_t)(g_vel * INC_PER_TURN);
	OD_RAM.x6077_torqueActualValue = OD_RAM.x6078_currentActualValue = sat16(g_iq_f * pm);
	OD_RAM.x6074_torqueDemand = sat16(g_iq_ref * pm);
	OD_RAM.x6079_DCLinkCircuitVoltage = g_vin > 0.0f ? (uint32_t)(g_vin * 1000.0f) : 0;

	/* references: only while main has the motor armed for CANopen and nothing else took it */
	uint8_t lim = 0, b12 = 0, b13 = 0, reached = 0;
	const int run = s_stream && g_can_owner && g_mode == CTRL_RUN && !g_coasting && !g_trip;
	const int nsp = (cw & 0x10) && !(s_cw_tick & 0x10); /* bit 4 rising: new set-point (PP), start homing (HM) */
	s_cw_tick = cw;
	if (run && s_mode_init) { /* the mode (re)starts: the ISR holds the present position (drive_enter) */
		s_tgt64 = s_pp_pos = s_pp_tgt = s_act64;
		s_tgt_last32 = (int32_t)s_act64;
		s_tgt_rx = 0; /* only a target written from now on moves it: not one left over in 607A */
		s_pp_frac = s_pp_v = 0.0f;
		s_pp_busy = s_pp_pend = s_pp_ack = 0;
		s_csp_halt = 0;
		s_csp_v = 0.0f;
		s_ferr_ms = s_win_ms = s_thr_ms = 0;
		s_hm = 0;
		s_mode_init = 0;
	}
	if (s_hm == 1 && !g_home_req) { /* the ISR took the new home: the position jumped, start over from it */
		s_cnt_prev = g_pos_cnt;
		s_act64 = s_tgt64 = s_pp_pos = s_pp_tgt = s_cnt_prev;
		s_tgt_last32 = s_cnt_prev;
		s_hm = 2;
	}
	if (run && g_loop == LOOP_VEL && (s_ramp || halt)) {
		const int quick = s_ramp == 2 || (!s_ramp && OD_PERSIST_APP.x605D_haltOptionCode == 2);
		const float dv = (float)(quick ? OD_PERSIST_APP.x6085_quickStopDeceleration : OD_PERSIST_APP.x6084_profileDeceleration) *
						 (TICK_S / INC_PER_TURN);
		float v = s_rv;
		if (v > dv)
			v -= dv;
		else if (v < -dv)
			v += dv;
		else {
			v = 0.0f;
			s_ramp_zero = 1;
		}
		s_rv = v;
		g_vel_ref = v;
		reached = v == 0.0f && fabsf(g_vel) < COAST_END_TURNS;
	}
	else if (run && (s_mode == MODE_CSV || s_mode == MODE_PV || s_mode == MODE_HM) && g_loop == LOOP_VEL) {
		const float vmax = s_mode == MODE_CSV ? g_params.vel_lim : fminf(g_params.vel_lim, (float)OD_PERSIST_APP.x607F_maxProfileVelocity * (1.0f / INC_PER_TURN));
		if (s_mode == MODE_CSV) { /* the master's value as it is */
			/* summed in float: in int32 a large target plus an offset could wrap and turn round at vlim */
			const float v = ((float)OD_RAM.x60FF_targetVelocity + (float)OD_RAM.x60B1_velocityOffset) * (1.0f / INC_PER_TURN);
			g_vel_ref = s_rv = clampf(v, vmax, &lim);
			b12 = 1;
		}
		else { /* PV: ramp to 0x60FF at 6083 (faster) / 6084 (slower). HM: hold zero speed, ramp at 609A */
			const float vt = s_mode == MODE_HM ? 0.0f : clampf((float)OD_RAM.x60FF_targetVelocity * (1.0f / INC_PER_TURN), vmax, &lim);
			float v = s_rv;
			const uint32_t a = s_mode == MODE_HM ? OD_PERSIST_APP.x609A_homingAcceleration
							 : (v * vt >= 0.0f && fabsf(vt) > fabsf(v)) ? OD_PERSIST_APP.x6083_profileAcceleration
																		  : OD_PERSIST_APP.x6084_profileDeceleration;
			const float dv = (float)a * (TICK_S / INC_PER_TURN);
			v = vt > v + dv ? v + dv : vt < v - dv ? v - dv : vt;
			g_vel_ref = s_rv = v;
		}
		if (s_mode == MODE_HM && nsp) {
			const int8_t m = OD_PERSIST_APP.x6098_homingMethod;
			if (m == 35 || m == 37) { /* home here: the present position becomes -0x607C */
				g_home_pos = -OD_PERSIST_APP.x607C_homeOffset;
				s_hm = 1;
				g_home_req = 1;
			}
			else
				s_hm = 3;
		}
	}
	else if (run && (s_mode == MODE_CSP || s_mode == MODE_PP) && g_loop == LOOP_POS) {
		const int32_t lo = OD_PERSIST_APP.x607D_softwarePositionLimit[0], hi = OD_PERSIST_APP.x607D_softwarePositionLimit[1];
		int32_t t = OD_RAM.x607A_targetPosition;
		if (s_mode == MODE_CSP && halt) {
			/* halt: brake the reference from the master's rate to rest at 6084 (605D = 2: 6085) with the PP profile,
			   and hold there; the targets meanwhile are ignored */
			if (!s_csp_halt) {
				s_csp_halt = 1;
				s_pp_pos = s_tgt64;
				s_pp_frac = 0.0f;
				s_pp_v = s_csp_v;
				s_pp_busy = s_pp_pend = 0;
			}
			pp_profile(1);
			pos_step((int32_t)(s_pp_pos - s_tgt64), TICK_S, &lim);
			reached = s_pp_v == 0.0f;
		}
		else if (s_mode == MODE_CSP) {
			if (s_csp_halt) { /* released: like a mode start, only targets written from now on move it, relative to here */
				s_csp_halt = 0;
				s_csp_v = 0.0f;
				s_tgt_last32 = (int32_t)s_tgt64;
				s_tgt_rx = 0;
			}
			if (lo < hi && (t < lo || t > hi)) {
				t = t < lo ? lo : hi;
				lim = 1;
			}
			const int32_t dc = (int32_t)((uint32_t)t - (uint32_t)s_tgt_last32);
			/* every target received is a segment over one period, an unchanged one too (hold). None this tick: the
			   ISR carries on at the last rate, a late frame or the 1 ms tick grid must not stop the reference */
			const float T = interp_period();
			if (s_tgt_rx && pos_step(dc, T, &lim)) {
				s_tgt_last32 = t;
				s_tgt_rx = 0;
				s_csp_v = (float)dc / T;
			}
			b12 = 1;
		}
		else {
			if (nsp && !s_pp_pend) { /* take the set-point: now (bit 5, or idle), or after the move in progress */
				int64_t p = (cw & 0x40) ? s_pp_tgt + t /* relative to the last target */
										: s_act64 + (int32_t)((uint32_t)t - (uint32_t)s_act64);
				if (lo < hi && (p < lo || p > hi)) p = p < lo ? lo : hi;
				if ((cw & 0x20) || !s_pp_busy) {
					s_pp_tgt = p;
					s_pp_busy = 1;
				}
				else {
					s_pp_next = p;
					s_pp_pend = 1;
				}
				s_pp_ack = 1;
			}
			if (!(cw & 0x10) && !s_pp_pend) s_pp_ack = 0;
			pp_profile(halt);
			pos_step((int32_t)(s_pp_pos - s_tgt64), TICK_S, &lim);
			b12 = s_pp_ack;
			if (halt)
				reached = s_pp_v == 0.0f;
			else {
				const int64_t e = s_pp_tgt - s_act64;
				const uint32_t w = OD_PERSIST_APP.x6067_positionWindow;
				s_win_ms = !s_pp_busy && (w == 0xFFFFFFFFu || (e <= (int64_t)w && e >= -(int64_t)w)) ? (s_win_ms < 0xFFFF ? s_win_ms + 1 : s_win_ms) : 0;
				reached = s_win_ms > OD_PERSIST_APP.x6068_positionWindowTime;
			}
		}
	}
	else if (run && s_mode == MODE_CST && g_loop == LOOP_CURRENT) {
		float iq = halt ? 0.0f : (float)(OD_RAM.x6071_targetTorque + OD_RAM.x60B2_torqueOffset) * rated * 0.001f;
		float imax = (float)OD_PERSIST_APP.x6072_maxTorque * rated * 0.001f;
		if (imax > g_params.max_current) imax = g_params.max_current;
		g_iq_ref = clampf(iq, imax, &lim);
		b12 = !halt;
		reached = halt;
	}
	s_limited = lim;

	/* PV: target reached (0x606D/0x606E), speed = 0 (0x606F/0x6070). HM: the homing status */
	s_vf += ((float)OD_RAM.x606C_velocityActualValue - s_vf) * (TICK_S / 0.01f);
	if (s_mode == MODE_PV) {
		const int32_t va = (int32_t)s_vf;
		const int64_t e = (int64_t)OD_RAM.x60FF_targetVelocity - va;
		const int64_t vw = OD_PERSIST_APP.x606D_velocityWindow, vth = OD_PERSIST_APP.x606F_velocityThreshold;
		s_win_ms = (e <= vw && e >= -vw) ? (s_win_ms < 0xFFFF ? s_win_ms + 1 : s_win_ms) : 0;
		s_thr_ms = ((int64_t)va <= vth && (int64_t)va >= -vth) ? (s_thr_ms < 0xFFFF ? s_thr_ms + 1 : s_thr_ms) : 0;
		if (!halt) reached = s_win_ms > OD_PERSIST_APP.x606E_velocityWindowTime;
		b12 = s_thr_ms > OD_PERSIST_APP.x6070_velocityThresholdTime;
	}
	else if (s_mode == MODE_HM) { /* 13/12/10: 001 not started or interrupted, 000 homing, 011 done, 101 error */
		reached = s_hm != 1;
		b12 = s_hm == 2;
		b13 = s_hm == 3;
	}

	/* position demand and following error (CSP, PP) */
	const int pos = run && (s_mode == MODE_CSP || s_mode == MODE_PP) && g_loop == LOOP_POS;
	/* the demand is the ISR's interpolated reference, not the end of the segment (s_tgt64): that one leads by up to
	   a whole 0x60C2 period of travel, which at speed alone overran the window */
	const int64_t dem = pos ? s_act64 + (int32_t)((uint32_t)ref - (uint32_t)cnt) : s_act64;
	const int64_t fe = dem - s_act64;
	OD_RAM.x6062_positionDemandValue = (int32_t)dem;
	OD_RAM.x60F4_followingErrorActualValue = fe > INT32_MAX ? INT32_MAX : fe < INT32_MIN ? INT32_MIN : (int32_t)fe;
	const uint32_t win = OD_PERSIST_APP.x6065_followingErrorWindow;
	const int over = pos && win != 0xFFFFFFFFu && (fe > (int64_t)win || fe < -(int64_t)win);
	if (!over)
		s_ferr_ms = 0;
	else if (++s_ferr_ms > OD_PERSIST_APP.x6066_followingErrorTimeOut)
		s_ferr = 1; /* main faults */
	if (over) b13 = 1;
	OD_RAM.x606B_velocityDemandValue = g_loop == LOOP_VEL && g_mode == CTRL_RUN ? (int32_t)(g_vel_ref * INC_PER_TURN) : 0;

	/* statusword and mode display */
	const int remote = !(g_busy || (g_mode != CTRL_OFF && !g_can_owner));
	uint16_t sw = k_sw_state[st < sizeof k_sw_state / sizeof k_sw_state[0] ? st : 0];
	if (g_vin > g_params.vbus_min) sw |= SW_VE;
	if (s_warn) sw |= SW_WARN;
	if (remote) sw |= SW_REMOTE;
	if (st == FSA_OPERATION_ENABLED) {
		if (reached) sw |= SW_REACHED;
		if (lim) sw |= SW_LIMIT;
		if (b12) sw |= SW_OMS12;
		if (b13) sw |= SW_OMS13;
	}
	else if (reached)
		sw |= SW_REACHED; /* quick stop: the motor stands */
	const int8_t disp = (st == FSA_OPERATION_ENABLED || st == FSA_QUICK_STOP_ACTIVE) ? s_mode : OD_RAM.x6060_modesOfOperation;
	if (sw != OD_RAM.x6041_statusword || disp != OD_RAM.x6061_modesOfOperationDisplay) {
		OD_RAM.x6041_statusword = sw;
		OD_RAM.x6061_modesOfOperationDisplay = disp;
		OD_requestTPDO(OD_ENTRY_H6041, 0); /* event-driven TPDO1 */
	}
}

/* ------------------------------------------------------------------ OD extensions */
/* 0x6060: only the modes this drive has */
static ODR_t w6060(OD_stream_t* s, const void* buf, OD_size_t count, OD_size_t* n) {
	if (count != 1) return ODR_TYPE_MISMATCH;
	const int8_t m = *(const int8_t*)buf;
	if (m != 0 && !supported(m)) return ODR_INVALID_VALUE; /* refused: the mode stays */
	return OD_writeOriginal(s, buf, count, n);
}

/* 0x6098: the methods without switches or index pulse: 37 (and the older 35), home here */
static ODR_t w6098(OD_stream_t* s, const void* buf, OD_size_t count, OD_size_t* n) {
	if (count != 1) return ODR_TYPE_MISMATCH;
	const int8_t m = *(const int8_t*)buf;
	if (m != 0 && m != 35 && m != 37) return ODR_INVALID_VALUE;
	return OD_writeOriginal(s, buf, count, n);
}

/* 0x6073, 0x6075, 0x6076, 0x6080: views of the motor parameters in CiA 402 units */
static float kt(void) { /* Nm/A, line-to-line peak convention as cal_kv(): Kt = 1.5 pp lambda */
	if (!(g_params.kv > 0.0f)) return 0.0f;
	const float lambda = 60.0f / (2.0f * (float)M_PI * SQRT3 * (float)g_params.pole_pairs * g_params.kv);
	return 1.5f * (float)g_params.pole_pairs * lambda;
}

static ODR_t r_param(OD_stream_t* s, void* buf, OD_size_t count, OD_size_t* n) {
	uint32_t v;
	OD_size_t len = 4;
	switch (s->index) {
		case 0x6073: {
			const float r = g_params.rated_current;
			const uint16_t pm = r > 0.0f ? (uint16_t)(g_params.max_current / r * 1000.0f + 0.5f) : 0;
			if (count < 2) return ODR_DEV_INCOMPAT;
			memcpy(buf, &pm, 2);
			*n = 2;
			return ODR_OK;
		}
		case 0x6075: v = (uint32_t)(g_params.rated_current * 1000.0f + 0.5f); break;
		case 0x6076: v = (uint32_t)(kt() * g_params.rated_current * 1000.0f + 0.5f); break;
		case 0x6080: v = (uint32_t)(g_params.vel_lim * INC_PER_TURN + 0.5f); break;
		default: return OD_readOriginal(s, buf, count, n);
	}
	if (count < len) return ODR_DEV_INCOMPAT;
	memcpy(buf, &v, 4);
	*n = 4;
	return ODR_OK;
}

static ODR_t w_param(OD_stream_t* s, const void* buf, OD_size_t count, OD_size_t* n) {
	float f;
	uint16_t idx;
	uint8_t sub;
	if (s->index == 0x6073) {
		if (count != 2) return ODR_TYPE_MISMATCH;
		uint16_t pm;
		memcpy(&pm, buf, 2);
		f = (float)pm * 0.001f * g_params.rated_current;
		idx = 0x4010, sub = 10;
	}
	else {
		if (count != 4) return ODR_TYPE_MISMATCH;
		uint32_t v;
		memcpy(&v, buf, 4);
		if (s->index == 0x6075)
			f = (float)v * 0.001f, idx = 0x4010, sub = 9;
		else if (s->index == 0x6080)
			f = (float)v * (1.0f / INC_PER_TURN), idx = 0x4021, sub = 4;
		else
			return ODR_READONLY;
	}
	const uint32_t ab = od_write(idx, sub, (const uint8_t*)&f, 4);
	if (ab) return (ODR_t)co_app_abort_to_odr(ab);
	*n = count;
	return ODR_OK;
}

static OD_extension_t s_ext_6060 = {.read = OD_readOriginal, .write = w6060};
static OD_extension_t s_ext_6098 = {.read = OD_readOriginal, .write = w6098};
static OD_extension_t s_ext_param[4] = {
	{.read = r_param, .write = w_param}, {.read = r_param, .write = w_param},
	{.read = r_param, .write = w_param}, {.read = r_param, .write = w_param},
};
static OD_extension_t s_ext_sw = {.read = OD_readOriginal, .write = OD_writeOriginal}; /* TPDO request flags */

/* 0x607A: flag every write, an unchanged target too (CSP: the master is still there, hold) */
static ODR_t w607A(OD_stream_t* st, const void* buf, OD_size_t count, OD_size_t* n) {
	const ODR_t r = OD_writeOriginal(st, buf, count, n);
	if (r == ODR_OK) s_tgt_rx = 1;
	return r;
}
static OD_extension_t s_ext_607A = {.read = OD_readOriginal, .write = w607A};

void co402_init(void) {
	OD_extension_init(OD_ENTRY_H6060, &s_ext_6060);
	OD_extension_init(OD_ENTRY_H6098, &s_ext_6098);
	OD_extension_init(OD_ENTRY_H6073, &s_ext_param[0]);
	OD_extension_init(OD_ENTRY_H6075, &s_ext_param[1]);
	OD_extension_init(OD_ENTRY_H6076, &s_ext_param[2]);
	OD_extension_init(OD_ENTRY_H6080, &s_ext_param[3]);
	OD_extension_init(OD_ENTRY_H6041, &s_ext_sw);
	OD_extension_init(OD_ENTRY_H607A, &s_ext_607A);
	s_cnt_prev = g_pos_cnt; /* the unwrapped position starts where the encoder is (within +-32768 turns of home) */
	s_act64 = s_cnt_prev;
	g_co402_state = FSA_SWITCH_ON_DISABLED;
}
