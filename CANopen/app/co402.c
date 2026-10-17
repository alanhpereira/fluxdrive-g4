#include "co402.h"

#include <math.h>
#include <string.h>

#include "CANopen.h"
#include "OD.h"
#include "calib.h"
#include "co_app.h"
#include "objdict.h" /* od_write(): the parameter views go through the same checks as the USB protocol */
#include "pi_ctrl.h" /* g_vel_ref, g_iq_ref */

#define INC_PER_TURN 2097152.0f /* 0x608F: 2^21 increments per motor revolution */
#define TICK_S 0.001f
#define STOP_WAIT_MS 1000 /* after a stop ramp reached zero: coast anyway if the motor is not slow by then */

/* statusword bits (CiA 402) */
#define SW_VE 0x0010u	  /* voltage enabled */
#define SW_WARN 0x0080u	  /* warning */
#define SW_REMOTE 0x0200u /* controlword is processed */
#define SW_REACHED 0x0400u
#define SW_LIMIT 0x0800u  /* internal limit active */
#define SW_OMS12 0x1000u  /* cyclic modes: drive follows the command value */

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
/* main only */
enum { AFTER_SO = 1, AFTER_RTSO, AFTER_SOD, AFTER_HOLD };
static uint8_t s_after; /* where a stop ramp ends: switched on, ready, switch on disabled, or hold in quick stop */
static uint32_t s_zero_ms;
static uint16_t s_cw_prev;
static uint8_t s_st_prev;
static uint8_t s_nmt_op;  /* NMT was operational while the master had the motor */
static uint8_t s_hb_prev; /* heartbeat consumer timeout seen */
static uint8_t s_fault_bit;

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
	{ST_ABORTED, 0x47, 0xFF01},	 /* manufacturer: stopped from the USB/serial link */
	{ST_CAN_LOST, 0x48, 0x8130}, /* heartbeat / life guard error: master lost */
};
#define FAULT_OTHER_BIT 0x49
#define FAULT_OTHER_CODE 0x1000 /* generic error */

static CO_EM_t* em(void) {
	CO_t* co = co_app_co();
	return co ? co->em : NULL;
}

static loop_t loop_of(int8_t mode) {
	return mode == MODE_CST ? LOOP_CURRENT : LOOP_VEL;
}

static int supported(int8_t mode) {
	return mode == MODE_CSV || mode == MODE_CST;
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
	const int fresh = cw != s_cw_prev || st != s_st_prev; /* switch on / enable: tried once per command or state */
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
		if (why)
			fault(why);
		else if (!g_can_owner)
			fault(ST_ABORTED); /* a local STOP took the motor */
		else if (g_mode != CTRL_RUN)
			fault(ST_LOOP_DEAD);
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
				if (s_ramp) s_ramp = 0; /* the master changed its mind during a disable / shutdown ramp */
				if (want != s_mode && supported(want)) {
					s_stream = 0; /* switch the loop bumplessly, then stream the new mode */
					if (drive_enter(loop_of(want)) == ST_OK) {
						s_rv = g_vel_ref;
						s_mode = want;
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
			else if (eo && s_after == AFTER_HOLD) { /* transition 16: options 5..8 may go back to operation */
				s_ramp = 0;
				if (s_mode == MODE_CST) drive_enter(LOOP_CURRENT);
				s_rv = g_vel_ref;
				st = FSA_OPERATION_ENABLED;
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
	OD_RAM.x6064_positionActualValue = g_pos_cnt;
	OD_RAM.x606C_velocityActualValue = (int32_t)(g_vel * INC_PER_TURN);
	OD_RAM.x6077_torqueActualValue = OD_RAM.x6078_currentActualValue = sat16(g_iq_f * pm);
	OD_RAM.x6074_torqueDemand = sat16(g_iq_ref * pm);
	OD_RAM.x6079_DCLinkCircuitVoltage = g_vin > 0.0f ? (uint32_t)(g_vin * 1000.0f) : 0;

	/* references: only while main has the motor armed for CANopen and nothing else took it */
	uint8_t lim = 0, follows = 0, reached = 0;
	const int run = s_stream && g_can_owner && g_mode == CTRL_RUN && !g_coasting && !g_trip;
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
	else if (run && s_mode == MODE_CSV && g_loop == LOOP_VEL) {
		const float v = (float)(OD_RAM.x60FF_targetVelocity + OD_RAM.x60B1_velocityOffset) * (1.0f / INC_PER_TURN);
		g_vel_ref = s_rv = clampf(v, g_params.vel_lim, &lim);
		follows = 1;
	}
	else if (run && s_mode == MODE_CST && g_loop == LOOP_CURRENT) {
		float iq = halt ? 0.0f : (float)(OD_RAM.x6071_targetTorque + OD_RAM.x60B2_torqueOffset) * rated * 0.001f;
		float imax = (float)OD_PERSIST_APP.x6072_maxTorque * rated * 0.001f;
		if (imax > g_params.max_current) imax = g_params.max_current;
		g_iq_ref = clampf(iq, imax, &lim);
		follows = !halt;
		reached = halt;
	}
	s_limited = lim;
	OD_RAM.x606B_velocityDemandValue = g_loop == LOOP_VEL && g_mode == CTRL_RUN ? (int32_t)(g_vel_ref * INC_PER_TURN) : 0;

	/* statusword and mode display */
	const int remote = !(g_busy || (g_mode != CTRL_OFF && !g_can_owner));
	uint16_t sw = k_sw_state[st < sizeof k_sw_state / sizeof k_sw_state[0] ? st : 0];
	if (g_vin > g_params.vbus_min) sw |= SW_VE;
	if (s_warn) sw |= SW_WARN;
	if (remote) sw |= SW_REMOTE;
	if (reached) sw |= SW_REACHED;
	if (lim) sw |= SW_LIMIT;
	if (follows && st == FSA_OPERATION_ENABLED) sw |= SW_OMS12;
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
	if (m != 0 && !supported(m)) return ODR_INVALID_VALUE;
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
			memcpy(buf, &pm, 2);
			*n = 2;
			return count >= 2 ? ODR_OK : ODR_DEV_INCOMPAT;
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
static OD_extension_t s_ext_param[4] = {
	{.read = r_param, .write = w_param}, {.read = r_param, .write = w_param},
	{.read = r_param, .write = w_param}, {.read = r_param, .write = w_param},
};
static OD_extension_t s_ext_sw = {.read = OD_readOriginal, .write = OD_writeOriginal}; /* TPDO request flags */

void co402_init(void) {
	OD_extension_init(OD_ENTRY_H6060, &s_ext_6060);
	OD_extension_init(OD_ENTRY_H6073, &s_ext_param[0]);
	OD_extension_init(OD_ENTRY_H6075, &s_ext_param[1]);
	OD_extension_init(OD_ENTRY_H6076, &s_ext_param[2]);
	OD_extension_init(OD_ENTRY_H6080, &s_ext_param[3]);
	OD_extension_init(OD_ENTRY_H6041, &s_ext_sw);
	g_co402_state = FSA_SWITCH_ON_DISABLED;
}
