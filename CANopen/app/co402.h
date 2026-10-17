#ifndef CO402_H
#define CO402_H

/* CiA 402 drive profile on top of the firmware's run modes (docs/canopen-handoff.md).
 * co402_poll(), main loop: the state machine (controlword 0x6040), mode changes, faults, master loss. It is the
 *   only CANopen code that changes g_mode / g_loop, through the drive_*() functions of calib.c.
 * co402_tick(), CANopen tick (1 kHz, after the RPDOs): actual values, statusword, and the references while
 *   streaming (CSV, CST, the stop and halt ramps). It only writes references, and only while main has the motor
 *   armed for CANopen and nothing else (a trip, a coast, a local stop) took it */

#include <stdint.h>

/* CiA 402 FSA states, also object 0x4042:1 */
enum {
	FSA_NOT_READY = 0,
	FSA_SWITCH_ON_DISABLED,
	FSA_READY_TO_SWITCH_ON,
	FSA_SWITCHED_ON,
	FSA_OPERATION_ENABLED,
	FSA_QUICK_STOP_ACTIVE,
	FSA_FAULT_REACTION_ACTIVE,
	FSA_FAULT,
};

/* modes of operation (0x6060) */
enum { MODE_PP = 1, MODE_PV = 3, MODE_HM = 6, MODE_CSP = 8, MODE_CSV = 9, MODE_CST = 10 };

extern volatile uint8_t g_co402_state;	/* FSA_* */
extern volatile uint8_t g_co402_refused; /* st_t why the last switch on / enable was refused, 0 = none */
extern volatile uint8_t g_co402_fault;	/* st_t of the fault in 0x603F, 0 = none */

void co402_init(void); /* once, before the first communication reset: OD extensions */
void co402_poll(void); /* main loop */
void co402_tick(void); /* CANopen tick, after CO_process_RPDO() */

#endif
