#ifndef HALL_H
#define HALL_H

/* Hall sensors on J13 (HALL1..3 = PC2, PC1, PC0), the angle source of ang_src 3 and 4 (docs/design-notes.md,
 * Hall). Polled once per cycle from currentLoop(); the EXTI lines are not used */

#include <stdint.h>

#define HALL_LOST_N 250 /* cycles (10 ms) of invalid states (000, 111: sensor unplugged or unpowered) stop the motor */

extern volatile uint8_t g_hall_state;	   /* filtered state, HALL1 = bit 0, HALL2 = bit 1, HALL3 = bit 2 */
extern volatile uint16_t g_hall_bad_n;	   /* consecutive cycles with an invalid state, see HALL_LOST_N */
extern volatile uint32_t g_hall_bad_total; /* invalid states and non-adjacent jumps since boot, diagnostics */

void hall_init(void);		  /* pull-ups, EXTI0-2 off, table; once at start-up after params_load() */
void hall_table_apply(void);  /* rebuilds the sector table from g_params.hall_edge; main, motor off */
int hall_table_ok(void);	  /* PF_HALL_VALID and a usable table */

/* once per cycle in currentLoop(): electrical angle now (Q32) and mechanical speed (turn/s). Returns 1 with a
 * calibrated table and a valid state, 0 otherwise (the outputs are then 0, or held at the last valid ones) */
int hall_update(uint32_t* th, float* vel);

/* 1 if th (Q32 electrical) lies in the present sector, widened by margin (Q32) on both sides */
int hall_in_sector(uint32_t th, uint32_t margin);

#endif
