#ifndef CO_APP_H
#define CO_APP_H

/* CANopen node (CANopenNode v4 on FDCAN1): docs/canopen-handoff.md.
 * CO_process(), SYNC, RPDO and TPDO run in the 1 kHz TIM7 tick at CO_IRQ_PRIO, so heartbeat and SDO keep going
 * while the main loop blocks (calibrations). Communication resets (NMT, new node ID or bit rate) are done by
 * co_app_poll() in the main loop, since HAL_FDCAN_Init() waits on HAL_GetTick() */

#include <stdint.h>

/* settings for the next communication reset (object 0x4040, comm record in flash) */
extern volatile uint8_t g_can_node_id;	 /* 1..127 */
extern volatile uint16_t g_can_kbit;	 /* 20, 50, 125, 250, 500, 1000 */
/* in use since the last communication reset */
extern volatile uint8_t g_can_node_id_act;
extern volatile uint16_t g_can_kbit_act;
/* driver counters (CO_driver_stm32g4.c) */
extern volatile uint32_t g_can_rx, g_can_tx, g_can_rx_lost;

uint16_t co_can_prescaler(uint16_t kbit); /* 0 = bit rate not supported */

void co_app_init(void); /* once, after MX_FDCAN1_Init(), before the main loop */
void co_app_poll(void); /* main loop: pending communication / application resets */
void co_app_request_reset(void); /* communication reset at the next co_app_poll() (0x4040:5) */

uint8_t co_app_nmt_state(void);	 /* CO_NMT_internalState_t: 0 init, 4 stopped, 5 operational, 127 pre-operational */
uint16_t co_app_can_state(void); /* CANerrorStatus bits (CO_CAN_ERR*) */

#endif
