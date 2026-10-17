#ifndef CO_STORE_H
#define CO_STORE_H

/* CANopen communication record in flash pages 60-61 (A/B slots, seq + CRC32, like params.c):
 * the node ID and bit rate (0x4040) and a copy of OD_PERSIST_COMM (0x1000-0x1FFF: PDO mapping, heartbeat,
 * SYNC/EMCY settings...). The OD copy carries a signature of the OD layout, so after a CANopenEditor
 * re-export that changes the layout it is ignored and the exported defaults load instead. */

#include <stdint.h>

#include "stm32g4xx_hal.h"

void co_store_load(void);	 /* once at boot: node ID and bit rate from the newest valid record */
void co_store_apply_od(void); /* at each communication reset: OD_PERSIST_COMM from the record, else the defaults */
int co_store_dirty(void);	 /* 1 if node ID, bit rate or OD_PERSIST_COMM differ from the record (or there is none) */

/* writes the record (MOE must be off). restore_od = 1 keeps node ID / bit rate but drops the OD copy, so the
 * defaults load at the next communication reset (0x1011) */
HAL_StatusTypeDef co_store_save(int restore_od);

/* 0x1010 / 0x1011 over SDO run in the CANopen tick, which must not erase flash: they leave a request here and
 * the main loop carries it out (calib.c, with the bridge off) */
enum { CO_RQ_SAVE_COMM = 1, CO_RQ_SAVE_APP = 2, CO_RQ_RESTORE_COMM = 4 };
uint8_t co_store_take_request(void);
void co_store_request(uint8_t rq);

#endif
