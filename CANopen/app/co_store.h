#ifndef CO_STORE_H
#define CO_STORE_H

/* CANopen record in flash pages 60-61 (A/B slots, seq + CRC32, like params.c): the node ID and bit rate (0x4040),
 * a copy of OD_PERSIST_COMM (0x1000-0x1FFF: PDO mapping, heartbeat, SYNC/EMCY settings...) and of OD_PERSIST_APP
 * (the CiA 402 settings: profile ramps, windows, option codes...). The OD copies carry a signature of the OD
 * layout, so after a CANopenEditor re-export that changes the layout they are ignored and the exported defaults
 * load instead; the node ID and bit rate survive. */

#include <stdint.h>

#include "stm32g4xx_hal.h"

void co_store_load(void);	 /* once at boot: node ID, bit rate and OD_PERSIST_APP from the newest valid record */
void co_store_apply_od(void); /* at each communication reset: OD_PERSIST_COMM from the record, else the defaults */
int co_store_dirty(void);	 /* 1 if anything kept here differs from the record (or there is none) */

/* 0x1010 / 0x1011 over SDO run in the CANopen tick, which must not erase flash: they leave a request here and
 * the main loop carries it out (calib.c, with the bridge off) */
enum { CO_RQ_SAVE_COMM = 1, CO_RQ_SAVE_APP = 2, CO_RQ_RESTORE_COMM = 4, CO_RQ_RESTORE_APP = 8 };

/* writes the record with what is in RAM now (MOE must be off). CO_RQ_RESTORE_COMM / _APP in rq mark that part
 * as not saved, so its exported defaults load at the next communication reset / boot (0x1011) */
HAL_StatusTypeDef co_store_save(uint8_t rq);
uint8_t co_store_take_request(void);
void co_store_request(uint8_t rq);

#endif
