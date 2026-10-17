/* CANopenNode target definitions for this drive (STM32G431, FDCAN1, classic CAN).
 * Based on CanOpenSTM32 (Apache-2.0, https://github.com/CANopenNode/CanOpenSTM32), reduced to the G4 FDCAN.
 * Included from 301/CO_driver.h */
#ifndef CO_DRIVER_TARGET_H
#define CO_DRIVER_TARGET_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ stack configuration (see 301/CO_config.h) */
#define CO_USE_GLOBALS /* static objects, no malloc (the heap is 0) */

#define CO_CONFIG_NMT 0
#define CO_CONFIG_HB_CONS (CO_CONFIG_HB_CONS_ENABLE | CO_CONFIG_FLAG_OD_DYNAMIC)
#define CO_CONFIG_NODE_GUARDING 0
#define CO_CONFIG_EM (CO_CONFIG_EM_PRODUCER | CO_CONFIG_EM_PROD_CONFIGURABLE | CO_CONFIG_EM_PROD_INHIBIT | CO_CONFIG_EM_HISTORY)
#define CO_CONFIG_SDO_SRV (CO_CONFIG_SDO_SRV_SEGMENTED | CO_CONFIG_FLAG_OD_DYNAMIC)
#define CO_CONFIG_SDO_SRV_BUFFER_SIZE 32
#define CO_CONFIG_SDO_CLI 0
#define CO_CONFIG_TIME 0
#define CO_CONFIG_SYNC (CO_CONFIG_SYNC_ENABLE | CO_CONFIG_FLAG_OD_DYNAMIC)
#define CO_CONFIG_PDO                                                                                                     \
	(CO_CONFIG_RPDO_ENABLE | CO_CONFIG_TPDO_ENABLE | CO_CONFIG_RPDO_TIMERS_ENABLE | CO_CONFIG_TPDO_TIMERS_ENABLE |      \
	 CO_CONFIG_PDO_SYNC_ENABLE | CO_CONFIG_PDO_OD_IO_ACCESS | CO_CONFIG_FLAG_OD_DYNAMIC)
#define CO_CONFIG_STORAGE CO_CONFIG_STORAGE_ENABLE
#define CO_CONFIG_LEDS 0
#define CO_CONFIG_GFC 0
#define CO_CONFIG_SRDO 0
#define CO_CONFIG_LSS 0
#define CO_CONFIG_GTW 0
#define CO_CONFIG_CRC16 0
#define CO_CONFIG_FIFO 0
#define CO_CONFIG_TRACE 0

/* ------------------------------------------------------------------ basic types */
#define CO_LITTLE_ENDIAN
#define CO_SWAP_16(x) x
#define CO_SWAP_32(x) x
#define CO_SWAP_64(x) x

typedef uint_fast8_t bool_t;
typedef float float32_t;
typedef double float64_t;

/* received CAN frame */
typedef struct {
	uint32_t ident; /* 11-bit id, FLAG_RTR for remote frames */
	uint8_t dlc;
	uint8_t data[8];
} CO_CANrxMsg_t;

#define CO_CANrxMsg_readIdent(msg) ((uint16_t)(((CO_CANrxMsg_t*)(msg)))->ident)
#define CO_CANrxMsg_readDLC(msg) ((uint8_t)(((CO_CANrxMsg_t*)(msg)))->dlc)
#define CO_CANrxMsg_readData(msg) ((uint8_t*)(((CO_CANrxMsg_t*)(msg)))->data)

typedef struct {
	uint16_t ident;
	uint16_t mask;
	void* object;
	void (*CANrx_callback)(void* object, void* message);
} CO_CANrx_t;

typedef struct {
	uint32_t ident;
	uint8_t DLC;
	uint8_t data[8];
	volatile bool_t bufferFull;
	volatile bool_t syncFlag;
} CO_CANtx_t;

typedef struct {
	void* CANptr; /* FDCAN_HandleTypeDef* */
	CO_CANrx_t* rxArray;
	uint16_t rxSize;
	CO_CANtx_t* txArray;
	uint16_t txSize;
	uint16_t CANerrorStatus;
	volatile bool_t CANnormal;
	volatile bool_t useCANrxFilters;
	volatile bool_t bufferInhibitFlag;
	volatile bool_t firstCANtxMessage;
	volatile uint16_t CANtxCount;
	uint32_t errOld;
} CO_CANmodule_t;

typedef struct {
	void* addr;
	size_t len;
	uint8_t subIndexOD;
	uint8_t attr;
	void* addrNV;
} CO_storage_entry_t;

/* ------------------------------------------------------------------ locking
 * Everything CANopen runs at NVIC priority CO_IRQ_PRIO (FDCAN1_IT0 and the TIM7 tick, which cannot preempt each
 * other) or in the main loop. The locks mask only priorities >= CO_IRQ_PRIO through BASEPRI, so the control loop
 * (priority 0) and USB (1) are never held off */
#define CO_IRQ_PRIO 2u
#define CO_LOCK_GENERIC(v)                                    \
	do {                                                      \
		uint32_t v = __get_BASEPRI();                         \
		__set_BASEPRI_MAX(CO_IRQ_PRIO << (8u - __NVIC_PRIO_BITS)); \
		__DSB();                                              \
		__ISB();
#define CO_UNLOCK_GENERIC(v) \
	__set_BASEPRI(v);        \
	__DSB();                 \
	__ISB();                 \
	}                        \
	while (0)

#define CO_LOCK_CAN_SEND(CAN_MODULE) CO_LOCK_GENERIC(basepri_send)
#define CO_UNLOCK_CAN_SEND(CAN_MODULE) CO_UNLOCK_GENERIC(basepri_send)
#define CO_LOCK_EMCY(CAN_MODULE) CO_LOCK_GENERIC(basepri_emcy)
#define CO_UNLOCK_EMCY(CAN_MODULE) CO_UNLOCK_GENERIC(basepri_emcy)
#define CO_LOCK_OD(CAN_MODULE) CO_LOCK_GENERIC(basepri_od)
#define CO_UNLOCK_OD(CAN_MODULE) CO_UNLOCK_GENERIC(basepri_od)

/* receive (FDCAN IRQ) -> processing (tick): same priority, so a plain flag is enough */
#define CO_MemoryBarrier() __DMB()
#define CO_FLAG_READ(rxNew) ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew)       \
	do {                         \
		CO_MemoryBarrier();      \
		rxNew = (void*)1L;       \
	} while (0)
#define CO_FLAG_CLEAR(rxNew)     \
	do {                         \
		CO_MemoryBarrier();      \
		rxNew = NULL;            \
	} while (0)

#ifdef __cplusplus
}
#endif

#endif /* CO_DRIVER_TARGET_H */
