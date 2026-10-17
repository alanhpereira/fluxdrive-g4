/* CANopenNode driver for the STM32G4 FDCAN in classic CAN mode.
 * Based on CanOpenSTM32's CO_driver_STM32.c (Apache-2.0, https://github.com/CANopenNode/CanOpenSTM32): FDCAN only,
 * runtime bit rate, software RX filtering (all standard ids go to RX FIFO 0), IRQ at CO_IRQ_PRIO.
 * Owns FDCAN1_IT0_IRQHandler: keep the FDCAN1 interrupts disabled in the .ioc */
#include <string.h>

#include "301/CO_driver.h"
#include "co_app.h"
#include "fdcan.h"

#define CANID_MASK 0x07FFu
#define FLAG_RTR 0x8000u
#define TX_BUFFERS (FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2)

static CO_CANmodule_t* volatile s_mod; /* one instance */

volatile uint32_t g_can_rx, g_can_tx, g_can_rx_lost;

/* 170 MHz FDCAN clock, 17 tq per bit (sync + 13 + 3), sample point 82 %. Returns 0 for an unsupported rate */
uint16_t co_can_prescaler(uint16_t kbit) {
	switch (kbit) {
		case 1000: return 10;
		case 500: return 20;
		case 250: return 40;
		case 125: return 80;
		case 50: return 200;
		case 20: return 500;
		default: return 0;
	}
}

static const uint32_t k_dlc[9] = {FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4,
								  FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8};

/* call with the CAN lock held */
static int send_frame(CO_CANmodule_t* m, CO_CANtx_t* b) {
	FDCAN_HandleTypeDef* h = m->CANptr;
	if (HAL_FDCAN_GetTxFifoFreeLevel(h) == 0) return 0;
	FDCAN_TxHeaderTypeDef hdr = {
		.Identifier = b->ident & CANID_MASK,
		.IdType = FDCAN_STANDARD_ID,
		.TxFrameType = (b->ident & FLAG_RTR) ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME,
		.DataLength = k_dlc[b->DLC <= 8 ? b->DLC : 8],
		.ErrorStateIndicator = FDCAN_ESI_ACTIVE,
		.BitRateSwitch = FDCAN_BRS_OFF,
		.FDFormat = FDCAN_CLASSIC_CAN,
		.TxEventFifoControl = FDCAN_NO_TX_EVENTS,
		.MessageMarker = 0,
	};
	if (HAL_FDCAN_AddMessageToTxFifoQ(h, &hdr, b->data) != HAL_OK) return 0;
	g_can_tx++;
	return 1;
}

void CO_CANsetConfigurationMode(void* CANptr) {
	if (CANptr) HAL_FDCAN_Stop(CANptr);
}

void CO_CANsetNormalMode(CO_CANmodule_t* m) {
	if (m && m->CANptr && HAL_FDCAN_Start(m->CANptr) == HAL_OK) m->CANnormal = true;
}

CO_ReturnError_t CO_CANmodule_init(CO_CANmodule_t* m, void* CANptr, CO_CANrx_t rxArray[], uint16_t rxSize, CO_CANtx_t txArray[],
								   uint16_t txSize, uint16_t CANbitRate) {
	FDCAN_HandleTypeDef* h = CANptr;
	const uint16_t psc = co_can_prescaler(CANbitRate);
	if (!m || !h || !rxArray || !txArray || !psc) return CO_ERROR_ILLEGAL_ARGUMENT;

	m->CANptr = h;
	m->rxArray = rxArray;
	m->rxSize = rxSize;
	m->txArray = txArray;
	m->txSize = txSize;
	m->CANerrorStatus = 0;
	m->CANnormal = false;
	m->useCANrxFilters = false;
	m->bufferInhibitFlag = false;
	m->firstCANtxMessage = true;
	m->CANtxCount = 0;
	m->errOld = 0;
	for (uint16_t i = 0; i < rxSize; i++) {
		rxArray[i].ident = 0;
		rxArray[i].mask = 0xFFFF;
		rxArray[i].object = NULL;
		rxArray[i].CANrx_callback = NULL;
	}
	for (uint16_t i = 0; i < txSize; i++) txArray[i].bufferFull = false;
	s_mod = m;

	/* the .ioc holds 1 Mbit/s; the rate actually used is set here */
	h->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	h->Init.Mode = FDCAN_MODE_NORMAL;
	h->Init.AutoRetransmission = ENABLE;
	h->Init.TransmitPause = DISABLE;
	h->Init.ProtocolException = DISABLE;
	h->Init.NominalPrescaler = psc;
	h->Init.NominalSyncJumpWidth = 3;
	h->Init.NominalTimeSeg1 = 13;
	h->Init.NominalTimeSeg2 = 3;
	h->Init.StdFiltersNbr = 0;
	h->Init.ExtFiltersNbr = 0;
	h->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	if (HAL_FDCAN_Init(h) != HAL_OK) return CO_ERROR_ILLEGAL_ARGUMENT;

	/* no hardware filters: every standard data frame goes to FIFO 0 and is matched in software */
	if (HAL_FDCAN_ConfigGlobalFilter(h, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
		return CO_ERROR_ILLEGAL_ARGUMENT;
	if (HAL_FDCAN_ActivateNotification(h, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_TX_COMPLETE | FDCAN_IT_BUS_OFF,
									   TX_BUFFERS) != HAL_OK)
		return CO_ERROR_ILLEGAL_ARGUMENT;
	HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, CO_IRQ_PRIO, 0);
	HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
	return CO_ERROR_NO;
}

void CO_CANmodule_disable(CO_CANmodule_t* m) {
	if (m && m->CANptr) HAL_FDCAN_Stop(m->CANptr);
}

CO_ReturnError_t CO_CANrxBufferInit(CO_CANmodule_t* m, uint16_t index, uint16_t ident, uint16_t mask, bool_t rtr, void* object,
									void (*CANrx_callback)(void* object, void* message)) {
	if (!m || !object || !CANrx_callback || index >= m->rxSize) return CO_ERROR_ILLEGAL_ARGUMENT;
	CO_CANrx_t* b = &m->rxArray[index];
	b->object = object;
	b->CANrx_callback = CANrx_callback;
	b->ident = (ident & CANID_MASK) | (rtr ? FLAG_RTR : 0u);
	b->mask = (mask & CANID_MASK) | FLAG_RTR;
	return CO_ERROR_NO;
}

CO_CANtx_t* CO_CANtxBufferInit(CO_CANmodule_t* m, uint16_t index, uint16_t ident, bool_t rtr, uint8_t noOfBytes, bool_t syncFlag) {
	if (!m || index >= m->txSize) return NULL;
	CO_CANtx_t* b = &m->txArray[index];
	b->ident = (ident & CANID_MASK) | (rtr ? FLAG_RTR : 0u);
	b->DLC = noOfBytes;
	b->bufferFull = false;
	b->syncFlag = syncFlag;
	return b;
}

CO_ReturnError_t CO_CANsend(CO_CANmodule_t* m, CO_CANtx_t* b) {
	CO_ReturnError_t err = CO_ERROR_NO;
	if (b->bufferFull) {
		if (!m->firstCANtxMessage) m->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW; /* not while the boot-up is still queued */
		err = CO_ERROR_TX_OVERFLOW;
	}
	CO_LOCK_CAN_SEND(m);
	if (send_frame(m, b))
		m->bufferInhibitFlag = b->syncFlag;
	else if (!b->bufferFull) { /* hardware queue full: the TX-complete interrupt sends it */
		b->bufferFull = true;
		m->CANtxCount++;
	}
	CO_UNLOCK_CAN_SEND(m);
	return err;
}

void CO_CANclearPendingSyncPDOs(CO_CANmodule_t* m) {
	uint32_t deleted = 0;
	CO_LOCK_CAN_SEND(m);
	if (m->bufferInhibitFlag) {
		m->bufferInhibitFlag = false;
		deleted = 1;
	}
	if (m->CANtxCount > 0) {
		CO_CANtx_t* b = m->txArray;
		for (uint16_t i = m->txSize; i > 0; i--, b++) {
			if (b->bufferFull && b->syncFlag) {
				b->bufferFull = false;
				m->CANtxCount--;
				deleted = 2;
			}
		}
	}
	CO_UNLOCK_CAN_SEND(m);
	if (deleted) m->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
}

/* error state from the protocol status register; called from CO_process() */
void CO_CANmodule_process(CO_CANmodule_t* m) {
	FDCAN_HandleTypeDef* h = m->CANptr;
	const uint32_t err = h->Instance->PSR & (FDCAN_PSR_BO | FDCAN_PSR_EW | FDCAN_PSR_EP);
	/* bus-off sets INIT; restart every tick until it sticks (the hardware then waits 129 x 11 recessive bits),
	 * not only when the state changes, or a second bus-off in a row leaves the node off the bus for good */
	if ((err & FDCAN_PSR_BO) && m->CANnormal) CLEAR_BIT(h->Instance->CCCR, FDCAN_CCCR_INIT);
	if (err == m->errOld) return;
	m->errOld = err;
	uint16_t st = m->CANerrorStatus;
	if (err & FDCAN_PSR_BO)
		st |= CO_CAN_ERRTX_BUS_OFF;
	else {
		st &= (uint16_t) ~(CO_CAN_ERRTX_BUS_OFF | CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE);
		if (err & FDCAN_PSR_EW) st |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRTX_WARNING;
		if (err & FDCAN_PSR_EP) st |= CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_PASSIVE;
		if (!(st & CO_CAN_ERRTX_PASSIVE)) st &= (uint16_t)~CO_CAN_ERRTX_OVERFLOW; /* not latched: see CanOpenSTM32 */
	}
	m->CANerrorStatus = st;
}

/* ------------------------------------------------------------------ interrupt side */
void FDCAN1_IT0_IRQHandler(void) {
	HAL_FDCAN_IRQHandler(&hfdcan1);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* h, uint32_t its) {
	CO_CANmodule_t* m = s_mod;
	if (its & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) {
		g_can_rx_lost++;
		if (m) m->CANerrorStatus |= CO_CAN_ERRRX_OVERFLOW;
	}
	if (!m || !(its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)) return;
	FDCAN_RxHeaderTypeDef hdr;
	uint8_t data[64]; /* an FD frame from a misconfigured node must not overrun 8 bytes */
	while (HAL_FDCAN_GetRxFifoFillLevel(h, FDCAN_RX_FIFO0) > 0) {
		if (HAL_FDCAN_GetRxMessage(h, FDCAN_RX_FIFO0, &hdr, data) != HAL_OK) return;
		g_can_rx++;
		CO_CANrxMsg_t msg;
		msg.ident = hdr.Identifier | (hdr.RxFrameType == FDCAN_REMOTE_FRAME ? FLAG_RTR : 0u);
		msg.dlc = 0;
		for (uint8_t n = 0; n <= 8; n++)
			if (k_dlc[n] == hdr.DataLength) msg.dlc = n; /* anything longer than 8 reads as 0 */
		memcpy(msg.data, data, msg.dlc);
		CO_CANrx_t* b = m->rxArray;
		for (uint16_t i = m->rxSize; i > 0; i--, b++) {
			if (((msg.ident ^ b->ident) & b->mask) == 0) {
				if (b->CANrx_callback) b->CANrx_callback(b->object, &msg);
				break;
			}
		}
	}
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef* h, uint32_t indexes) {
	(void)h;
	(void)indexes;
	CO_CANmodule_t* m = s_mod;
	if (!m) return;
	m->firstCANtxMessage = false; /* the boot-up went out */
	m->bufferInhibitFlag = false;
	CO_LOCK_CAN_SEND(m);
	if (m->CANtxCount > 0) {
		CO_CANtx_t* b = m->txArray;
		for (uint16_t i = m->txSize; i > 0; i--, b++) {
			if (!b->bufferFull) continue;
			if (!send_frame(m, b)) break; /* hardware queue full again */
			b->bufferFull = false;
			m->CANtxCount--;
			m->bufferInhibitFlag = b->syncFlag;
		}
	}
	CO_UNLOCK_CAN_SEND(m);
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* h, uint32_t its) {
	if (its & FDCAN_IT_BUS_OFF) CLEAR_BIT(h->Instance->CCCR, FDCAN_CCCR_INIT); /* recover: 128 x 11 recessive bits, then on */
}
