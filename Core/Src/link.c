#include "link.h"

#include <string.h>

#include "main.h"
#include "proto.h"

/* link_tx() critical sections: mask the link IRQ (priority 1) and below, never the control loop at 0 */
#define LINK_BASEPRI (1u << (8u - __NVIC_PRIO_BITS))

static void (*volatile s_defer)(void);

void link_defer(void (*fn)(void)) {
	s_defer = fn;
#if PROTO_LINK_USB
	NVIC_SetPendingIRQ(USB_LP_IRQn);
#else
	NVIC_SetPendingIRQ(USART1_IRQn); /* HAL_UART_IRQHandler() then finds no flag and returns */
#endif
}

void link_irq(void) {
	void (*fn)(void) = s_defer;
	if (fn) {
		s_defer = NULL;
		fn();
	}
#if PROTO_LINK_USB
	link_usb_pump(); /* producers pend this IRQ after queuing TX bytes */
#endif
}

#if PROTO_LINK_USB
/* ------------------------------------------------------------------ USB CDC
 * Producers (main, and the deferred telemetry in this IRQ; both with priority >= 1 masked) append to the
 * ring and pend USB_LP_IRQn.
 * Only the USB IRQ (priority 1, below the control loop) calls into the USB stack: it sends the
 * contiguous part of the ring and frees it when the transfer completes. Free-running indices:
 * the head is written by producers only, the tail by the USB IRQ only */
#include "usbd_cdc.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

#define TXQ 2048u /* power of two; ~11 ms of telemetry */
static uint8_t txq[TXQ];
static volatile uint32_t txh, txt;
static uint32_t inflight; /* bytes of the current transfer, USB IRQ only */
static volatile uint8_t s_open;

void link_init(void) {} /* MX_USB_Device_Init() has done it all; RX starts in CDC_Init_FS */

int link_tx_ready(void) {
	return s_open;
}

int link_tx(const void* p, uint16_t n) {
	if (!s_open) return 1; /* nobody listening */
	const uint32_t bp = __get_BASEPRI();
	__set_BASEPRI_MAX(LINK_BASEPRI);
	const uint32_t h = txh;
	if (TXQ - (h - txt) < n) {
		__set_BASEPRI(bp);
		return 0;
	}
	const uint32_t i = h & (TXQ - 1);
	const uint32_t k = n < TXQ - i ? n : TXQ - i;
	memcpy(txq + i, p, k);
	memcpy(txq, (const uint8_t*)p + k, n - k);
	txh = h + n;
	__set_BASEPRI(bp);
	NVIC_SetPendingIRQ(USB_LP_IRQn);
	return 1;
}

void link_usb_pump(void) {
	USBD_CDC_HandleTypeDef* c = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
	if (c == NULL || c->TxState != 0U) return; /* busy: link_usb_tx_done() pumps again */
	if (!s_open || hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
		txt = txh; /* drop, so a host opening the port gets fresh data */
		inflight = 0;
		return;
	}
	const uint32_t t = txt, i = t & (TXQ - 1);
	uint32_t n = txh - t;
	if (n == 0) return;
	if (n > TXQ - i) n = TXQ - i;
	inflight = n;
	USBD_CDC_SetTxBuffer(&hUsbDeviceFS, txq + i, n);
	if (USBD_CDC_TransmitPacket(&hUsbDeviceFS) != USBD_OK) inflight = 0;
}

void link_usb_tx_done(void) {
	txt += inflight;
	inflight = 0;
	link_usb_pump();
}

void link_usb_open(int dtr) {
	s_open = (uint8_t)(dtr != 0); /* a transfer still in flight completes or is reset with the class */
}

void link_usb_reset(void) {
	s_open = 0;
	inflight = 0; /* the stack has forgotten any transfer; the next pump drops the queue */
}

#else
/* ------------------------------------------------------------------ USART1 (fallback)
 * RX: HAL, byte interrupts at priority 1 (below the control loop, like USB). TX: DMA1_Channel3 by register,
 * no interrupts (HAL_UART_Transmit_DMA() and its DMA and UART TC interrupts ran at priority 0). A transfer is
 * over when the channel has moved all its bytes (CNDTR = 0); the last ones are still in the UART FIFO then,
 * and the next transfer queues behind them */
#include "usart.h"

extern DMA_HandleTypeDef hdma_usart1_tx;
static uint8_t rxb;
static uint32_t txb[20]; /* telemetry (72) or a protocol frame (<= 71), alive until the DMA is done */

void link_init(void) {
	/* the 8-byte RX FIFO rides out currentLoop(), which preempts this IRQ */
	HAL_UARTEx_EnableFifoMode(&huart1);
	HAL_UART_Receive_IT(&huart1, &rxb, 1);
	DMA_Channel_TypeDef* c = hdma_usart1_tx.Instance; /* MX_USART1_UART_Init() set its DMAMUX request */
	c->CCR = 0;
	c->CPAR = (uint32_t)&USART1->TDR;
	c->CMAR = (uint32_t)txb;
	c->CNDTR = 0;
	c->CCR = DMA_CCR_DIR | DMA_CCR_MINC; /* bytes, low priority, no interrupts; enabled per transfer */
	SET_BIT(USART1->CR3, USART_CR3_DMAT);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* h) {
	if (h == &huart1) {
		proto_rx_bytes(&rxb, 1);
		HAL_UART_Receive_IT(&huart1, &rxb, 1);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* h) {
	if (h == &huart1) HAL_UART_Receive_IT(&huart1, &rxb, 1); /* overrun/noise must not kill RX */
}

int link_tx_ready(void) {
	const DMA_Channel_TypeDef* c = hdma_usart1_tx.Instance;
	return !(c->CCR & DMA_CCR_EN) || c->CNDTR == 0;
}

/* check and start with the link IRQ masked: main and the deferred telemetry both send, ready means txb is free */
int link_tx(const void* p, uint16_t n) {
	if (n > sizeof txb) return 1;
	const uint32_t bp = __get_BASEPRI();
	__set_BASEPRI_MAX(LINK_BASEPRI);
	const int ok = link_tx_ready();
	if (ok) {
		DMA_Channel_TypeDef* c = hdma_usart1_tx.Instance;
		/* by words: newlib's memcpy goes byte by byte from flash, ~6 cycles a byte */
		const uint8_t* src = p;
		uint16_t i = 0;
		for (; i + 4u <= n; i += 4) {
			uint32_t w;
			__builtin_memcpy(&w, src + i, 4); /* one LDR */
			txb[i / 4] = w;
		}
		for (; i < n; i++) ((uint8_t*)txb)[i] = src[i];
		c->CCR &= ~DMA_CCR_EN;
		c->CNDTR = n;
		c->CCR |= DMA_CCR_EN;
	}
	__set_BASEPRI(bp);
	return ok;
}
#endif
