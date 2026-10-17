#include "link.h"

#include <string.h>

#include "main.h"
#include "proto.h"

#if PROTO_LINK_USB
/* ------------------------------------------------------------------ USB CDC
 * Producers (the ADC ISR at priority 0, main with IRQs off) append to the ring and pend USB_LP_IRQn.
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
	const uint32_t pm = __get_PRIMASK();
	__disable_irq();
	const uint32_t h = txh;
	if (TXQ - (h - txt) < n) {
		__set_PRIMASK(pm);
		return 0;
	}
	const uint32_t i = h & (TXQ - 1);
	const uint32_t k = n < TXQ - i ? n : TXQ - i;
	memcpy(txq + i, p, k);
	memcpy(txq, (const uint8_t*)p + k, n - k);
	txh = h + n;
	__set_PRIMASK(pm);
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
/* ------------------------------------------------------------------ USART1 (fallback) */
#include "usart.h"

static uint8_t rxb;
static uint8_t txb[80]; /* telemetry (72) or a protocol frame (<= 71), alive until the DMA is done */

void link_init(void) {
	/* the 8-byte RX FIFO rides out currentLoop(), which runs at the same IRQ priority as the UART */
	HAL_UARTEx_EnableFifoMode(&huart1);
	HAL_UART_Receive_IT(&huart1, &rxb, 1);
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
	return huart1.gState == HAL_UART_STATE_READY;
}

/* check and start with IRQs off: main and the telemetry ISR both send, READY means txb is free */
int link_tx(const void* p, uint16_t n) {
	if (n > sizeof txb) return 1;
	const uint32_t pm = __get_PRIMASK();
	__disable_irq();
	const int ok = huart1.gState == HAL_UART_STATE_READY;
	if (ok) {
		memcpy(txb, p, n);
		HAL_UART_Transmit_DMA(&huart1, txb, n);
	}
	__set_PRIMASK(pm);
	return ok;
}
#endif
