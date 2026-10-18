#ifndef LINK_H
#define LINK_H

/* Byte pipe under the telemetry and config frames: USB CDC (default) or USART1 at 2 Mbaud.
 * Received bytes go to proto_rx_bytes(). The frame formats are the same on both */

#include <stdint.h>
#ifndef PROTO_LINK_USB
#define PROTO_LINK_USB 1
#endif

void link_init(void); /* starts RX; call once after MX_USART1_UART_Init() / MX_USB_Device_Init() */

/* Queues a whole frame or nothing. Returns 0 only when it must be retried (UART busy, USB queue full).
 * With no host on the USB port the bytes are discarded and it returns 1. Callable from main and from the
 * link IRQ (link_defer()), not from priority 0: it masks priority >= 1 (BASEPRI), never the control loop */
int link_tx(const void* p, uint16_t n);

/* 1 when link_tx() is worth calling: UART idle, or a USB host has the port open (DTR). Callable anywhere */
int link_tx_ready(void);

/* from the control ISR: run fn once from the link IRQ (priority 1: USART1, or USB_LP), where it may call
 * link_tx(). The telemetry frame is finished and queued there, outside the 25 kHz budget */
void link_defer(void (*fn)(void));
/* USART1_IRQn / USB_LP_IRQn, USER CODE section 0 */
void link_irq(void);

#if PROTO_LINK_USB
/* for usbd_cdc_if.c and USB_LP_IRQHandler: USB IRQ context only */
void link_usb_pump(void);
void link_usb_tx_done(void);
void link_usb_open(int dtr); /* SET_CONTROL_LINE_STATE */
void link_usb_reset(void);	 /* CDC class init/deinit (enumeration, bus reset) */
#endif

#endif
