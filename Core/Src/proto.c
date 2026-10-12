#include "proto.h"

#include <string.h>

#include "objdict.h"
#include "params.h"
#include "usart.h"

volatile uint32_t g_proto_err;

/* ------------------------------------------------------------------ UART RX (single bytes, IRQ) */
/* 256 with uint8_t indices: the wrap is free. Whole frames arrive at 2 Mbaud (a byte every 5 us) */
#define RXQ 256
static volatile uint8_t rxq[RXQ];
static volatile uint8_t rxh, rxt;
static uint8_t rxb;

void proto_init(void) {
	objdict_init();
	/* the 8-byte RX FIFO rides out currentLoop(), which runs at the same IRQ priority as the UART */
	HAL_UARTEx_EnableFifoMode(&huart1);
	HAL_UART_Receive_IT(&huart1, &rxb, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* h) {
	if (h == &huart1) {
		uint8_t n = (uint8_t)(rxh + 1);
		if (n != rxt) {
			rxq[rxh] = rxb;
			rxh = n;
		}
		HAL_UART_Receive_IT(&huart1, &rxb, 1);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* h) {
	if (h == &huart1) HAL_UART_Receive_IT(&huart1, &rxb, 1); /* overrun/noise must not kill RX */
}

static int rx_pop(uint8_t* c) {
	if (rxt == rxh) return 0;
	*c = rxq[rxt];
	rxt = (uint8_t)(rxt + 1);
	return 1;
}

/* ------------------------------------------------------------------ framing */
#define SYNC0 0xA5
#define SYNC1 0x5C
#define FRAME_TIMEOUT_MS 50 /* a frame whose bytes stop coming is dropped */

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xor out */
static uint16_t crc16(const uint8_t* p, int n) {
	uint16_t c = 0xFFFF;
	while (n--) {
		c ^= (uint16_t)(*p++ << 8);
		for (int i = 0; i < 8; i++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
	}
	return c;
}

static uint8_t txb[2 + 3 + PROTO_MAX_LEN + 2];

void proto_send(uint8_t type, uint8_t tok, const void* payload, uint8_t len) {
	if (len > PROTO_MAX_LEN) return;
	const uint32_t t0 = HAL_GetTick();
	/* READY once means our previous frame has left txb; only the ISR's telemetry can start in between */
	while (huart1.gState != HAL_UART_STATE_READY)
		if (HAL_GetTick() - t0 > 20) return;
	txb[0] = SYNC0;
	txb[1] = SYNC1;
	txb[2] = type;
	txb[3] = tok;
	txb[4] = len;
	memcpy(txb + 5, payload, len);
	const uint16_t c = crc16(txb + 2, 3 + len);
	txb[5 + len] = (uint8_t)c;
	txb[6 + len] = (uint8_t)(c >> 8);
	/* check and start with IRQs off: the telemetry ISR does the same check, so we never collide */
	for (;;) {
		__disable_irq();
		if (huart1.gState == HAL_UART_STATE_READY) {
			HAL_UART_Transmit_DMA(&huart1, txb, (uint16_t)(7 + len));
			__enable_irq();
			return;
		}
		__enable_irq();
		if (HAL_GetTick() - t0 > 20) return;
	}
}

void proto_reply_st(uint8_t type, uint8_t tok, uint32_t st) {
	proto_send(type | MSG_REPLY, tok, &st, 4);
}

void proto_event(uint8_t tok, uint16_t code, int n, const float* v) {
	uint8_t b[PROTO_MAX_LEN];
	if (n > (PROTO_MAX_LEN - 3) / 4) n = (PROTO_MAX_LEN - 3) / 4;
	memcpy(b, &code, 2);
	b[2] = (uint8_t)n;
	memcpy(b + 3, v, 4 * n);
	proto_send(MSG_EVENT, tok, b, (uint8_t)(3 + 4 * n));
}

/* ------------------------------------------------------------------ parser */
enum { P_IDLE,
	   P_SYNC,
	   P_FRAME };
static uint8_t rx_state, rx_n;
static uint32_t rx_t;
static uint8_t rx_b[3 + PROTO_MAX_LEN + 2]; /* type, tok, len, payload, crc */

/* returns 1 when rx_b holds a complete frame with a good CRC. Bytes outside frames are dropped */
static int feed(uint8_t c) {
	const uint32_t now = HAL_GetTick();
	if (rx_state != P_IDLE && now - rx_t > FRAME_TIMEOUT_MS) {
		if (rx_state == P_FRAME) g_proto_err++;
		rx_state = P_IDLE;
	}
	rx_t = now;
	switch (rx_state) {
		case P_SYNC:
			if (c == SYNC1) {
				rx_state = P_FRAME;
				rx_n = 0;
				return 0;
			}
			rx_state = P_IDLE; /* not a frame: look at this byte again */
							   /* fall through */
		case P_IDLE:
			if (c == SYNC0) rx_state = P_SYNC;
			return 0;
		default:
			rx_b[rx_n++] = c;
			if (rx_n == 3 && c > PROTO_MAX_LEN) {
				g_proto_err++;
				rx_state = P_IDLE;
				return 0;
			}
			if (rx_n < 3 || rx_n < 3 + rx_b[2] + 2) return 0;
			rx_state = P_IDLE;
			if (crc16(rx_b, rx_n - 2) != (uint16_t)(rx_b[rx_n - 2] | rx_b[rx_n - 1] << 8)) {
				g_proto_err++;
				return 0;
			}
			return 1;
	}
}

static void handle(int busy, int* abort) {
	uint8_t f[3 + PROTO_MAX_LEN]; /* copy: a command may block and re-enter the parser, which reuses rx_b */
	memcpy(f, rx_b, 3 + rx_b[2]);
	const uint8_t type = f[0], tok = f[1], len = f[2];
	const uint8_t* p = f + 3;
	uint8_t r[PROTO_MAX_LEN];
	uint32_t st;
	uint16_t idx;
	if (type & MSG_REPLY) return; /* replies and events only go the other way */
	switch (type) {
		case MSG_PING: {
			const uint16_t pv = PROTO_VERSION, parv = PARAM_VERSION;
			const uint32_t h = objdict_hash(), b = objdict_build_id();
			memcpy(r, &pv, 2);
			memcpy(r + 2, &h, 4);
			memcpy(r + 6, &parv, 2);
			memcpy(r + 8, &b, 4);
			proto_send(type | MSG_REPLY, tok, r, 12);
			return;
		}
		case MSG_READ: {
			uint8_t n = 0;
			memcpy(&idx, p, 2);
			st = len == 3 ? od_read(idx, p[2], r + 4, &n) : SDO_BAD_LEN;
			if (st) n = 0;
			memcpy(r, &st, 4);
			proto_send(type | MSG_REPLY, tok, r, (uint8_t)(4 + n));
			return;
		}
		case MSG_WRITE:
			memcpy(&idx, p, 2);
			st = busy ? ST_BUSY : len < 4 ? SDO_BAD_LEN
										  : od_write(idx, p[2], p + 3, (uint8_t)(len - 3));
			proto_reply_st(type, tok, st);
			return;
		case MSG_LIST: {
			if (len != 2) {
				r[0] = 0;
				proto_send(type | MSG_REPLY, tok, r, 1);
				return;
			}
			memcpy(&idx, p, 2);
			r[0] = (uint8_t)od_list(idx, r + 1, (PROTO_MAX_LEN - 1) / 5);
			proto_send(type | MSG_REPLY, tok, r, (uint8_t)(1 + 5 * r[0]));
			return;
		}
		case MSG_ABORT:
			if (busy) *abort = 1;
			proto_reply_st(type, tok, ST_OK);
			return;
		case MSG_STOP:
			if (busy) { /* STOP always works: it ends a calibration too */
				*abort = 1;
				proto_reply_st(type, tok, ST_OK);
				return;
			}
			break;
		case MSG_SAVE:
		case MSG_CAL:
		case MSG_RUN_CUR:
		case MSG_RUN_VEL:
		case MSG_RUN_POS:
		case MSG_HOME:
			break;
		default:
			proto_reply_st(type, tok, ST_BAD_TYPE);
			return;
	}
	if (busy)
		proto_reply_st(type, tok, ST_BUSY);
	else
		cmd_message(type, tok, p, len);
}

int proto_poll(int busy) {
	int abort = 0;
	uint8_t c;
	while (rx_pop(&c))
		if (feed(c)) handle(busy, &abort);
	return abort;
}
