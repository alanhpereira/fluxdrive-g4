#ifndef PROTO_H
#define PROTO_H

/* Binary config protocol on USART1, next to the telemetry frames (docs/config-gui-handoff.md):
 *   A5 5C | type u8 | tok u8 | len u8 | payload[len] | crc u16
 *   crc = CRC-16/CCITT-FALSE over type..payload, little-endian; len <= PROTO_MAX_LEN
 * Bytes outside frames are ignored.
 * Keep in sync with software/protocol.py: message types, status codes, event codes */

#include <stdint.h>

#define PROTO_VERSION 4u
#define PROTO_MAX_LEN 64u

/* message types, host -> device; the reply is (type | MSG_REPLY) with the same tok */
#define MSG_PING 0x01	 /* -> proto ver u16, table hash u32, PARAM_VERSION u16, build id u32 */
#define MSG_READ 0x10	 /* index u16, sub u8 -> status u32, value */
#define MSG_WRITE 0x11	 /* index u16, sub u8, value -> status u32 */
#define MSG_LIST 0x12	 /* first u16 -> n u8, n x (index u16, sub u8, type u8, flags u8) */
#define MSG_SAVE 0x20	 /* -> status u32 */
#define MSG_CAL 0x30	 /* which u8, arg f32 -> status u32 (ACK), then events, EV_DONE at the end */
#define MSG_ABORT 0x31	 /* -> status u32 */
#define MSG_RUN_CUR 0x40 /* id f32, iq f32 -> status u32 */
#define MSG_RUN_VEL 0x41 /* rpm f32 -> status u32 */
#define MSG_RUN_POS 0x42 /* turns i64 Q32.32 -> status u32 */
#define MSG_STOP 0x43	 /* -> status u32 (ramped stop; aborts a calibration) */
#define MSG_HOME 0x44	 /* -> status u32 */
#define MSG_REBOOT 0x50 /* mode u8: 0 reset, 1 ST bootloader (USB DFU) -> status u32, then the device drops off the bus */
#define MSG_REPLY 0x80	 /* or-ed into the request type */
#define MSG_EVENT 0x7F	 /* no REPLY bit, so it never equals a reply type. Device -> host: code u16, n u8, n x f32; tok = the request that caused it, or 0 */

/* MSG_CAL which */
enum {
	CAL_ENC = 1, /* direction, pole pairs, offset ('e') */
	CAL_R,		 /* 'r' */
	CAL_LD,		 /* 'l', arg = test current A, 0 = default and store */
	CAL_LQ,		 /* 'q', same; hold the shaft */
	CAL_TUNE,	 /* 't' */
	CAL_KV,		 /* 'k', the motor spins */
	CAL_ZERO,	 /* 'z', reset the encoder zero */
	CAL_HFI_MAP, /* HFI saliency map, arg = id bias A, 0 = 0/0.25/0.5 x rated and hfi_id, stores hfi_sal */
	CAL_HFI_POL, /* HFI lock from a random angle + polarity check, compared with the encoder */
	CAL_CUR_OFF, /* current-sense zero offsets, rotor still; stores cur_off */
	CAL_N
};

/* command status (u32). Object access uses CANopen SDO abort codes (SDO_*) in the same field */
typedef enum {
	ST_OK = 0,
	ST_ABORTED,		 /* aborted by user */
	ST_BRIDGE_FAULT, /* bridge fault */
	ST_VBUS_LOW,	 /* bus voltage too low */
	ST_OVERCURRENT,	 /* overcurrent */
	ST_OVERSPEED,	 /* overspeed (1.5 x vlim) */
	ST_BRIDGE_OFF,	 /* bridge disabled (fault latched?) */
	ST_NEED_RATED,	 /* set rated/imax first */
	ST_NEED_R,		 /* run 'r' first */
	ST_NEED_RL,		 /* measure R and L first */
	ST_NEED_BW,		 /* set the bandwidth first */
	ST_NEED_ERLT,	 /* run 'e', 'r', 'l' and 't' first */
	ST_NEED_ET,		 /* run 'e' and 't' first */
	ST_NEED_VGAINS,	 /* set the speed gains first (vkp, vki) */
	ST_NEED_VLIM,	 /* set the speed limit first (vlim) */
	ST_NEED_PKP,	 /* set the position gain first (pkp) */
	ST_VLIMIT,		 /* voltage limit hit before reaching the target current */
	ST_DI_SMALL,	 /* current change too small */
	ST_I_LOW,		 /* current too low */
	ST_I_TEST_HIGH,	 /* test current must stay below 0.9 x imax */
	ST_REC_TIMEOUT,	 /* recording timeout */
	ST_LR_SLOW,		 /* current did not fall to 25% inside the window */
	ST_LR_FAST,		 /* L/R shorter than ~3 samples */
	ST_NO_FOLLOW,	 /* rotor did not follow the field */
	ST_PP_NONINT,	 /* pole pairs not an integer: rotor lost steps */
	ST_KV_SLOW,		 /* too slow to measure back-EMF */
	ST_KV_BACKWARDS, /* spun backwards */
	ST_KV_FEW,		 /* too few samples during coast-down */
	ST_KV_SIGN,		 /* back-EMF has the wrong sign */
	ST_LOOP_DEAD,	 /* control loop not running */
	ST_BUSY,		 /* a calibration is running */
	ST_BAD_ARG,		 /* bad argument / payload length */
	ST_BAD_TYPE,	 /* unknown message type */
	ST_SAVE_FAILED,	 /* flash save failed */
	ST_RUNNING,		 /* not allowed while the motor runs: stop first */
	ST_NEED_HV,		 /* set the injection voltage (hv) first */
	ST_NEED_HMAP,	 /* run the HFI saliency map first (hsal = 0) */
	ST_HFI_NO_SIGNAL, /* no HF current response */
	ST_HFI_NO_LOCK,	 /* HFI did not lock on an axis */
	ST_HFI_POL_WRONG, /* polarity check ended on the wrong side */
	ST_HFI_LOST,	  /* sensorless: HFI stopped tracking (50 ms without a lock) */
	ST_HFI_POL_WEAK, /* sensorless: polarity contrast too low to trust */
	ST_NEED_TUNE,	 /* run the PI autotune ('t') first */
	ST_NEED_KV,		 /* set kv first ('k', or from the datasheet) */
	ST_NEED_ENC_POS, /* position mode needs the encoder (ang_src 0 or 1) */
	ST_VBUS_HIGH,	 /* bus voltage above vbus_max */
	ST_TRIPPED,		 /* a trip is latched: STOP resets it once the motor has coasted down */
	ST_CUR_OFF_BAD,	 /* current-sense offsets implausible (rotor turning? sense stage?) */
	ST_N
} st_t;

#define SDO_NO_OBJECT 0x06020000u /* object does not exist */
#define SDO_NO_SUB 0x06090011u	  /* subindex does not exist */
#define SDO_READ_ONLY 0x06010002u /* attempt to write a read-only object */
#define SDO_BAD_LEN 0x06070010u	  /* data type does not match, length */
#define SDO_RANGE 0x06090030u	  /* value out of range */
#define SDO_STATE 0x08000022u	  /* not allowed in the present device state */

/* events (MSG_EVENT code), the floats each one carries */
enum {
	EV_DONE = 1,	 /* status, which: end of a calibration (motor is off again) */
	EV_STOPPED,		 /* status, id, iq: the run watchdog switched off */
	EV_RPM_LIMITED,	 /* rpm: speed setpoint limited to vlim */
	EV_CUR_LIMITED,	 /* rated A: current setpoint limited */
	EV_RESUMED,		 /* bus V, resumes so far: an overvoltage coast ended by itself, the run carries on */
	EV_ALIGN = 0x10, /* - */
	EV_SWEEP,		 /* electrical revolutions */
	EV_OFFSET_AVG,	 /* - */
	EV_HOLD_SHAFT,	 /* seconds until the q test starts */
	EV_UNLOAD_SHAFT, /* seconds until the motor spins */
	EV_EEPROM_WAIT,	 /* seconds */
	EV_BW_CLAMPED,	 /* bw, bw max (Hz) */
	EV_R = 0x20,	 /* R, v_hi, i_hi, v_lo, i_lo, v_off */
	EV_LCURVE,		 /* I0, v_off: a secant-L curve follows */
	EV_LCURVE_PT,	 /* i (A), L (H) */
	EV_L,			 /* axis (0 d, 1 q), L (H), tau (s), I0 (A), stored (0/1) */
	EV_PP_BAD,		 /* measured pole pairs */
	EV_ENC,			 /* dir, pp, measured pp, single point - average (deg), spread (deg) */
	EV_TUNE,		 /* bw, kp_d, ki_d, kp_q, ki_q */
	EV_KV,			 /* kv, lambda (Wb), Kt (Nm/A), offset (V), samples, peak rpm */
	EV_HFI_PT,		 /* bias (A), angle from d (deg elec), L along it (H), across/along ratio */
	EV_HFI_FIT,		 /* bias (A), a0, a2, b2 (1/H: 1/L = a0 + a2 cos 2phi + b2 sin 2phi), saliency, stored (0/1) */
	EV_HFI_POL,		 /* error before (deg elec, HFI - encoder), error after, flipped (0/1), contrast */
	EV_HFI_START,	 /* sensorless start (tok 0) or sensorless polarity test: contrast, flipped (0/1), HFI - encoder (deg elec) */
	EV_CUR_OFF,		 /* offset a, b, c (counts from 2048), spread a, b, c (counts), stored (0/1) */
};

void proto_init(void); /* starts RX; call once after MX_USART1_UART_Init() and MX_USB_Device_Init() */
void proto_rx_bytes(const uint8_t* p, uint32_t n); /* from the link's RX IRQ */

/* Runs the RX parser over everything received. busy = 0: full dispatch. busy = 1 (from inside a blocking
 * command): READ is served, ABORT/STOP request an abort, everything else gets ST_BUSY. Returns 1 if an
 * abort was requested */
int proto_poll(int busy);

/* queues the frame on the link (link.h), retrying while it is busy or full. Drops after 20 ms */
void proto_send(uint8_t type, uint8_t tok, const void* payload, uint8_t len);
void proto_reply_st(uint8_t type, uint8_t tok, uint32_t st);
void proto_event(uint8_t tok, uint16_t code, int n, const float* v);

extern volatile uint32_t g_proto_err; /* frames dropped: bad CRC, length or timeout */

/* in calib.c */
void cmd_message(uint8_t type, uint8_t tok, const uint8_t* p, uint8_t len); /* SAVE, CAL, RUN_*, STOP, HOME */
extern volatile uint8_t g_busy; /* CAL_* running, 0 = idle */

#endif
