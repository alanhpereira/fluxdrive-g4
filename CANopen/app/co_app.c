#include "co_app.h"

#include "CANopen.h"
#include "OD.h"
#include "co402.h"
#include "co_store.h"
#include "storage/CO_storage.h"
#include "boot.h"		/* boot_reboot() */
#include "bridgeCtrl.h" /* disableBridge() */
#include "calib.h"		/* g_mode */
#include "fdcan.h"
#include "objdict.h" /* od_read_fast(), od_write(): the native objects */
#include "params.h" /* PARAM_VERSION */
#include "proto.h"	/* PROTO_VERSION */

/* NMT: start pre-operational (the master starts the node). Communication errors (bus-off, heartbeat consumer
 * timeout, the communication bit of 0x1001) drop an operational node to pre-operational; drive faults do not */
#define NMT_CONTROL (CO_NMT_ERR_ON_BUSOFF_HB | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_COMMUNICATION)
#define FIRST_HB_MS 500
#define SDO_SRV_TIMEOUT_MS 1000

volatile uint8_t g_can_node_id = 1;
volatile uint16_t g_can_kbit = 1000;
volatile uint8_t g_can_node_id_act;
volatile uint16_t g_can_kbit_act;

static CO_t* s_co;
static volatile uint8_t s_run;		/* the tick processes the stack */
static volatile uint8_t s_reset_req; /* CO_RESET_COMM / CO_RESET_APP from the tick or 0x4040:5, done in main */

/* 0x1010 store / 0x1011 restore: sub 2 = communication (OD_PERSIST_COMM), sub 3 = application (the CiA 402
 * settings, OD_PERSIST_APP, and for store also the motor params), sub 1 = all. Restore never touches the motor
 * calibration. They run in the tick, which must not erase flash: leave a request for main */
static CO_storage_t s_storage;
static CO_storage_entry_t s_store_entries[] = {
	{.addr = &OD_PERSIST_COMM, .len = sizeof(OD_PERSIST_COMM), .subIndexOD = 2, .attr = CO_storage_cmd | CO_storage_restore},
#ifdef OD_ATTR_PERSIST_APP
	{.addr = &OD_PERSIST_APP, .len = sizeof(OD_PERSIST_APP), .subIndexOD = 3, .attr = CO_storage_cmd | CO_storage_restore},
#endif
	{.addr = &g_params, .len = sizeof(g_params), .subIndexOD = 3, .attr = CO_storage_cmd},
};

/* refused here, not dropped later: cmd_poll() does not save under a running motor or a latched trip */
static ODR_t store_cb(CO_storage_entry_t* e, CO_CANmodule_t* m) {
	(void)m;
	if (g_mode != CTRL_OFF || g_trip) return ODR_DATA_DEV_STATE; /* the flash erase would stall the running control loop */
	co_store_request(e->subIndexOD == 3 ? CO_RQ_SAVE_APP : CO_RQ_SAVE_COMM);
	return ODR_OK;
}

static ODR_t restore_cb(CO_storage_entry_t* e, CO_CANmodule_t* m) {
	(void)m;
	if (g_mode != CTRL_OFF || g_trip) return ODR_DATA_DEV_STATE;
	co_store_request(e->addr == (void*)&OD_PERSIST_COMM ? CO_RQ_RESTORE_COMM : CO_RQ_RESTORE_APP);
	return ODR_OK;
}

/* ------------------------------------------------------------------ native objects 0x4000-0x40FF over SDO
 * The OD holds them (exported from driverTCC.xdd) so masters and the EDS see them, but the values live in the
 * firmware's own variables: reads and writes go through objdict.c, with the same range checks and side effects
 * (pi_apply(), OF_NOTRUN...) as the USB/serial protocol. Sub-index 0 stays the OD's own */
int co_app_abort_to_odr(uint32_t ab) {
	switch (ab) {
		case SDO_NO_OBJECT: return ODR_IDX_NOT_EXIST;
		case SDO_NO_SUB: return ODR_SUB_NOT_EXIST;
		case SDO_READ_ONLY: return ODR_READONLY;
		case SDO_BAD_LEN: return ODR_TYPE_MISMATCH;
		case SDO_RANGE: return ODR_INVALID_VALUE;
		case SDO_STATE: return ODR_DATA_DEV_STATE;
		default: return ODR_GENERAL;
	}
}

static ODR_t native_read(OD_stream_t* st, void* buf, OD_size_t count, OD_size_t* n) {
	if (st->subIndex == 0) return OD_readOriginal(st, buf, count, n);
	uint8_t v[8], len = 0;
	const uint32_t ab = od_read_fast(st->index, st->subIndex, v, &len);
	if (ab) return (ODR_t)co_app_abort_to_odr(ab);
	if (count < len) return ODR_DEV_INCOMPAT; /* all native values are 1..4 bytes: never segmented */
	memcpy(buf, v, len);
	*n = len;
	return ODR_OK;
}

static ODR_t native_write(OD_stream_t* st, const void* buf, OD_size_t count, OD_size_t* n) {
	if (st->subIndex == 0) return ODR_READONLY;
	if (count > 8) return ODR_DATA_LONG;
	const uint32_t ab = od_write(st->index, st->subIndex, buf, (uint8_t)count);
	if (ab) return (ODR_t)co_app_abort_to_odr(ab);
	*n = count;
	return ODR_OK;
}

static OD_extension_t s_native_ext = {.object = NULL, .read = native_read, .write = native_write};

static void native_init(void) {
	for (uint16_t i = 0; i < OD->size; i++) {
		OD_entry_t* e = &OD->list[i];
		if (e->index >= 0x4000 && e->index <= 0x40FF) OD_extension_init(e, &s_native_ext);
	}
}

/* serial number: the 96-bit unique device id folded with FNV-1a */
static uint32_t uid_serial(void) {
	const uint32_t* u = (const uint32_t*)UID_BASE;
	uint32_t h = 2166136261u;
	for (int i = 0; i < 3; i++)
		for (int b = 0; b < 32; b += 8) h = (h ^ ((u[i] >> b) & 0xFFu)) * 16777619u;
	return h;
}

/* (re)initialises the stack with the pending node ID and bit rate; the tick is held off meanwhile */
static void comm_reset(void) {
	s_run = 0;
	/* let the frames already queued go out first (the SDO reply to 0x4040:5, an EMCY): stopping the FDCAN drops
	 * them. 20 ms is three frames at 20 kbit/s; without an ACK they never go, so do not wait longer */
	const uint32_t t0 = HAL_GetTick();
	while (hfdcan1.State == HAL_FDCAN_STATE_BUSY && hfdcan1.Instance->TXBRP && HAL_GetTick() - t0 < 20) {
	}
	if (s_co) {
		CO_CANsetConfigurationMode(&hfdcan1);
		CO_delete(s_co);
	}
	uint32_t heap = 0;
	s_co = CO_new(NULL, &heap);
	if (!s_co) return;

	uint8_t id = g_can_node_id;
	uint16_t kbit = g_can_kbit;
	if (id < 1 || id > 127) id = 1; /* the stored values are range-checked already; belt and braces */
	if (!co_can_prescaler(kbit)) kbit = 1000;

	co_store_apply_od(); /* CiA 301: a communication reset reloads the stored communication parameters */
	OD_PERSIST_COMM.x1018_identity.revisionNumber = ((uint32_t)PROTO_VERSION << 16) | PARAM_VERSION;
	OD_PERSIST_COMM.x1018_identity.serialNumber = uid_serial();

	s_co->CANmodule->CANnormal = false;
	CO_CANsetConfigurationMode(&hfdcan1);
	CO_CANmodule_disable(s_co->CANmodule);
	if (CO_CANinit(s_co, &hfdcan1, kbit) != CO_ERROR_NO) return;
	if (CO_storage_init(&s_storage, s_co->CANmodule, OD_ENTRY_H1010_storeParameters, OD_ENTRY_H1011_restoreDefaultParameters, store_cb,
						restore_cb, s_store_entries, (uint8_t)(sizeof s_store_entries / sizeof s_store_entries[0])) == CO_ERROR_NO)
		s_storage.enabled = true;
	uint32_t errInfo = 0;
	if (CO_CANopenInit(s_co, NULL, NULL, OD, NULL, NMT_CONTROL, FIRST_HB_MS, SDO_SRV_TIMEOUT_MS, 0, false, id, &errInfo) != CO_ERROR_NO) return;
	if (CO_CANopenInitPDO(s_co, s_co->em, OD, id, &errInfo) != CO_ERROR_NO) return;
	co402_comm_reset();
	g_can_node_id_act = id;
	g_can_kbit_act = kbit;
	CO_CANsetNormalMode(s_co->CANmodule);
	s_run = 1;
}

void co_app_init(void) {
	native_init();
	co402_init();
	co_store_load(); /* node ID and bit rate */
	comm_reset();

	/* 1 kHz tick: TIM7 (basic timer, not in the .ioc) at 170 MHz / 170 / 1000 */
	__HAL_RCC_TIM7_CLK_ENABLE();
	TIM7->PSC = 169;
	TIM7->ARR = 999;
	TIM7->EGR = TIM_EGR_UG;
	TIM7->SR = 0;
	TIM7->DIER = TIM_DIER_UIE;
	TIM7->CR1 = TIM_CR1_CEN;
	HAL_NVIC_SetPriority(TIM7_IRQn, CO_IRQ_PRIO, 0);
	HAL_NVIC_EnableIRQ(TIM7_IRQn);
}

void co_app_request_reset(void) {
	if (s_reset_req == 0) s_reset_req = CO_RESET_COMM;
}

void co_app_poll(void) {
	const uint8_t r = s_reset_req;
	if (r == CO_RESET_COMM) {
		s_reset_req = 0;
		comm_reset();
	}
	else if (r == CO_RESET_APP) { /* NMT reset node: like MSG_REBOOT, never under a running motor */
		if (g_mode == CTRL_OFF) {
			disableBridge();
			boot_reboot(0);
		}
		/* wait for it to stop, but keep the stack running meanwhile (heartbeat, SDO, NMT): the command is consumed,
		 * and a silent node would leave the motor on its last reference with nobody able to reach it. A motor the
		 * master drives coasts now; one the USB side runs is left to it, and the reset follows its stop */
		if (g_can_owner && g_mode == CTRL_RUN && !g_coasting && !g_trip) drive_coast();
		s_run = 1;
	}
}

/* the CANopen tick, 1 ms */
void TIM7_IRQHandler(void) {
	TIM7->SR = 0;
	if (!s_run) return; /* no lock: the FDCAN IRQ has the same priority and cannot preempt this */
	const CO_NMT_reset_cmd_t rc = CO_process(s_co, false, 1000, NULL);
	if (rc == CO_RESET_COMM || rc == CO_RESET_APP) {
		if ((uint8_t)rc > s_reset_req) s_reset_req = (uint8_t)rc; /* a reset node waiting for the motor stays pending */
		s_run = 0; /* main does the reset */
	}
	else if (!s_co->nodeIdUnconfigured && s_co->CANmodule->CANnormal) {
		const bool_t sync = CO_process_SYNC(s_co, 1000, NULL);
		CO_process_RPDO(s_co, sync, 1000, NULL);
		co402_tick(); /* references from the fresh RPDO data, actual values for the TPDOs */
		CO_process_TPDO(s_co, sync, 1000, NULL);
	}
}

void* co_app_co(void) {
	return s_co;
}

uint8_t co_app_nmt_state(void) {
	return s_co ? (uint8_t)s_co->NMT->operatingState : 0;
}

uint16_t co_app_can_state(void) {
	return s_co ? s_co->CANmodule->CANerrorStatus : 0;
}
