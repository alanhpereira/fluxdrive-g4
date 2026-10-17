#include "objdict.h"

#include <string.h>

#include "bridgeCtrl.h" /* readBridgeStatus() */
#include "main.h"		/* DRV_RDY pin */
#include "calib.h"
#include "co_app.h"   /* 0x4040/0x4041 */
#include "co_store.h" /* co_store_dirty() */
#include "hfi.h" /* HFI_HALF_MAX */
#include "motorCtrl.h" /* g_loop_cyc */
#include "mt6835.h"
#include "params.h"
#include "proto.h"

/* ------------------------------------------------------------------ live values behind 0x4000/0x4001 */
static const uint16_t k_proto_ver = PROTO_VERSION;
static const uint16_t k_param_ver = PARAM_VERSION;
static uint32_t s_hash, s_build;

static struct {
	uint8_t mode, loop, moe, nfault, busy, dirty, brk, ready;
	uint16_t drv; /* gate-driver status byte, 0x100 = I2C error */
	uint32_t flags, seq;
	float vin;
} s_st;

static struct {
	uint8_t nmt, apply;
	uint16_t err;
} s_can;

#define F(x) ((void*)&(x))
#define NOLIM 0.0f, 0.0f
#define BIG 1.0e9f
/* ro: min/max unused. clr: counters that only accept 0 (write 0 clears) */
static const obj_t k_tab[] = {
	/* 0x4000 device info */
	{0x4000, 1, T_U16, OF_RO, F(k_proto_ver), NOLIM},
	{0x4000, 2, T_U32, OF_RO, F(s_hash), NOLIM},
	{0x4000, 3, T_U16, OF_RO, F(k_param_ver), NOLIM},
	{0x4000, 4, T_U32, OF_RO, F(s_build), NOLIM},
	/* 0x4001 status (refreshed on read) */
	{0x4001, 1, T_U8, OF_RO, F(s_st.mode), NOLIM},
	{0x4001, 2, T_U8, OF_RO, F(s_st.loop), NOLIM},
	{0x4001, 3, T_U32, OF_RO, F(s_st.flags), NOLIM},
	{0x4001, 4, T_U8, OF_RO, F(s_st.moe), NOLIM},
	{0x4001, 5, T_U8, OF_RO, F(s_st.nfault), NOLIM},
	{0x4001, 6, T_U16, OF_RO, F(s_st.drv), NOLIM},
	{0x4001, 7, T_F32, OF_RO, F(s_st.vin), NOLIM},
	{0x4001, 8, T_U8, OF_RO, F(s_st.busy), NOLIM},
	{0x4001, 9, T_U8, OF_RO, F(s_st.dirty), NOLIM},
	{0x4001, 10, T_U32, OF_RO, F(s_st.seq), NOLIM},
	{0x4001, 11, T_U8, OF_RO, F(s_st.brk), NOLIM},
	{0x4001, 12, T_U8, OF_RO, F(s_st.ready), NOLIM},
	{0x4001, 13, T_U8, OF_RO, F(g_trip), NOLIM},
	{0x4001, 14, T_U8, OF_RO, F(g_trip_moe), NOLIM},
	{0x4001, 15, T_U8, OF_RO, F(g_coasting), NOLIM},
	/* 0x4002 diagnostics */
	{0x4002, 1, T_U32, OF_RO, F(g_loop_cyc), NOLIM},
	{0x4002, 2, T_U32, 0, F(g_loop_cyc_max), 0.0f, 0.0f},
	{0x4002, 3, T_U32, 0, F(hMT6835.samples), 0.0f, 0.0f},
	{0x4002, 4, T_U32, 0, F(hMT6835.crc_errors), 0.0f, 0.0f},
	{0x4002, 5, T_U32, 0, F(hMT6835.late), 0.0f, 0.0f},
	{0x4002, 6, T_U32, 0, F(hMT6835.stalls), 0.0f, 0.0f},
	{0x4002, 7, T_U8, OF_RO, F(hMT6835.status), NOLIM},
	{0x4002, 8, T_U8, OF_RO, F(hMT6835.crc_check), NOLIM},
	{0x4002, 9, T_U32, 0, F(g_proto_err), 0.0f, 0.0f},
	{0x4002, 10, T_U32, 0, F(g_tlm_drop), 0.0f, 0.0f},
	{0x4002, 11, T_U32, 0, F(g_ov_resumes), 0.0f, 0.0f},
	/* 0x4010 motor */
	{0x4010, 1, T_F32, OF_STORED, F(g_params.kv), 0.0f, 1.0e5f},
	{0x4010, 2, T_U8, OF_STORED | OF_NOTRUN, F(g_params.pole_pairs), 1.0f, 64.0f},
	{0x4010, 3, T_I8, OF_STORED | OF_NOTRUN, F(g_params.enc_dir), -1.0f, 1.0f},
	{0x4010, 4, T_U32, OF_STORED | OF_NOTRUN, F(g_params.elec_offset), 0.0f, 4294967296.0f},
	{0x4010, 5, T_F32, OF_STORED, F(g_params.rd), 0.0f, 100.0f},
	{0x4010, 6, T_F32, OF_STORED, F(g_params.rq), 0.0f, 100.0f},
	{0x4010, 7, T_F32, OF_STORED, F(g_params.ld), 0.0f, 1.0f},
	{0x4010, 8, T_F32, OF_STORED, F(g_params.lq), 0.0f, 1.0f},
	{0x4010, 9, T_F32, OF_STORED, F(g_params.rated_current), 0.0f, 50.0f},
	{0x4010, 10, T_F32, OF_STORED, F(g_params.max_current), 0.0f, 50.0f},
	{0x4010, 11, T_U32, OF_STORED | OF_NOTRUN, F(g_params.flags), 0.0f, 255.0f},
	/* 0x4011 angle source */
	{0x4011, 1, T_U8, OF_STORED | OF_NOTRUN, F(g_params.ang_src), 0.0f, 2.0f},
	/* 0x4020 current loop */
	{0x4020, 1, T_F32, OF_STORED | OF_PI, F(g_params.kp_d), 0.0f, 1.0e6f},
	{0x4020, 2, T_F32, OF_STORED | OF_PI, F(g_params.ki_d), 0.0f, 1.0e6f},
	{0x4020, 3, T_F32, OF_STORED | OF_PI, F(g_params.kp_q), 0.0f, 1.0e6f},
	{0x4020, 4, T_F32, OF_STORED | OF_PI, F(g_params.ki_q), 0.0f, 1.0e6f},
	{0x4020, 5, T_F32, OF_STORED, F(g_params.cur_BW), 0.0f, CTRL_FS_HZ / 20.0f},
	/* 0x4021 speed/position */
	{0x4021, 1, T_F32, OF_STORED | OF_PI, F(g_params.vel_kp), 0.0f, 1.0e4f},
	{0x4021, 2, T_F32, OF_STORED | OF_PI, F(g_params.vel_ki), 0.0f, 1.0e4f},
	{0x4021, 3, T_F32, OF_STORED, F(g_params.pos_kp), 0.0f, 1.0e4f},
	{0x4021, 4, T_F32, OF_STORED, F(g_params.vel_lim), 0.0f, 1000.0f},
	{0x4021, 5, T_F32, OF_STORED, F(g_params.acc_lim), 0.0f, 1.0e7f},
	/* 0x4022 limits */
	{0x4022, 1, T_F32, OF_STORED, F(g_params.vbus_min), 0.0f, 100.0f},
	{0x4022, 2, T_F32, OF_STORED, F(g_params.vbus_max), 0.0f, 100.0f},
	{0x4022, 3, T_F32, OF_STORED, F(g_params.temp_max), -40.0f, 200.0f},
	{0x4022, 4, T_F32, OF_STORED, F(g_params.vbus_clamp), 0.0f, 20.0f},
	{0x4022, 5, T_F32, OF_STORED, F(g_params.vbus_resume), 0.0f, 20.0f},
	/* 0x4023 current sensing: stored offsets (CAL_CUR_OFF), then the ones in use (boot measurement or fallback) */
	{0x4023, 1, T_I16, OF_STORED | OF_NOTRUN, F(g_params.cur_off[0]), -300.0f, 300.0f},
	{0x4023, 2, T_I16, OF_STORED | OF_NOTRUN, F(g_params.cur_off[1]), -300.0f, 300.0f},
	{0x4023, 3, T_I16, OF_STORED | OF_NOTRUN, F(g_params.cur_off[2]), -300.0f, 300.0f},
	{0x4023, 4, T_I16, OF_RO, F(g_cs_off[0]), NOLIM},
	{0x4023, 5, T_I16, OF_RO, F(g_cs_off[1]), NOLIM},
	{0x4023, 6, T_I16, OF_RO, F(g_cs_off[2]), NOLIM},
	{0x4023, 7, T_U8, OF_RO, F(g_cs_off_src), NOLIM},
	/* 0x4030 observer */
	{0x4030, 1, T_U8, OF_STORED, F(g_params.obs_blend), 0.0f, 1.0f},
	{0x4030, 2, T_F32, OF_STORED, F(g_params.obs_gain), 0.0f, 1.0e6f},
	{0x4030, 3, T_F32, OF_STORED, F(g_params.obs_lo_v), 0.0f, 100.0f},
	{0x4030, 4, T_F32, OF_STORED, F(g_params.obs_hi_v), 0.0f, 100.0f},
	{0x4030, 5, T_F32, OF_STORED, F(g_params.obs_flux_tol), 0.0f, 1.0f},
	{0x4030, 6, T_F32, OF_STORED, F(g_params.obs_err_max), 0.0f, 180.0f},
	/* 0x4031 high-frequency injection */
	{0x4031, 1, T_U8, OF_STORED | OF_NOTRUN, F(g_params.hfi_mode), 0.0f, 1.0f},
	{0x4031, 2, T_U8, OF_STORED | OF_NOTRUN, F(g_params.hfi_half), 2.0f, (float)HFI_HALF_MAX},
	{0x4031, 3, T_F32, OF_STORED, F(g_params.hfi_v), 0.0f, 10.0f},
	{0x4031, 4, T_F32, OF_STORED, F(g_params.hfi_bw), 0.0f, 500.0f},
	{0x4031, 5, T_F32, OF_STORED, F(g_params.hfi_id), 0.0f, 50.0f},
	{0x4031, 6, T_F32, OF_STORED, F(g_params.hfi_sal), -1.0f, 1.0f},
	{0x4031, 7, T_F32, OF_STORED, F(g_params.hfi_pol_i), 0.0f, 50.0f},
	/* 0x4040 CAN settings: comm record (co_store.c), saved by MSG_SAVE; used from the next communication reset */
	{0x4040, 1, T_U8, OF_STORED, F(g_can_node_id), 1.0f, 127.0f},
	{0x4040, 2, T_U16, OF_STORED, F(g_can_kbit), 20.0f, 1000.0f}, /* 20, 50, 125, 250, 500, 1000 */
	{0x4040, 3, T_U8, OF_RO, F(g_can_node_id_act), NOLIM},
	{0x4040, 4, T_U16, OF_RO, F(g_can_kbit_act), NOLIM},
	{0x4040, 5, T_U8, OF_NOTRUN, F(s_can.apply), 1.0f, 1.0f}, /* write 1: communication reset with 0x4040:1/2 */
	/* 0x4041 CAN status (refreshed on read) */
	{0x4041, 1, T_U8, OF_RO, F(s_can.nmt), NOLIM}, /* 0 initializing, 4 stopped, 5 operational, 127 pre-operational */
	{0x4041, 2, T_U16, OF_RO, F(s_can.err), NOLIM}, /* CO_CAN_ERR* bits */
	{0x4041, 3, T_U32, 0, F(g_can_rx), 0.0f, 0.0f},
	{0x4041, 4, T_U32, 0, F(g_can_tx), 0.0f, 0.0f},
	{0x4041, 5, T_U32, 0, F(g_can_rx_lost), 0.0f, 0.0f},
};
#define N_OBJ ((int)(sizeof k_tab / sizeof k_tab[0]))

static const uint8_t k_size[] = {0, 1, 1, 2, 4, 4, 4, 8, 2}; /* by type */

/* FNV-1a, 32 bit */
static uint32_t fnv(uint32_t h, uint8_t b) {
	return (h ^ b) * 16777619u;
}

void objdict_init(void) {
	uint32_t h = 2166136261u;
	for (int i = 0; i < N_OBJ; i++) {
		const obj_t* o = &k_tab[i];
		h = fnv(h, (uint8_t)o->index);
		h = fnv(h, (uint8_t)(o->index >> 8));
		h = fnv(h, o->sub);
		h = fnv(h, o->type);
		h = fnv(h, o->flags);
	}
	s_hash = h;

	/* build id: the whole flash image (code, constants, .data init), so any rebuild changes it */
	extern uint32_t _sidata, _sdata, _edata;
	const uint32_t* w = (const uint32_t*)FLASH_BASE;
	const uint32_t* end = &_sidata + (&_edata - &_sdata);
	h = 2166136261u;
	while (w < end) h = (h ^ *w++) * 16777619u;
	s_build = h;
}

uint32_t objdict_hash(void) { return s_hash; }
uint32_t objdict_build_id(void) { return s_build; }

static const obj_t* find(uint16_t index, uint8_t sub, uint32_t* err) {
	int idx_seen = 0;
	for (int i = 0; i < N_OBJ; i++) {
		if (k_tab[i].index != index) continue;
		idx_seen = 1;
		if (k_tab[i].sub == sub) return &k_tab[i];
	}
	*err = idx_seen ? SDO_NO_SUB : SDO_NO_OBJECT;
	return NULL;
}

static void status_refresh(uint8_t sub) {
	s_st.mode = (uint8_t)g_mode;
	s_st.loop = (uint8_t)g_loop;
	s_st.flags = g_params.flags;
	s_st.moe = (TIM1->BDTR & TIM_BDTR_MOE) ? 1 : 0;
	s_st.nfault = (GPIOE->IDR & GPIO_PIN_15) ? 1 : 0; /* 1 = high = no fault */
	s_st.vin = g_vin;
	s_st.busy = g_busy;
	s_st.dirty = (uint8_t)(params_dirty() || co_store_dirty());
	s_st.seq = g_params.seq;
	s_st.brk = (TIM1->SR & (TIM_SR_BIF | TIM_SR_B2IF)) ? 1 : 0; /* TIM1 break latched */
	s_st.ready = (DRV_RDY_GPIO_Port->IDR & DRV_RDY_Pin) ? 1 : 0;
	if (sub == 6) { /* I2C, ~100 us: only when asked */
		uint8_t b = 0;
		s_st.drv = readBridgeStatus(&b) == HAL_OK ? b : 0x100;
	}
}

uint32_t od_read(uint16_t index, uint8_t sub, uint8_t* out, uint8_t* len) {
	uint32_t err;
	const obj_t* o = find(index, sub, &err);
	if (!o) return err;
	if (index == 0x4001) status_refresh(sub);
	if (index == 0x4041) {
		s_can.nmt = co_app_nmt_state();
		s_can.err = co_app_can_state();
	}
	*len = k_size[o->type];
	memcpy(out, (const void*)o->p, *len);
	return 0;
}

uint32_t od_write(uint16_t index, uint8_t sub, const uint8_t* v, uint8_t len) {
	uint32_t err;
	const obj_t* o = find(index, sub, &err);
	if (!o) return err;
	if (o->flags & OF_RO) return SDO_READ_ONLY;
	if (len != k_size[o->type]) return SDO_BAD_LEN;
	if ((o->flags & OF_NOTRUN) && g_mode != CTRL_OFF) return SDO_STATE;

	float x; /* range check in float: exact enough for these limits */
	switch (o->type) {
		case T_U8:
			x = (float)v[0];
			break;
		case T_I8:
			x = (float)(int8_t)v[0];
			break;
		case T_U16:
			x = (float)(uint16_t)(v[0] | v[1] << 8);
			break;
		case T_I16:
			x = (float)(int16_t)(v[0] | v[1] << 8);
			break;
		case T_U32: {
			uint32_t u;
			memcpy(&u, v, 4);
			x = (float)u;
			break;
		}
		case T_I32: {
			int32_t s;
			memcpy(&s, v, 4);
			x = (float)s;
			break;
		}
		case T_F32:
			memcpy(&x, v, 4);
			break;
		default:
			return SDO_BAD_LEN;
	}
	if (!(x >= o->min && x <= o->max)) return SDO_RANGE; /* also NaN */
	if (o->p == &g_params.enc_dir && x == 0.0f) return SDO_RANGE;
	if (o->p == &g_can_kbit && !co_can_prescaler((uint16_t)x)) return SDO_RANGE;
	if (o->p == &s_can.apply) { /* a command, not a value */
		co_app_request_reset();
		return 0;
	}

	memcpy(o->p, v, len); /* aligned 1/2/4-byte stores: atomic for the ISR */
	if (o->flags & OF_PI) pi_apply();
	return 0;
}

int od_list(uint16_t first, uint8_t* out, int max) {
	int n = 0;
	for (int i = first; i < N_OBJ && n < max; i++, n++) {
		const obj_t* o = &k_tab[i];
		*out++ = (uint8_t)o->index;
		*out++ = (uint8_t)(o->index >> 8);
		*out++ = o->sub;
		*out++ = o->type;
		*out++ = o->flags;
	}
	return n;
}
