#include "params.h"

#include <stddef.h>
#include <string.h>

/* Two 2 KB pages at the very end of the 128 KB flash (pages 62 and 63).
 * Reserve them in the linker script: set the FLASH length to 124K. */
#define SLOT_ADDR(i) (FLASH_BASE + (uint32_t)(62 + (i)) * FLASH_PAGE_SIZE)
#define SLOT_PAGE(i) (62u + (uint32_t)(i))

motor_params_t g_params;
static int g_active = -1; /* slot currently holding the newest valid copy, -1 = none */

/* Placeholders: replace with your motor's values. */
static const motor_params_t k_defaults = {
	.kv = 0.0f, /* unknown */
	.rd = 1.0f,
	.rq = 1.0f,
	.ld = 0.0005f,
	.lq = 0.0005f,
	.rated_current = 1.0f,
	.max_current = 1.5f,
	.elec_offset = 0,
	.pole_pairs = 7,
	.enc_dir = 1,
	.cur_scale = 0.0f, /* set from your shunt/gain, sign included */
	.cur_off = {0, 0, 0},
	.vin_scale = 0.0f,
	.kp_d = 0.0f,
	.ki_d = 0.0f,
	.kp_q = 0.0f,
	.ki_q = 0.0f,
	.vbus_min = 8.0f,
	.vbus_max = 30.0f,
	.temp_max = 80.0f,
	.vel_kp = 0.0f, /* tune before speed/position mode */
	.vel_ki = 0.0f,
	.pos_kp = 20.0f,
	.vel_lim = 10.0f,
	.obs_gain = 1000.0f,
	.obs_lo_v = 0.7f,
	.obs_hi_v = 1.4f,
	.obs_flux_tol = 0.3f,
	.obs_err_max = 30.0f,
	.acc_lim = 60000.0f,
	.obs_blend = 1,
};

static uint32_t crc32(const void* p, size_t n) {
	const uint8_t* b = (const uint8_t*)p;
	uint32_t c = 0xFFFFFFFFu;
	while (n--) {
		c ^= *b++;
		for (int i = 0; i < 8; i++) c = (c >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(c & 1u));
	}
	return ~c;
}

static const motor_params_t* slotPtr(int i) {
	return (const motor_params_t*)SLOT_ADDR(i);
}

/* only the current layout is accepted: any other version or size loads the defaults */
static int isValid(const motor_params_t* p) {
	if (p->magic != PARAM_MAGIC || p->version != PARAM_VERSION || p->size != sizeof(motor_params_t)) return 0;
	return p->crc == crc32(p, sizeof(motor_params_t) - sizeof(uint32_t));
}

int params_load(void) {
	const motor_params_t* a = slotPtr(0);
	const motor_params_t* b = slotPtr(1);
	int va = isValid(a), vb = isValid(b);

	if (va && vb) {
		g_active = ((int32_t)(a->seq - b->seq) >= 0) ? 0 : 1; /* wrap-safe compare */
	}
	else if (va) {
		g_active = 0;
	}
	else if (vb) {
		g_active = 1;
	}
	else {
		g_params = k_defaults;
		g_active = -1;
		return 1;
	}
	g_params = *slotPtr(g_active);
	return 0;
}

int params_dirty(void) {
	if (g_active < 0) return 1;
	const size_t a = offsetof(motor_params_t, flags), b = offsetof(motor_params_t, crc);
	return memcmp((const uint8_t*)&g_params + a, (const uint8_t*)slotPtr(g_active) + a, b - a) != 0;
}

HAL_StatusTypeDef params_save(void) {
	if (TIM1->BDTR & TIM_BDTR_MOE) return HAL_BUSY; /* never stall the CPU with the bridge live */

	__ALIGNED(8)
	motor_params_t tmp = g_params;
	tmp.magic = PARAM_MAGIC;
	tmp.version = PARAM_VERSION;
	tmp.size = sizeof(motor_params_t);
	tmp.seq = (g_active >= 0) ? slotPtr(g_active)->seq + 1u : 1u;
	tmp.crc = crc32(&tmp, sizeof(tmp) - sizeof(uint32_t));

	int target = (g_active == 0) ? 1 : 0; /* write the other slot, the old copy stays valid until we finish */

	HAL_StatusTypeDef st = HAL_FLASH_Unlock();
	if (st != HAL_OK) return st;
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

	FLASH_EraseInitTypeDef er = {
		.TypeErase = FLASH_TYPEERASE_PAGES,
		.Banks = FLASH_BANK_1,
		.Page = SLOT_PAGE(target),
		.NbPages = 1,
	};
	uint32_t page_err = 0;
	st = HAL_FLASHEx_Erase(&er, &page_err);

	for (uint32_t off = 0; st == HAL_OK && off < sizeof(tmp); off += 8) {
		uint64_t w;
		memcpy(&w, (const uint8_t*)&tmp + off, 8);
		st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, SLOT_ADDR(target) + off, w);
	}
	HAL_FLASH_Lock();
	if (st != HAL_OK) return st;

	if (!isValid(slotPtr(target)) || memcmp(slotPtr(target), &tmp, sizeof(tmp)) != 0) return HAL_ERROR;

	g_active = target;
	g_params.seq = tmp.seq;
	return HAL_OK;
}
