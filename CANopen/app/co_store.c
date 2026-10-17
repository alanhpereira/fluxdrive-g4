#include "co_store.h"

#include <stddef.h>
#include <string.h>

#include "301/CO_driver.h" /* CO_LOCK_OD */
#include "CANopen.h"
#include "OD.h"
#include "co_app.h"
#include "params.h" /* flash_crc32(), flash_write_page_from() */

/* pages 60 and 61, between the program (FLASH LENGTH 120K) and the motor params (62-63) */
#define SLOT_PAGE(i) (60u + (uint32_t)(i))
#define SLOT_ADDR(i) (FLASH_BASE + SLOT_PAGE(i) * FLASH_PAGE_SIZE)

#define REC_MAGIC 0x434F4D31u /* 'COM1' */
#define REC_VERSION 2u
#define OD_MAX 1536u /* room for the OD parts; fixed, so an OD re-export keeps the node ID and bit rate */

/* the OD storage groups kept here: [0] communication (0x1000-0x1FFF), [1] CiA 402 settings (if the OD has it) */
#ifdef OD_ATTR_PERSIST_APP
#define N_PARTS 2
#else
#define N_PARTS 1
#endif
static const struct {
	void* p;
	uint16_t len;
} k_part[N_PARTS] = {
	{&OD_PERSIST_COMM, sizeof(OD_PERSIST_COMM)},
#ifdef OD_ATTR_PERSIST_APP
	{&OD_PERSIST_APP, sizeof(OD_PERSIST_APP)},
#endif
};
#define PART_COMM 0
#define PART_APP 1

typedef struct {
	uint32_t magic;
	uint16_t version;
	uint16_t size;
	uint32_t seq;
	uint8_t node_id;
	uint8_t od_valid; /* bit per part: 0 = not saved, or restored by 0x1011 (the exported defaults apply) */
	uint16_t kbit;
	uint32_t od_sig; /* od_signature() when saved */
	uint16_t od_len; /* parts back to back */
	uint16_t reserved;
} rec_hdr_t;

typedef struct {
	rec_hdr_t h;
	uint8_t od[OD_MAX];
	uint32_t reserved2;
	uint32_t crc; /* CRC32 of everything above; keep last */
} rec_t;

_Static_assert(sizeof(rec_t) % 8 == 0, "flash programs 64-bit words");
_Static_assert(sizeof(rec_t) <= FLASH_PAGE_SIZE, "one record per page");
_Static_assert(offsetof(rec_t, od) == sizeof(rec_hdr_t), "no padding after the header");
_Static_assert(OD_MAX >= sizeof(OD_PERSIST_COMM)
#ifdef OD_ATTR_PERSIST_APP
							  + sizeof(OD_PERSIST_APP)
#endif
			   ,
			   "the OD parts grew: raise OD_MAX");

static int s_active = -1; /* slot with the newest valid record, -1 = none */
static volatile uint8_t s_rq;

static const rec_t* slot(int i) {
	return (const rec_t*)SLOT_ADDR(i);
}

static int valid(const rec_t* r) {
	if (r->h.magic != REC_MAGIC || r->h.version != REC_VERSION || r->h.size != sizeof(rec_t)) return 0;
	return r->crc == flash_crc32(r, sizeof(rec_t) - sizeof(uint32_t));
}

static uint16_t parts_len(void) {
	uint16_t n = 0;
	for (int i = 0; i < N_PARTS; i++) n += k_part[i].len;
	return n;
}

static uint16_t part_off(int i) {
	uint16_t n = 0;
	for (int k = 0; k < i; k++) n += k_part[k].len;
	return n;
}

/* FNV-1a over the OD table of contents (index, sub-entry count, object type) and the part sizes: changes whenever
 * a re-export could move fields inside the stored parts */
static uint32_t od_signature(void) {
	uint32_t h = 2166136261u;
	for (int i = 0; i < N_PARTS; i++)
		for (int b = 0; b < 16; b += 8) h = (h ^ ((k_part[i].len >> b) & 0xFFu)) * 16777619u;
	for (uint16_t i = 0; i < OD->size; i++) {
		const OD_entry_t* e = &OD->list[i];
		const uint8_t v[4] = {(uint8_t)e->index, (uint8_t)(e->index >> 8), e->subEntriesCount, e->odObjectType};
		for (int k = 0; k < 4; k++) h = (h ^ v[k]) * 16777619u;
	}
	return h;
}

/* the stored copy of part i, or NULL if there is none usable */
static const uint8_t* stored(int i) {
	if (s_active < 0) return NULL;
	const rec_t* r = slot(s_active);
	if (!(r->h.od_valid & (1u << i)) || r->h.od_len != parts_len() || r->h.od_sig != od_signature()) return NULL;
	return r->od + part_off(i);
}

/* the exported defaults of part i: initialised data, so the initial image is in flash at _sidata */
static const uint8_t* defaults(int i) {
	extern const uint8_t _sidata[], _sdata[], _edata[]; /* linker symbols: arrays, so no object size is assumed */
	const uint8_t* p = k_part[i].p;
	if (p < _sdata || p + k_part[i].len > _edata) return NULL;
	return _sidata + (p - _sdata);
}

static void apply(int i) {
	const uint8_t* src = stored(i);
	if (!src) src = defaults(i);
	if (src) memcpy(k_part[i].p, src, k_part[i].len);
}

void co_store_load(void) {
	const int va = valid(slot(0)), vb = valid(slot(1));
	if (va && vb)
		s_active = ((int32_t)(slot(0)->h.seq - slot(1)->h.seq) >= 0) ? 0 : 1; /* wrap-safe */
	else
		s_active = va ? 0 : vb ? 1 : -1;
	if (s_active >= 0) {
		const rec_t* r = slot(s_active);
		if (r->h.node_id >= 1 && r->h.node_id <= 127) g_can_node_id = r->h.node_id;
		if (co_can_prescaler(r->h.kbit)) g_can_kbit = r->h.kbit;
	}
#if N_PARTS > 1
	apply(PART_APP); /* the drive profile settings: once, at boot (a communication reset does not touch them) */
#endif
}

void co_store_apply_od(void) {
	apply(PART_COMM);
}

int co_store_dirty(void) {
	if (s_active < 0) return 1;
	const rec_t* r = slot(s_active);
	if (r->h.node_id != g_can_node_id || r->h.kbit != g_can_kbit) return 1;
	for (int i = 0; i < N_PARTS; i++) {
		const uint8_t* ref = stored(i);
		if (!ref) ref = defaults(i);
		if (!ref) continue;
		const uint8_t* cur = k_part[i].p;
		if (i == PART_COMM) { /* the identity's revision and serial are filled in at run time: compare the rest */
			const size_t id = offsetof(OD_PERSIST_COMM_t, x1018_identity), n = sizeof(OD_PERSIST_COMM.x1018_identity);
			if (memcmp(ref, cur, id) || memcmp(ref + id + n, cur + id + n, k_part[i].len - id - n)) return 1;
		}
		else if (memcmp(ref, cur, k_part[i].len))
			return 1;
	}
	return 0;
}

/* ------------------------------------------------------------------ save: the record is streamed into flash from its
 * sources (the header on the stack, the OD parts in place), so it needs no 1.5 KB RAM copy */
typedef struct {
	rec_hdr_t h;
	uint32_t crc;
} save_ctx_t;

static uint8_t rec_byte(const save_ctx_t* c, uint32_t off) {
	const uint32_t od0 = offsetof(rec_t, od);
	if (off < od0) return ((const uint8_t*)&c->h)[off];
	if (off < od0 + OD_MAX) {
		uint32_t o = off - od0;
		for (int i = 0; i < N_PARTS; i++) {
			if (o < k_part[i].len) return ((const uint8_t*)k_part[i].p)[o];
			o -= k_part[i].len;
		}
		return 0xFF;
	}
	if (off < offsetof(rec_t, crc)) return 0; /* reserved2 */
	return (uint8_t)(c->crc >> (8u * (off - offsetof(rec_t, crc))));
}

static void rec_get8(uint32_t off, uint8_t* dst, void* ctx) {
	for (uint32_t i = 0; i < 8; i++) dst[i] = rec_byte(ctx, off + i);
}

HAL_StatusTypeDef co_store_save(uint8_t rq) {
	save_ctx_t c;
	memset(&c, 0, sizeof c);
	c.h.magic = REC_MAGIC;
	c.h.version = REC_VERSION;
	c.h.size = sizeof(rec_t);
	c.h.seq = (s_active >= 0) ? slot(s_active)->h.seq + 1u : 1u;
	c.h.node_id = g_can_node_id;
	c.h.kbit = g_can_kbit;
	c.h.od_sig = od_signature();
	c.h.od_len = parts_len();
	c.h.od_valid = (uint8_t)((1u << N_PARTS) - 1u);
	if (rq & CO_RQ_RESTORE_COMM) c.h.od_valid &= (uint8_t)~(1u << PART_COMM);
#if N_PARTS > 1
	if (rq & CO_RQ_RESTORE_APP) c.h.od_valid &= (uint8_t)~(1u << PART_APP);
#endif
	const int target = (s_active == 0) ? 1 : 0; /* the old record stays valid until the new one is complete */

	/* SDO writes from the tick must not change the parts between the CRC and the programming. The erase and the
	 * programming stall the CPU anyway, so holding the CANopen interrupts off costs nothing extra */
	HAL_StatusTypeDef st; /* CO_LOCK_OD opens a block */
	CO_LOCK_OD(NULL);
	uint32_t crc = 0xFFFFFFFFu; /* flash_crc32(), streamed */
	for (uint32_t off = 0; off < offsetof(rec_t, crc); off++) {
		crc ^= rec_byte(&c, off);
		for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(crc & 1u));
	}
	c.crc = ~crc;
	st = flash_write_page_from(SLOT_PAGE(target), sizeof(rec_t), rec_get8, &c);
	CO_UNLOCK_OD(NULL);
	if (st != HAL_OK) return st;
	if (!valid(slot(target)) || slot(target)->h.seq != c.h.seq) return HAL_ERROR;
	s_active = target;
	return HAL_OK;
}

void co_store_request(uint8_t rq) {
	__atomic_fetch_or(&s_rq, rq, __ATOMIC_SEQ_CST);
}

uint8_t co_store_take_request(void) {
	return __atomic_exchange_n(&s_rq, 0, __ATOMIC_SEQ_CST);
}
