#include "co_store.h"

#include <stddef.h>
#include <string.h>

#include "301/CO_driver.h" /* CO_LOCK_OD */
#include "CANopen.h"
#include "OD.h"
#include "co_app.h"
#include "params.h" /* flash_crc32(), flash_write_page() */

/* pages 60 and 61, between the program (FLASH LENGTH 120K) and the motor params (62-63) */
#define SLOT_PAGE(i) (60u + (uint32_t)(i))
#define SLOT_ADDR(i) (FLASH_BASE + SLOT_PAGE(i) * FLASH_PAGE_SIZE)

#define REC_MAGIC 0x434F4D31u /* 'COM1' */
#define REC_VERSION 1u
#define OD_MAX 1024u /* room for OD_PERSIST_COMM; fixed, so an OD re-export keeps the node ID and bit rate */

typedef struct {
	uint32_t magic;
	uint16_t version;
	uint16_t size;
	uint32_t seq;
	uint8_t node_id;
	uint8_t od_valid; /* 0: no OD copy (never saved, or 0x1011 restore): the exported defaults apply */
	uint16_t kbit;
	uint32_t od_sig; /* od_signature() when saved */
	uint16_t od_len;
	uint16_t reserved;
	uint8_t od[OD_MAX];
	uint32_t reserved2;
	uint32_t crc; /* CRC32 of everything above; keep last */
} rec_t;

_Static_assert(sizeof(rec_t) % 8 == 0, "flash programs 64-bit words");
_Static_assert(sizeof(rec_t) <= FLASH_PAGE_SIZE, "one record per page");
_Static_assert(sizeof(OD_PERSIST_COMM) <= OD_MAX, "OD_PERSIST_COMM grew: raise OD_MAX (and REC_VERSION)");

static int s_active = -1; /* slot with the newest valid record, -1 = none */
static volatile uint8_t s_rq;

static const rec_t* slot(int i) {
	return (const rec_t*)SLOT_ADDR(i);
}

static int valid(const rec_t* r) {
	if (r->magic != REC_MAGIC || r->version != REC_VERSION || r->size != sizeof(rec_t)) return 0;
	return r->crc == flash_crc32(r, sizeof(rec_t) - sizeof(uint32_t));
}

/* FNV-1a over the OD table of contents (index, sub-entry count, object type) and the size of OD_PERSIST_COMM:
 * changes whenever a re-export could move fields inside OD_PERSIST_COMM */
static uint32_t od_signature(void) {
	uint32_t h = 2166136261u;
	const uint32_t n = sizeof(OD_PERSIST_COMM);
	for (int b = 0; b < 32; b += 8) h = (h ^ ((n >> b) & 0xFFu)) * 16777619u;
	for (uint16_t i = 0; i < OD->size; i++) {
		const OD_entry_t* e = &OD->list[i];
		const uint8_t v[4] = {(uint8_t)e->index, (uint8_t)(e->index >> 8), e->subEntriesCount, e->odObjectType};
		for (int k = 0; k < 4; k++) h = (h ^ v[k]) * 16777619u;
	}
	return h;
}

static int od_usable(const rec_t* r) {
	return r->od_valid && r->od_len == sizeof(OD_PERSIST_COMM) && r->od_sig == od_signature();
}

/* the exported defaults: OD_PERSIST_COMM is initialised data, so its initial image is in flash at _sidata */
static const uint8_t* od_defaults(void) {
	extern const uint8_t _sidata[], _sdata[], _edata[]; /* linker symbols: arrays, so no object size is assumed */
	const uint8_t* p = (const uint8_t*)&OD_PERSIST_COMM;
	if (p < _sdata || p + sizeof(OD_PERSIST_COMM) > _edata) return NULL;
	return _sidata + (p - _sdata);
}

void co_store_load(void) {
	const int va = valid(slot(0)), vb = valid(slot(1));
	if (va && vb)
		s_active = ((int32_t)(slot(0)->seq - slot(1)->seq) >= 0) ? 0 : 1; /* wrap-safe */
	else
		s_active = va ? 0 : vb ? 1 : -1;
	if (s_active < 0) return; /* the defaults in co_app.c stay */
	const rec_t* r = slot(s_active);
	if (r->node_id >= 1 && r->node_id <= 127) g_can_node_id = r->node_id;
	if (co_can_prescaler(r->kbit)) g_can_kbit = r->kbit;
}

void co_store_apply_od(void) {
	if (s_active >= 0 && od_usable(slot(s_active))) {
		memcpy(&OD_PERSIST_COMM, slot(s_active)->od, sizeof(OD_PERSIST_COMM));
		return;
	}
	const uint8_t* d = od_defaults();
	if (d) memcpy(&OD_PERSIST_COMM, d, sizeof(OD_PERSIST_COMM));
}

int co_store_dirty(void) {
	if (s_active < 0) return 1;
	const rec_t* r = slot(s_active);
	if (r->node_id != g_can_node_id || r->kbit != g_can_kbit) return 1;
	const uint8_t* ref = od_usable(r) ? r->od : od_defaults();
	if (!ref) return 0;
	/* the identity's revision and serial are filled in at run time: compare everything else */
	uint8_t tmp[sizeof(OD_PERSIST_COMM)];
	memcpy(tmp, ref, sizeof tmp);
	const size_t id = offsetof(OD_PERSIST_COMM_t, x1018_identity);
	memcpy(tmp + id, (const uint8_t*)&OD_PERSIST_COMM + id, sizeof(OD_PERSIST_COMM.x1018_identity));
	return memcmp(tmp, &OD_PERSIST_COMM, sizeof tmp) != 0;
}

HAL_StatusTypeDef co_store_save(int restore_od) {
	static __ALIGNED(8) rec_t rec; /* 1 KB: not on the stack */
	memset(&rec, 0xFF, sizeof rec);
	rec.magic = REC_MAGIC;
	rec.version = REC_VERSION;
	rec.size = sizeof(rec_t);
	rec.seq = (s_active >= 0) ? slot(s_active)->seq + 1u : 1u;
	rec.node_id = g_can_node_id;
	rec.kbit = g_can_kbit;
	rec.od_valid = restore_od ? 0 : 1;
	rec.od_sig = od_signature();
	rec.od_len = sizeof(OD_PERSIST_COMM);
	rec.reserved = 0;
	rec.reserved2 = 0;
	if (!restore_od) {
		CO_LOCK_OD(NULL); /* SDO writes from the tick must not tear the copy */
		memcpy(rec.od, &OD_PERSIST_COMM, sizeof(OD_PERSIST_COMM));
		CO_UNLOCK_OD(NULL);
	}
	rec.crc = flash_crc32(&rec, sizeof rec - sizeof(uint32_t));

	const int target = (s_active == 0) ? 1 : 0; /* the old record stays valid until the new one is complete */
	const HAL_StatusTypeDef st = flash_write_page(SLOT_PAGE(target), &rec, sizeof rec);
	if (st != HAL_OK) return st;
	if (!valid(slot(target)) || memcmp(slot(target), &rec, sizeof rec) != 0) return HAL_ERROR;
	s_active = target;
	return HAL_OK;
}

void co_store_request(uint8_t rq) {
	__atomic_fetch_or(&s_rq, rq, __ATOMIC_SEQ_CST);
}

uint8_t co_store_take_request(void) {
	return __atomic_exchange_n(&s_rq, 0, __ATOMIC_SEQ_CST);
}
