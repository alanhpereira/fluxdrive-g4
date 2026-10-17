#ifndef OBJDICT_H
#define OBJDICT_H

/* Object table: every value the config protocol can read or write, addressed index:subindex like a
 * CANopen object dictionary. Native objects live in 0x4000-0x4FFF, SI units as stored in g_params.
 * Keep in sync with software/objects.py (same order: the table hash covers index, sub, type, flags) */

#include <stdint.h>

enum { T_U8 = 1,
	   T_I8,
	   T_U16,
	   T_U32,
	   T_I32,
	   T_F32,
	   T_I64,
	   T_I16 };

#define OF_RO 0x01	   /* read only */
#define OF_STORED 0x02 /* saved by MSG_SAVE: g_params, or the CAN comm record (0x4040) */
#define OF_PI 0x04	   /* pi_apply() after a write */
#define OF_NOTRUN 0x08 /* refused unless g_mode == CTRL_OFF */

typedef struct {
	uint16_t index;
	uint8_t sub;
	uint8_t type;
	uint8_t flags;
	void* p;
	float min, max; /* writes outside are refused (SDO_RANGE); RO objects ignore them */
} obj_t;

void objdict_init(void);
uint32_t objdict_hash(void);
uint32_t objdict_build_id(void);

/* value written to out (type-sized, little-endian); returns 0 or an SDO abort code, *len = bytes */
uint32_t od_read(uint16_t index, uint8_t sub, uint8_t* out, uint8_t* len);
/* same, but never blocks (0x4001:6, the gate-driver status over I2C, returns the last value): for the CANopen tick */
uint32_t od_read_fast(uint16_t index, uint8_t sub, uint8_t* out, uint8_t* len);
uint32_t od_write(uint16_t index, uint8_t sub, const uint8_t* v, uint8_t len);
/* up to max entries from first, 5 bytes each (index, sub, type, flags); returns the count */
int od_list(uint16_t first, uint8_t* out, int max);

#endif
