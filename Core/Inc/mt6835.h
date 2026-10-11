#ifndef MT6835_H
#define MT6835_H

#include "stm32g4xx_hal.h"

#define MT6835_OP_READ 0x3
#define MT6835_OP_WRITE 0x6
#define MT6835_OP_PRG_EEPROM 0xC
#define MT6835_OP_SET_ZERO 0x5
#define MT6835_OP_BURST 0xA

#define MT6835_ADDR_USER_ID 0x001
#define MT6835_ADDR_ANGLE_H 0x003
#define MT6835_ADDR_ANGLE_M 0x004
#define MT6835_ADDR_ANGLE_L 0x005
#define MT6835_ADDR_CRC 0x006
#define MT6835_ADDR_ABZ_RES_H 0x007
#define MT6835_ADDR_ABZ_RES_L 0x008
#define MT6835_ADDR_ZERO_POS_H 0x009
#define MT6835_ADDR_ZERO_POS_L 0x00A
#define MT6835_ADDR_UVW 0x00B
#define MT6835_ADDR_PWM 0x00C
#define MT6835_ADDR_ROT 0x00D
#define MT6835_ADDR_CAL 0x00E
#define MT6835_ADDR_BW 0x011
#define MT6835_ADDR_NLC_START 0x013

/* Burst mode stays on: one burst-read command at start, then CS stays low. The sensor shifts out 4-byte
 * groups (ANGLE_H, ANGLE_M, ANGLE_L|STATUS, CRC) whenever the SPI clocks, and the SPI only clocks while
 * the DMA feeds it. Every PWM period the TIM1 update interrupt at the counter bottom starts a DMA read of
 * MT6835_GROUPS groups, sized to end just before the ADC samples the currents (CNT = ARR - 75 on the
 * way up, 19.6 us after the bottom); MT6835_Sample() in currentLoop() takes the last (freshest) group.
 * 6 groups = 24 bytes = 18.1 us at 10.6 MHz, ending ~1 us before the ADC: the last group is ~4 us old.
 * If the 'f' command reports "late", the read did not end in time: lower MT6835_GROUPS */
#define MT6835_GROUPS 6
#define MT6835_BUF_LEN (MT6835_GROUPS * 4)
#define MT6835_BYTE_S (8.0f / 10.625e6f) /* one SPI byte at 170 MHz / 16 */
#define MT6835_ST_OVERSPEED 0x01		 /* STATUS[2:0] in the low bits of ANGLE_L */
#define MT6835_ST_WEAK_FIELD 0x02
#define MT6835_ST_UNDERVOLT 0x04

typedef struct {
	SPI_HandleTypeDef* hspi;
	GPIO_TypeDef* nCS_port;
	uint16_t nCS_pin;
	uint8_t bursting; /* burst mode on: the blocking register functions refuse */
	uint8_t tx_data[3];
	uint8_t rx_data[3];
	volatile uint32_t angle;
	volatile int32_t turns; /* full revolutions counted since sampling started, raw encoder direction */
	uint8_t angle_valid;	/* 0 until the first sample, so start-up does not count a fake wrap */

	/* per-period read */
	volatile uint32_t trig_seq; /* incremented by every read started */
	uint32_t used_seq;			/* trig_seq of the last read collected */
	volatile uint16_t trig_cnt; /* TIM1 CNT when the read started (counting up from the bottom) */
	volatile uint8_t status;	/* STATUS bits of the last good group */
	uint8_t crc_check;			/* reject groups with a bad CRC; cleared automatically if every group fails */
	uint8_t crc_fail_run;
	volatile uint32_t samples, crc_errors, late, stalls; /* late: read not finished at the ADC; stalls: no read started */
	uint8_t tx_zero;
	__ALIGNED(4)
	volatile uint8_t buf[MT6835_BUF_LEN];
} MT6835_HandleTypeDef;
extern MT6835_HandleTypeDef hMT6835;
HAL_StatusTypeDef MT6835_ReadRegister(MT6835_HandleTypeDef* hMT6835, uint16_t addr, uint8_t* pValue);
HAL_StatusTypeDef MT6835_WriteRegister(MT6835_HandleTypeDef* hMT6835, uint16_t addr, uint8_t value);
HAL_StatusTypeDef MT6835_SaveEEPROM(MT6835_HandleTypeDef* hMT6835);
HAL_StatusTypeDef MT6835_SetZero(MT6835_HandleTypeDef* hMT6835);
HAL_StatusTypeDef MT6835_ReadAngle(MT6835_HandleTypeDef* hMT6835);
HAL_StatusTypeDef MT6835_StartBurst(MT6835_HandleTypeDef* hMT6835); /* burst mode + per-period reads */
HAL_StatusTypeDef MT6835_StopBurst(MT6835_HandleTypeDef* hMT6835);
HAL_StatusTypeDef MT6835_ResetZero(MT6835_HandleTypeDef* hMT6835);

/* call at the start of currentLoop(): takes the newest complete group of this period's read (normally
 * the last) with a good CRC, updates angle/turns and stores in *age_s how long before the current sample
 * the sensor sampled it. Returns 1 if a fresh angle was taken, 0 if the old one is kept */
int MT6835_Sample(MT6835_HandleTypeDef* hMT6835, float* age_s);

/* multi-turn position in counts (2^21 per turn), raw encoder direction.
 * Safe from thread context: retries if currentLoop() updated in between.
 * Inside currentLoop() (or an ISR of the same priority), turns/angle can be read directly. */
static inline int64_t MT6835_Position(const MT6835_HandleTypeDef* h) {
	int32_t t;
	uint32_t a;
	do {
		t = h->turns;
		a = h->angle;
	} while (t != h->turns);
	return (int64_t)t * 2097152 + (int64_t)a;
}

#endif