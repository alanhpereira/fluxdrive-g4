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

#define BURST_RED 6

typedef struct {
	SPI_HandleTypeDef* hspi;
	GPIO_TypeDef* nCS_port;
	uint16_t nCS_pin;
	uint8_t bursting;
	uint8_t tx_data[3];
	uint8_t rx_data[3];
	volatile uint32_t angle;
	volatile int32_t turns; /* full revolutions counted from burst start, raw encoder direction */
	uint8_t angle_valid;	/* 0 until the first burst sample, so start-up does not count a fake wrap */
	__ALIGNED(4)
	volatile uint8_t dma_buffer[BURST_RED * 8];

} MT6835_HandleTypeDef;
extern MT6835_HandleTypeDef hMT6835;
HAL_StatusTypeDef MT6835_ReadRegister(MT6835_HandleTypeDef* hMT6835, uint16_t addr, uint8_t* pValue);
HAL_StatusTypeDef MT6835_WriteRegister(MT6835_HandleTypeDef* hMT6835, uint16_t addr, uint8_t value);
HAL_StatusTypeDef MT6835_SaveEEPROM(MT6835_HandleTypeDef* hMT6835);
HAL_StatusTypeDef MT6835_SetZero(MT6835_HandleTypeDef* hMT6835);
HAL_StatusTypeDef MT6835_ReadAngle(MT6835_HandleTypeDef* hMT6835);
HAL_StatusTypeDef MT6835_StartBurst(MT6835_HandleTypeDef* hMT6835);
HAL_StatusTypeDef MT6835_StopBurst(MT6835_HandleTypeDef* hMT6835);
HAL_StatusTypeDef MT6835_ResetZero(MT6835_HandleTypeDef* hMT6835);
void MT6835_RxHalfCpltCallback(MT6835_HandleTypeDef* hMT6835);
void MT6835_RxCpltCallback(MT6835_HandleTypeDef* hMT6835);

/* multi-turn position in counts (2^21 per turn), raw encoder direction.
 * Safe from thread context: retries if the DMA callback updated in between.
 * Inside an ISR of the same priority as the SPI DMA, turns/angle can be read directly. */
static inline int64_t MT6835_Position(const MT6835_HandleTypeDef* h) {
	int32_t t;
	uint32_t a;
	do {
		t = h->turns;
		a = h->angle;
	} while (t != h->turns);
	return ((int64_t)t << 21) + (int64_t)a;
}

#endif