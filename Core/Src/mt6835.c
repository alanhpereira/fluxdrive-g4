#include "mt6835.h"
#include "spi.h"
#include "temp.h" /* temp_isr_start() */
MT6835_HandleTypeDef hMT6835 = {
	.hspi = &hspi1,
	.nCS_pin = GPIO_PIN_15,
	.nCS_port = GPIOA,
};
HAL_StatusTypeDef MT6835_ReadRegister(MT6835_HandleTypeDef* hMT6835, uint16_t addr, uint8_t* pValue) {
	HAL_StatusTypeDef status;
	if (hMT6835->bursting) return HAL_BUSY;
	HAL_GPIO_WritePin(hMT6835->nCS_port, hMT6835->nCS_pin, GPIO_PIN_RESET);

	hMT6835->tx_data[0] = (uint8_t)(MT6835_OP_READ << 4 | ((addr >> 8) & 0xf));
	hMT6835->tx_data[1] = (uint8_t)((addr) & 0xff);
	status = HAL_SPI_TransmitReceive(hMT6835->hspi, hMT6835->tx_data, hMT6835->rx_data, 3, 10);

	HAL_GPIO_WritePin(hMT6835->nCS_port, hMT6835->nCS_pin, GPIO_PIN_SET);
	if (status != HAL_OK) return status;
	*pValue = hMT6835->rx_data[2];
	return HAL_OK;
}

HAL_StatusTypeDef MT6835_WriteRegister(MT6835_HandleTypeDef* hMT6835, uint16_t addr, uint8_t value) {
	HAL_StatusTypeDef status;
	if (hMT6835->bursting) return HAL_BUSY;
	HAL_GPIO_WritePin(hMT6835->nCS_port, hMT6835->nCS_pin, GPIO_PIN_RESET);

	hMT6835->tx_data[0] = (uint8_t)(MT6835_OP_WRITE << 4 | ((addr >> 8) & 0xf));
	hMT6835->tx_data[1] = (uint8_t)((addr) & 0xff);
	hMT6835->tx_data[2] = value;
	status = HAL_SPI_TransmitReceive(hMT6835->hspi, hMT6835->tx_data, hMT6835->rx_data, 3, 10);

	HAL_GPIO_WritePin(hMT6835->nCS_port, hMT6835->nCS_pin, GPIO_PIN_SET);
	return status;
}

HAL_StatusTypeDef MT6835_SaveEEPROM(MT6835_HandleTypeDef* hMT6835) {
	HAL_StatusTypeDef status;
	if (hMT6835->bursting) return HAL_BUSY;
	HAL_GPIO_WritePin(hMT6835->nCS_port, hMT6835->nCS_pin, GPIO_PIN_RESET);

	hMT6835->tx_data[0] = (uint8_t)(MT6835_OP_READ << 4);
	hMT6835->tx_data[1] = (uint8_t)(0);
	status = HAL_SPI_TransmitReceive(hMT6835->hspi, hMT6835->tx_data, hMT6835->rx_data, 3, 10);

	HAL_GPIO_WritePin(hMT6835->nCS_port, hMT6835->nCS_pin, GPIO_PIN_SET);
	if (status != HAL_OK) return status;
	if (hMT6835->rx_data[2] != 0x55) return HAL_ERROR;
	return HAL_OK;
}

HAL_StatusTypeDef MT6835_SetZero(MT6835_HandleTypeDef* hMT6835) {
	HAL_StatusTypeDef status;
	if (hMT6835->bursting) return HAL_BUSY;
	HAL_GPIO_WritePin(hMT6835->nCS_port, hMT6835->nCS_pin, GPIO_PIN_RESET);

	hMT6835->tx_data[0] = (uint8_t)(MT6835_OP_SET_ZERO << 4);
	hMT6835->tx_data[1] = (uint8_t)(0);
	status = HAL_SPI_TransmitReceive(hMT6835->hspi, hMT6835->tx_data, hMT6835->rx_data, 3, 10);

	HAL_GPIO_WritePin(hMT6835->nCS_port, hMT6835->nCS_pin, GPIO_PIN_SET);
	if (status != HAL_OK) return status;
	if (hMT6835->rx_data[2] != 0x55) return HAL_ERROR;
	return HAL_OK;
}

HAL_StatusTypeDef MT6835_ReadAngle(MT6835_HandleTypeDef* hMT6835) {
	uint8_t bytes[3];
	HAL_StatusTypeDef status;
	if (hMT6835->bursting) return HAL_BUSY;
	status = MT6835_ReadRegister(hMT6835, MT6835_ADDR_ANGLE_H, &bytes[0]);
	if (status != HAL_OK) return status;
	status = MT6835_ReadRegister(hMT6835, MT6835_ADDR_ANGLE_M, &bytes[1]);
	if (status != HAL_OK) return status;
	status = MT6835_ReadRegister(hMT6835, MT6835_ADDR_ANGLE_L, &bytes[2]);
	if (status != HAL_OK) return status;
	hMT6835->angle = (((uint32_t)(bytes[2])) >> 3) | (((uint32_t)(bytes[1])) << 5) | (((uint32_t)(bytes[0])) << 13);
	return HAL_OK;
}

/* ZERO_POS[11:0] = 0: bits 11:4 are all of 0x009, bits 3:0 the high nibble of 0x00A (its low nibble is Z_EDGE and
 * Z_PUL_WID, kept) */
HAL_StatusTypeDef MT6835_ResetZero(MT6835_HandleTypeDef* hMT6835) {
	HAL_StatusTypeDef status;
	status = MT6835_WriteRegister(hMT6835, MT6835_ADDR_ZERO_POS_H, 0x00);
	if (status != HAL_OK) return status;
	uint8_t tmp;
	status = MT6835_ReadRegister(hMT6835, MT6835_ADDR_ZERO_POS_L, &tmp);
	if (status != HAL_OK) return status;
	status = MT6835_WriteRegister(hMT6835, MT6835_ADDR_ZERO_POS_L, tmp & 0x0f);
	return status;
}

/* from here to the end runs every PWM period (TIM1 update IRQ, currentLoop()): keep it at -O2 in a
 * size-optimized build. The blocking register functions above stay small */
#pragma GCC optimize("O2")

static inline uint32_t MT6835_DecodeAngle(const volatile uint8_t* data) {
	return ((uint32_t)data[0] << 13) | ((uint32_t)data[1] << 5) | ((uint32_t)data[2] >> 3);
}

/* a jump of more than half a turn between samples is a wrap through zero (needs < 0.5 turn per
 * 40 us period, i.e. under 750k rpm). turns is written before angle; see MT6835_Position() */
static inline void MT6835_UpdateAngle(MT6835_HandleTypeDef* hMT6835, uint32_t a) {
	if (hMT6835->angle_valid) {
		int32_t d = (int32_t)a - (int32_t)hMT6835->angle;
		if (d < -(1 << 20)) hMT6835->turns++;
		else if (d > (1 << 20)) hMT6835->turns--;
	}
	hMT6835->angle_valid = 1;
	hMT6835->angle = a;
}

/* ------------------------------------------------------------------ per-period burst reads
 * Driven by registers (no HAL calls, no DMA interrupts): the HAL SPI handle stays READY, so the
 * blocking register functions keep working whenever burst mode is stopped. */

/* CRC-8, polynomial x^8 + x^2 + x + 1, init 0, over ANGLE_H, ANGLE_M, ANGLE_L|STATUS.
 * Table in flash (was built in RAM at start: 256 B of SRAM back). Generated with the poly 0x07 loop */
static const uint8_t crc_tab[256] = {
	0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
	0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
	0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
	0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
	0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
	0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
	0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
	0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
	0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
	0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
	0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
	0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
	0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
	0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
	0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
	0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3,
};

static inline uint8_t crc8(const volatile uint8_t* d, int n) {
	uint8_t c = 0;
	while (n--) c = crc_tab[c ^ *d++];
	return c;
}

static void dma_stop(DMA_HandleTypeDef* hdma) {
	hdma->Instance->CCR = 0;
	hdma->DmaBaseAddress->IFCR = DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1FU);
}

static void spi_drain(SPI_TypeDef* spi) {
	while (spi->SR & SPI_SR_FRLVL) (void)*(volatile uint8_t*)&spi->DR; /* also clears OVR */
	(void)spi->SR;
}

/* TIM1 update interrupt, counter bottom: clock MT6835_GROUPS more groups out of the sensor */
static void MT6835_Trigger(MT6835_HandleTypeDef* h) {
	DMA_Channel_TypeDef* rx = h->hspi->hdmarx->Instance;
	DMA_Channel_TypeDef* tx = h->hspi->hdmatx->Instance;
	if (rx->CNDTR != 0 && (rx->CCR & DMA_CCR_EN)) return; /* previous read still running: let it finish */
	h->trig_cnt = (uint16_t)TIM1->CNT;
	rx->CCR = 0;
	tx->CCR = 0;
	rx->CNDTR = MT6835_BUF_LEN;
	tx->CNDTR = MT6835_BUF_LEN;
	rx->CCR = DMA_CCR_MINC | DMA_CCR_PL_1 | DMA_CCR_EN; /* RX first, then TX starts the clock */
	tx->CCR = DMA_CCR_DIR | DMA_CCR_EN;					/* the same zero byte, MINC off */
	h->trig_seq++;
}

int MT6835_Sample(MT6835_HandleTypeDef* h, float* age_s) {
	if (!h->bursting) return 0;
	uint32_t left = h->hspi->hdmarx->Instance->CNDTR;
	if (h->trig_seq == h->used_seq) { /* no read started since last time (TIM1 stopped?) */
		h->stalls++;
		return 0;
	}
	if (left > MT6835_BUF_LEN - 4u) { /* not even one group complete */
		h->late++;
		return 0;
	}
	if (left != 0) h->late++; /* still running at the ADC: use what is there, but count it */
	h->used_seq = h->trig_seq;

	/* time from the read start to the current sample, in TIM1 counts (170 MHz) */
	float to_adc = (float)((int32_t)TIM1->CCR4 - (int32_t)h->trig_cnt) * (1.0f / 170e6f);
	uint32_t start = (MT6835_BUF_LEN - left) / 4u * 4u - 4u; /* newest complete group */
	for (int tries = 0; tries < 2 && start < MT6835_BUF_LEN; tries++, start -= 4u) {
		const volatile uint8_t* g = &h->buf[start];
		const uint32_t word = *(const volatile uint32_t*)g; /* buf is 4-aligned, start a multiple of 4 */
		if (word == 0u || word == 0xFFFFFFFFu) {
			/* MISO stuck low or high: no sensor answering (all zeros even passes the CRC). Never an angle */
			h->crc_errors++;
			continue;
		}
		if (crc8(g, 3) != g[3]) {
			h->crc_errors++;
			if (h->crc_check) {
				/* noise gives isolated failures; every group failing from the start means the CRC rule does
				 * not match this sensor: stop rejecting (diagnostics 0x4002:8 shows it). Once a group has
				 * passed, failures are a fault (lost alignment, wiring) and keep being rejected */
				if (!h->crc_seen && ++h->crc_fail_run >= 25) h->crc_check = 0;
				continue; /* try the group before, 3 us older */
			}
		}
		else {
			h->crc_fail_run = 0;
			h->crc_seen = 1;
		}
		h->samples++;
		h->status = g[2] & 0x07;
		MT6835_UpdateAngle(h, MT6835_DecodeAngle(g));
		/* the sensor latches when the group starts to shift out */
		*age_s = to_adc - (float)start * MT6835_BYTE_S;
		return 1;
	}
	return 0;
}

HAL_StatusTypeDef MT6835_StartBurst(MT6835_HandleTypeDef* hMT6835) {
	if (hMT6835->bursting) return HAL_BUSY;
	SPI_TypeDef* spi = hMT6835->hspi->Instance;
	DMA_HandleTypeDef* hrx = hMT6835->hspi->hdmarx;
	DMA_HandleTypeDef* htx = hMT6835->hspi->hdmatx;
	hMT6835->angle_valid = 0; /* the angle may have moved (or been re-zeroed) while stopped */
	hMT6835->crc_check = 1;
	hMT6835->crc_fail_run = 0;
	hMT6835->crc_seen = 0;
	hMT6835->tx_zero = 0;

	dma_stop(hrx);
	dma_stop(htx);
	spi->CR2 &= ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
	spi->CR1 |= SPI_CR1_SPE; /* SCK at its idle level before CS goes low */
	spi_drain(spi);

	/* burst command by polling, then throw away the 2 bytes received meanwhile: from here on every
	 * 4 bytes clocked are one group, so reads of whole groups stay aligned */
	HAL_GPIO_WritePin(hMT6835->nCS_port, hMT6835->nCS_pin, GPIO_PIN_RESET);
	*(volatile uint8_t*)&spi->DR = (uint8_t)(MT6835_OP_BURST << 4 | ((MT6835_ADDR_ANGLE_H >> 8) & 0xf));
	*(volatile uint8_t*)&spi->DR = (uint8_t)(MT6835_ADDR_ANGLE_H & 0xff);
	uint32_t t0 = HAL_GetTick();
	for (int n = 0; n < 2;) {
		if (spi->SR & SPI_SR_RXNE) {
			(void)*(volatile uint8_t*)&spi->DR;
			n++;
		}
		else if (HAL_GetTick() - t0 > 2) {
			HAL_GPIO_WritePin(hMT6835->nCS_port, hMT6835->nCS_pin, GPIO_PIN_SET);
			return HAL_TIMEOUT;
		}
	}

	/* fixed parts of the two channels; MT6835_Trigger() only reloads the counts */
	hrx->Instance->CPAR = (uint32_t)&spi->DR;
	hrx->Instance->CMAR = (uint32_t)hMT6835->buf;
	htx->Instance->CPAR = (uint32_t)&spi->DR;
	htx->Instance->CMAR = (uint32_t)&hMT6835->tx_zero;
	spi->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN; /* channels still off: nothing moves yet */
	hMT6835->used_seq = hMT6835->trig_seq;

	TIM1->SR = ~TIM_SR_UIF;
	TIM1->DIER |= TIM_DIER_UIE;
	HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
	hMT6835->bursting = 1;
	return HAL_OK;
}

HAL_StatusTypeDef MT6835_StopBurst(MT6835_HandleTypeDef* hMT6835) {
	SPI_TypeDef* spi = hMT6835->hspi->Instance;
	hMT6835->bursting = 0; /* the update interrupt stays on: it also starts the temperature conversions */
	uint32_t t0 = HAL_GetTick(); /* let a read in flight finish */
	while (hMT6835->hspi->hdmarx->Instance->CNDTR != 0 && (hMT6835->hspi->hdmarx->Instance->CCR & DMA_CCR_EN) &&
		   HAL_GetTick() - t0 < 2) {
	}
	dma_stop(hMT6835->hspi->hdmatx);
	dma_stop(hMT6835->hspi->hdmarx);
	spi->CR2 &= ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
	spi_drain(spi);
	HAL_GPIO_WritePin(hMT6835->nCS_port, hMT6835->nCS_pin, GPIO_PIN_SET);
	return HAL_OK;
}

/* Owned here, not by CubeMX: keep the TIM1 update interrupt disabled in the .ioc (NVIC tab),
 * otherwise CubeMX generates a second TIM1_UP_TIM16_IRQHandler in stm32g4xx_it.c */
void TIM1_UP_TIM16_IRQHandler(void) {
	if (TIM1->SR & TIM_SR_UIF) {
		TIM1->SR = ~TIM_SR_UIF;
		/* center-aligned: after the bottom the counter counts up (DIR = 0) */
		if (!(TIM1->CR1 & TIM_CR1_DIR)) {
			if (hMT6835.bursting) MT6835_Trigger(&hMT6835);
			temp_isr_start();
		}
	}
}
