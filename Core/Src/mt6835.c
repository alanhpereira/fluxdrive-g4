#include "mt6835.h"
#include "spi.h"
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

HAL_StatusTypeDef MT6835_ResetZero(MT6835_HandleTypeDef* hMT6835) {
	HAL_StatusTypeDef status;
	status = MT6835_WriteRegister(hMT6835, MT6835_ADDR_ZERO_POS_H, 0x00);
	if (status != HAL_OK) return status;
	uint8_t tmp;
	status = MT6835_ReadRegister(hMT6835, MT6835_ADDR_ZERO_POS_H, &tmp);
	if (status != HAL_OK) return status;
	status = MT6835_WriteRegister(hMT6835, MT6835_ADDR_ZERO_POS_H, tmp & 0x0f);
	return status;
}
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

/* CRC-8, polynomial x^8 + x^2 + x + 1, init 0, over ANGLE_H, ANGLE_M, ANGLE_L|STATUS */
static uint8_t crc_tab[256];

static void crc_init(void) {
	for (int i = 0; i < 256; i++) {
		uint8_t c = (uint8_t)i;
		for (int k = 0; k < 8; k++) c = (uint8_t)((c & 0x80) ? (c << 1) ^ 0x07 : c << 1);
		crc_tab[i] = c;
	}
}

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
		if (crc8(g, 3) != g[3]) {
			h->crc_errors++;
			if (h->crc_check) {
				/* noise gives isolated failures; every group failing means the CRC rule does not match
				 * this sensor: stop rejecting (the 'f' command shows it) */
				if (++h->crc_fail_run >= 25) h->crc_check = 0;
				continue; /* try the group before, 3 us older */
			}
		}
		else {
			h->crc_fail_run = 0;
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
	static uint8_t crc_ready;
	if (hMT6835->bursting) return HAL_BUSY;
	if (!crc_ready) {
		crc_init();
		crc_ready = 1;
	}
	SPI_TypeDef* spi = hMT6835->hspi->Instance;
	DMA_HandleTypeDef* hrx = hMT6835->hspi->hdmarx;
	DMA_HandleTypeDef* htx = hMT6835->hspi->hdmatx;
	hMT6835->angle_valid = 0; /* the angle may have moved (or been re-zeroed) while stopped */
	hMT6835->crc_check = 1;
	hMT6835->crc_fail_run = 0;
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
	TIM1->DIER &= ~TIM_DIER_UIE;
	hMT6835->bursting = 0;
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
		if (!(TIM1->CR1 & TIM_CR1_DIR) && hMT6835.bursting) MT6835_Trigger(&hMT6835);
	}
}
