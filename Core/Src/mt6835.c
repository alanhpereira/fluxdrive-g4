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

HAL_StatusTypeDef MT6835_StartBurst(MT6835_HandleTypeDef* hMT6835) {
	HAL_StatusTypeDef status;
	if (hMT6835->bursting) return HAL_BUSY;

	HAL_GPIO_WritePin(hMT6835->nCS_port, hMT6835->nCS_pin, GPIO_PIN_RESET);

	hMT6835->tx_data[0] = (uint8_t)(MT6835_OP_BURST << 4 | ((MT6835_ADDR_ANGLE_H >> 8) & 0xf));
	hMT6835->tx_data[1] = (uint8_t)((MT6835_ADDR_ANGLE_H) & 0xff);

	status = HAL_SPI_Transmit(hMT6835->hspi, hMT6835->tx_data, 2, 10);

	if (status != HAL_OK) {
		HAL_GPIO_WritePin(hMT6835->nCS_port, hMT6835->nCS_pin, GPIO_PIN_SET);
		return status;
	}
	status = HAL_SPI_Receive_DMA(hMT6835->hspi, hMT6835->dma_buffer, BURST_RED * 8);
	if (status != HAL_OK) {
		HAL_GPIO_WritePin(hMT6835->nCS_port, hMT6835->nCS_pin, GPIO_PIN_SET);
		return status;
	}
	hMT6835->bursting = 1;
	return HAL_OK;
}

HAL_StatusTypeDef MT6835_StopBurst(MT6835_HandleTypeDef* hMT6835) {
	HAL_StatusTypeDef status;
	status = HAL_SPI_DMAStop(hMT6835->hspi);
	hMT6835->bursting = 0;
	HAL_GPIO_WritePin(hMT6835->nCS_port, hMT6835->nCS_pin, GPIO_PIN_SET);
	return status;
}

static inline uint32_t MT6835_DecodeAngle(const uint8_t* data) {
	return ((uint32_t)data[0] << 13) | ((uint32_t)data[1] << 5) | ((uint32_t)data[2] >> 3);
}

void MT6835_RxHalfCpltCallback(MT6835_HandleTypeDef* hMT6835) {
	hMT6835->angle = MT6835_DecodeAngle(
		&hMT6835->dma_buffer[BURST_RED * 4 - 4]);
}

void MT6835_RxCpltCallback(MT6835_HandleTypeDef* hMT6835) {
	hMT6835->angle = MT6835_DecodeAngle(
		&hMT6835->dma_buffer[BURST_RED * 8 - 4]);
}