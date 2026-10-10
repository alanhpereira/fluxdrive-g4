/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "dac.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "opamp.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mt6835.h"
#include "motorCtrl.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static MT6835_HandleTypeDef hMT6835 = {
	.hspi = &hspi1,
	.nCS_pin = GPIO_PIN_15,
	.nCS_port = GPIOA,
};
static uint8_t ready = 0;
static uint8_t st = 0;
static uint8_t nFaultDig = 0;
uint8_t i2c_err;
uint8_t i2c_stt;
uint32_t lastFaultCleared = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	uint8_t dummy;
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_FDCAN1_Init();
	MX_I2C3_Init();
	MX_OPAMP1_Init();
	MX_OPAMP2_Init();
	MX_OPAMP3_Init();
	MX_TIM1_Init();
	MX_USB_PCD_Init();
	MX_ADC2_Init();
	MX_DAC1_Init();
	MX_SPI1_Init();
	MX_TIM4_Init();
	MX_USART1_UART_Init();
	MX_CORDIC_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim2);
	// set cs mid reference to vcc/2
	HAL_Delay(5);
	MT6835_StartBurst(&hMT6835);
	initMotor();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		ready = HAL_GPIO_ReadPin(DRV_RDY_GPIO_Port, DRV_RDY_Pin);
		nFaultDig = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15);
		if (HAL_GetTick() > lastFaultCleared + 100) {
			if ((HAL_GetTick() / 500) % 4 == 0) {
				setDuty(3200, 3200, 0);
			}
			else {
				setDuty(1600, 1600, 1600);
			}
		}
		HAL_StatusTypeDef rc = HAL_I2C_Mem_Read(&hi2c3, 0x47 << 1, 0x80, I2C_MEMADD_SIZE_8BIT, &st, 1, 10);

		uint32_t bdtr = TIM1->BDTR;
		uint32_t sr = TIM1->SR;

		int moe_off = (bdtr & TIM_BDTR_MOE) == 0;
		int brk_set = (sr & (TIM_SR_BIF | TIM_SR_B2IF)) != 0;

		if (rc == HAL_OK && (moe_off || brk_set)) {
			if (st & 0x03) {
				// THSD or VCC UVLO still active: wait, they clear themselves
			}
			else {
				if (st & 0x0C) {  // latched RESET / VDS_P: clear the driver latch
					uint8_t v = 0xFF;
					HAL_I2C_Mem_Write(&hi2c3, 0x47 << 1, 0x09, I2C_MEMADD_SIZE_8BIT, &v, 1, 10);
				}
				if (GPIOE->IDR & GPIO_PIN_15) {	 // nFAULT released → break input inactive
					setDuty(0, 0, 0);
					TIM1->EGR = TIM_EGR_UG;	 // load zero duty now (also resets the counter)
					__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK | TIM_FLAG_BREAK2);
					__HAL_TIM_MOE_ENABLE(&htim1);
					lastFaultCleared = HAL_GetTick();
				}
			}
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV8;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef* hspi) {
	if (hspi == hMT6835.hspi) {
		MT6835_RxHalfCpltCallback(&hMT6835);
	}
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi) {
	if (hspi == hMT6835.hspi) {
		MT6835_RxCpltCallback(&hMT6835);
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
