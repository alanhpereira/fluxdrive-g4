/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#include "stm32g4xx_ll_cordic.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HALL3_Pin GPIO_PIN_0
#define HALL3_GPIO_Port GPIOC
#define HALL3_EXTI_IRQn EXTI0_IRQn
#define HALL2_Pin GPIO_PIN_1
#define HALL2_GPIO_Port GPIOC
#define HALL2_EXTI_IRQn EXTI1_IRQn
#define HALL1_Pin GPIO_PIN_2
#define HALL1_GPIO_Port GPIOC
#define HALL1_EXTI_IRQn EXTI2_IRQn
#define MOT_TEMP_Pin GPIO_PIN_3
#define MOT_TEMP_GPIO_Port GPIOC
#define VIN_REF_Pin GPIO_PIN_0
#define VIN_REF_GPIO_Port GPIOA
#define CS1_Pin GPIO_PIN_2
#define CS1_GPIO_Port GPIOA
#define CS_REF_Pin GPIO_PIN_4
#define CS_REF_GPIO_Port GPIOA
#define SCREF_Pin GPIO_PIN_5
#define SCREF_GPIO_Port GPIOA
#define CS2_Pin GPIO_PIN_6
#define CS2_GPIO_Port GPIOA
#define DRV_TEMP_Pin GPIO_PIN_4
#define DRV_TEMP_GPIO_Port GPIOC
#define CS3_Pin GPIO_PIN_1
#define CS3_GPIO_Port GPIOB
#define Wake_Pin GPIO_PIN_7
#define Wake_GPIO_Port GPIOE
#define GLS1_Pin GPIO_PIN_8
#define GLS1_GPIO_Port GPIOE
#define GHS1_Pin GPIO_PIN_9
#define GHS1_GPIO_Port GPIOE
#define GLS2_Pin GPIO_PIN_10
#define GLS2_GPIO_Port GPIOE
#define GHS2_Pin GPIO_PIN_11
#define GHS2_GPIO_Port GPIOE
#define GLS3_Pin GPIO_PIN_12
#define GLS3_GPIO_Port GPIOE
#define GHS3_Pin GPIO_PIN_13
#define GHS3_GPIO_Port GPIOE
#define DRV_RDY_Pin GPIO_PIN_14
#define DRV_RDY_GPIO_Port GPIOE
#define drvnFault_Pin GPIO_PIN_15
#define drvnFault_GPIO_Port GPIOE
#define ENC_I_Pin GPIO_PIN_8
#define ENC_I_GPIO_Port GPIOA
#define SPI_NCS_Pin GPIO_PIN_15
#define SPI_NCS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
