#ifndef BRIDGECTRL_H
#define BRIDGECTRL_H

#include "stm32g4xx_hal.h"
#include "tim.h"

#define ZERO_TIME 100
#define SCREF_MV 500

void disableBridge();
HAL_StatusTypeDef initBridge();
static inline void setDuty(uint32_t a, uint32_t b, uint32_t c) {
	TIM1->CCR1 = a;
	TIM1->CCR2 = b;
	TIM1->CCR3 = c;
}
void currentLoop();
#endif