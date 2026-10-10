#ifndef CURSNSCTRL_H
#define CURSNSCTRL_H

#include "stm32g4xx_hal.h"

#define VDDA 3.3f
#define CS_SHUNT 0.002f
#define CS_GAIN 16.0f
#define VIN_RL 10.0f
#define VIN_RH 270.0f

extern volatile int16_t cur_raw[3];
extern volatile uint16_t vin_raw;
extern volatile float g_cur[3];
extern volatile float g_vin;

void disableCS();
HAL_StatusTypeDef initCS();
#endif