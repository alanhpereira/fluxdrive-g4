#ifndef CURSNSCTRL_H
#define CURSNSCTRL_H

#include "stm32g4xx_hal.h"

extern volatile int16_t cur_raw[3];
extern volatile uint16_t vin_raw;

void disableCS();
HAL_StatusTypeDef initCS();
#endif