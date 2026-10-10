#ifndef MOTORCTRL_H
#define MOTORCTRL_H

#include "stm32g4xx_hal.h"
#include "bridgeCtrl.h"
#include "curSnsCtrl.h"

void disableMotor();
HAL_StatusTypeDef initMotor();
void currentLoop();
#endif