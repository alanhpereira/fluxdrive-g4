#ifndef MOTORCTRL_H
#define MOTORCTRL_H

#include "stm32g4xx_hal.h"
#include "bridgeCtrl.h"
#include "curSnsCtrl.h"

void disableMotor();
HAL_StatusTypeDef initMotor();
void currentLoop();
int SVPWM(float alpha, float beta);

extern volatile uint32_t g_loop_cyc, g_loop_cyc_max; /* currentLoop() run time in CPU cycles (170 per us) */
#endif