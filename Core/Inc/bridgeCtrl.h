#ifndef BRIDGECTRL_H
#define BRIDGECTRL_H

#include "stm32g4xx_hal.h"
#include "tim.h"

#define ZERO_TIME 100
#define SCREF_MV 500
#define FAULT_CLEAR_MIN_TIME 100

extern int32_t g_pwm_res;
extern int32_t g_max_duty_cycle;
extern int32_t g_half_duty_cycle;
extern uint32_t g_lastFaultCleared;

void disableBridge();
HAL_StatusTypeDef initBridge();
static inline void setDuty(uint32_t a, uint32_t b, uint32_t c) {
	TIM1->CCR1 = a;
	TIM1->CCR2 = b;
	TIM1->CCR3 = c;
}
static inline void setSamplePoint(uint32_t p) {
	TIM1->CCR4 = p;
}
/* STSPIN32G4 gate driver over I2C3 */
#define DRV_I2C_ADDR (0x47 << 1)
#define DRV_REG_CLEAR 0x09	/* write 0xFF: clear the latched status bits */
#define DRV_REG_STATUS 0x80 /* bit0 VCC_UVLO, bit1 THSD, bit2 VDS_P, bit3 RESET, bit7 LOCK */
#define DRV_ST_FAULTS 0x0F
#define DRV_ST_RESET 0x08	/* STATUS bit3: the registers are back at their power-up defaults */
#define DRV_REG_POWMNG 0x01 /* protected; bit6 REG3V3_DIS, bit5 VCC_DIS, bit4 STBY_REG_EN, bits 1:0 VCC_VAL */
#define DRV_REG_LOCK 0x0B	/* protected registers are writable while LOCK (3:0) == ~NLOCK (7:4) */
#define DRV_LOCK_OPEN 0xF0
#define DRV_LOCK_CLOSED 0x00
#define DRV_VCC_8V 0
#define DRV_VCC_10V 1
#define DRV_VCC_12V 2
#define DRV_VCC_15V 3
/* gate-drive supply (VCC buck), 8 V is the power-up default. VCC also feeds the 5 V LDO U5 (0.5 W): it can supply
 * ~95 mA at 10 V, ~60 mA at 12 V */
#define DRV_VCC_SET DRV_VCC_10V
#define FAULT_RETRY_MS 20

/* checkAndClearBridgeFault() results besides 0 / 1 / -(status fault bits) */
#define BRIDGE_ERR_NFAULT (-16) /* status clean but nFAULT still low */
#define BRIDGE_ERR_I2C (-32)	/* driver did not answer */
#define BRIDGE_ERR_WAIT (-64)	/* still faulted, next attempt after FAULT_RETRY_MS */
#define BRIDGE_ERR_TRIP (-128)	/* switched off by a trip (g_trip_moe): not re-armed until STOP resets it */
#define BRIDGE_ERR_SPIN (-256)	/* ready, but the motor may still turn too fast for the zero vector (rearm_safe()) */

/* last resort of the trips in currentLoop(): outputs to their idle (off) state, the motor coasts.
 * checkAndClearBridgeFault() leaves MOE off while g_trip_moe is set */
extern volatile uint8_t g_trip_moe;
static inline void bridge_trip_off(void) {
	TIM1->BDTR &= ~TIM_BDTR_MOE;
	g_trip_moe = 1;
}

int32_t checkAndClearBridgeFault();
/* why the bridge went off, for the STOPPED event: driver status fault bits (DRV_ST_FAULTS, read now or cleared
 * by checkAndClearBridgeFault() since the last call) plus the BFI_* bits. Main only (I2C) */
#define BFI_NFAULT 0x10 /* nFAULT low now */
#define BFI_BRK 0x20	/* TIM1 break flag */
#define BFI_BRK2 0x40	/* TIM1 break 2 flag */
#define BFI_I2C 0x80	/* status read failed */
uint32_t bridge_fault_info(void);
HAL_StatusTypeDef readBridgeStatus(uint8_t* status);
#endif