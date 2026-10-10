#include "motorCtrl.h"

#include "i2c.h"

void disableMotor() {
	disableCS();
	disableBridge();
}

HAL_StatusTypeDef initMotor() {
	HAL_StatusTypeDef status;
	status = initCS();
	if (status != HAL_OK) {
		disableMotor();
		return status;
	}
	status = initBridge();
	if (status != HAL_OK) {
		disableMotor();
		return status;
	}
	return HAL_OK;
}

void currentLoop() {}