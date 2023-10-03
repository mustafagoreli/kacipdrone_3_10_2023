/*
 * power_module.h
 *
 *  Created on: Sep 29, 2023
 *      Author: CAN
 */

#ifndef INC_POWER_MODULE_H_
#define INC_POWER_MODULE_H_

#include "stdbool.h"
#include "stm32f4xx_hal.h"

extern ADC_HandleTypeDef hadc1;

typedef struct {
	float voltage;
	bool isBatteryLow;	//otonom eve donus
	bool isBatteryDead;	//yavasca asagi in
} battery_t;

void BatteryInit();
float getBatteryVoltage();

#endif /* INC_POWER_MODULE_H_ */
