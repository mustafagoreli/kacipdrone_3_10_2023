/*
 * power_module.c
 *
 *  Created on: Sep 29, 2023
 *      Author: CAN
 */
//mainde while hemen öncesinde çağır

#include "power_module.h"

void BatteryInit() {
	HAL_ADC_Start(&hadc1);
}

float getBatteryVoltage() {
	uint16_t adcValue;
	float adcVoltage;
	HAL_ADC_Start(&hadc1); // start the adc

	HAL_ADC_PollForConversion(&hadc1, 100); // poll for conversion

	adcValue = HAL_ADC_GetValue(&hadc1); // get the adc value

	HAL_ADC_Stop(&hadc1); // stop adc

	adcVoltage = 3.3 * adcValue / 4095;

	return adcVoltage * 10.818;
}

