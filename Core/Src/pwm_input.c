/*
 * pwm_input.c
 *
 *  Created on: Sep 21, 2023
 *      Author: CAN
 */
#include "pwm_input.h"

#define MANUAL_MODE 0//otonom manual modda olduğunu gösterecek
#define AUTONOMOUS_MODE 1
#define SAFETY_MODE 2

extern uint8_t drive_mode;

//pwm read için 2 kanalda rising ve falling alıyor global interrupt yapılmalı internal clock kullanılmalı
void pwm_enabled() {
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_2);

	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);

	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_2);

	HAL_TIM_IC_Start_IT(&htim9, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim9, TIM_CHANNEL_2);

	HAL_TIM_IC_Start_IT(&htim12, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim12, TIM_CHANNEL_2);
}

void pwm_disabled() {
	HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Stop(&htim3, TIM_CHANNEL_2);

	HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Stop(&htim2, TIM_CHANNEL_2);

	HAL_TIM_IC_Stop_IT(&htim4, TIM_CHANNEL_1);
	HAL_TIM_IC_Stop(&htim4, TIM_CHANNEL_2);

	HAL_TIM_IC_Stop_IT(&htim9, TIM_CHANNEL_1);
	HAL_TIM_IC_Stop(&htim9, TIM_CHANNEL_2);

	HAL_TIM_IC_Stop_IT(&htim12, TIM_CHANNEL_1);
	HAL_TIM_IC_Stop(&htim12, TIM_CHANNEL_2);
}

void pwm_read_ch1() {
	if (htim2.Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		rc_roll.ICValue = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);

		if (rc_roll.ICValue != 0) {
			rc_roll.dutyCycle = (HAL_TIM_ReadCapturedValue(&htim2,
			TIM_CHANNEL_2) * 15000) / rc_roll.ICValue; //max2000 oluyor pwm out ile aynı

			rc_roll.frequency = 400000 / rc_roll.ICValue;
		}
	}
}
void pwm_read_ch2() {
	if (htim3.Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		rc_yaw.ICValue = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);

		if (rc_yaw.ICValue != 0) {
			rc_yaw.dutyCycle = (HAL_TIM_ReadCapturedValue(&htim3,
			TIM_CHANNEL_2) * 15000) / rc_yaw.ICValue;

			rc_yaw.frequency = 400000 / rc_yaw.ICValue;
		}
	}
}
void pwm_read_ch3() {
	if (htim4.Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		rc_mode.ICValue = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);

		if (rc_mode.ICValue != 0) {
			rc_mode.dutyCycle = (HAL_TIM_ReadCapturedValue(&htim4,
			TIM_CHANNEL_2) * 15000) / rc_mode.ICValue;

			rc_mode.frequency = 400000 / rc_mode.ICValue;
		}

		if (rc_mode.dutyCycle < 1200) { // sadece 2 mod var kumanda da 3 mod var. kill mod falan eklenmek isteniyor ise kod eklenebilir.
			drive_mode = MANUAL_MODE;	// yukarıda iken manuel modda
		} else if (rc_mode.dutyCycle > 1700) {
			drive_mode = SAFETY_MODE;	//BURAYI OTONOM MOD YAZACAĞIZ GÜVENLİK İÇİN ŞU AN SAFETY'DE
		} else if (rc_mode.dutyCycle < 1700 || rc_mode.dutyCycle > 1200) {
			drive_mode = SAFETY_MODE;
		}
	}
}


void pwm_read_ch4() {
if (htim9.Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
	rc_throttle.ICValue = HAL_TIM_ReadCapturedValue(&htim9, TIM_CHANNEL_1);

	if (rc_throttle.ICValue != 0) {
		rc_throttle.dutyCycle = (HAL_TIM_ReadCapturedValue(&htim9,
		TIM_CHANNEL_2) * 15000) / rc_throttle.ICValue;

		rc_throttle.frequency = 400000 / rc_throttle.ICValue;

	}

}
}
void pwm_read_ch5() {
	if (htim12.Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		rc_pitch.ICValue = HAL_TIM_ReadCapturedValue(&htim12, TIM_CHANNEL_1);

		if (rc_pitch.ICValue != 0) {
			rc_pitch.dutyCycle = (HAL_TIM_ReadCapturedValue(&htim12,
			TIM_CHANNEL_2) * 15000) / rc_pitch.ICValue;

			rc_pitch.frequency = 400000 / rc_pitch.ICValue;
		}
	}
}

float MAP(float INvariable, float INmin, float INmax, float OUTmin,
		float OUTmax) {
	return (float) ((((INvariable - INmin) * (OUTmax - OUTmin))
			/ (INmax - INmin)) + OUTmin);
}
