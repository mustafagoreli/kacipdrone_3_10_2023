/*
 * pwm_input.c
 *
 *  Created on: Sep 21, 2023
 *      Author: CAN
 */
#include "pwm_input.h"
//pwm read için 2 kanalda rising ve falling alıyor global interrupt yapılmalı internal clock kullanılmalı
void init_pwm_read() {
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

void pwm_read_ch1() {
	if (htim2.Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		pwm_ch1.ICValue = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);

		if (pwm_ch1.ICValue != 0) {
			pwm_ch1.dutyCycle = (HAL_TIM_ReadCapturedValue(&htim2,
			TIM_CHANNEL_2) * 20000) / pwm_ch1.ICValue;//max2000 oluyor pwm out ile aynı

			pwm_ch1.frequency = 400000 / pwm_ch1.ICValue;
		}
	}
}
void pwm_read_ch2() {
	if (htim3.Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		pwm_ch2.ICValue = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);

		if (pwm_ch2.ICValue != 0) {
			pwm_ch2.dutyCycle = (HAL_TIM_ReadCapturedValue(&htim3,
			TIM_CHANNEL_2) * 20000) / pwm_ch2.ICValue;

			pwm_ch2.frequency = 400000 / pwm_ch2.ICValue;
		}
	}
}
void pwm_read_ch3() {
	if (htim4.Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		pwm_ch3.ICValue = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);

		if (pwm_ch3.ICValue != 0) {
			pwm_ch3.dutyCycle = (HAL_TIM_ReadCapturedValue(&htim4,
			TIM_CHANNEL_2) * 20000) / pwm_ch3.ICValue;

			pwm_ch3.frequency = 400000 / pwm_ch3.ICValue;
		}
	}

}
void pwm_read_ch4() {
	if (htim9.Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		pwm_ch4.ICValue = HAL_TIM_ReadCapturedValue(&htim9, TIM_CHANNEL_1);

		if (pwm_ch4.ICValue != 0) {
			pwm_ch4.dutyCycle = (HAL_TIM_ReadCapturedValue(&htim9,
			TIM_CHANNEL_2) * 20000) / pwm_ch4.ICValue;

			pwm_ch4.frequency = 400000 / pwm_ch4.ICValue;
		}
	}

}
void pwm_read_ch5() {
	if (htim12.Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		pwm_ch5.ICValue = HAL_TIM_ReadCapturedValue(&htim12, TIM_CHANNEL_1);

		if (pwm_ch5.ICValue != 0) {
			pwm_ch5.dutyCycle = (HAL_TIM_ReadCapturedValue(&htim12,
			TIM_CHANNEL_2) * 20000) / pwm_ch5.ICValue;

			pwm_ch5.frequency = 400000 / pwm_ch5.ICValue;
		}
	}
}
