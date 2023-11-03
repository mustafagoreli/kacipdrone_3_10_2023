/*
 * pwm_input.h
 *
 *  Created on: Sep 21, 2023
 *      Author: Mustafa
 */

#ifndef INC_PWM_INPUT_H_
#define INC_PWM_INPUT_H_

#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "main.h"

typedef struct {
	uint32_t ICValue;
	uint32_t frequency;
	float dutyCycle;
	bool isRCConnected;
	float rcToAngle;
} pwm_input_t;

extern pwm_input_t rc_roll;
extern pwm_input_t rc_yaw;
extern pwm_input_t rc_mode;
extern pwm_input_t rc_throttle;
extern pwm_input_t rc_pitch;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim12;

void pwm_enabled();
void pwm_disabled();

void pwm_read_ch1();
void pwm_read_ch2();
void pwm_read_ch3();
void pwm_read_ch4();
void pwm_read_ch5();

float MAP(float au32_IN, float au32_INmin, float au32_INmax, float au32_OUTmin, float au32_OUTmax);

#endif /* INC_PWM_INPUT_H_ */
