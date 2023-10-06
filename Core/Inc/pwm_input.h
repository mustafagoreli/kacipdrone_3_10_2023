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

typedef struct {
	uint32_t ICValue;
	uint32_t frequency;
	float dutyCycle;
	bool isRCConnected;
} pwm_input_t;

extern pwm_input_t pwm_ch1;
extern pwm_input_t pwm_ch2;
extern pwm_input_t pwm_ch3;
extern pwm_input_t pwm_ch4;
extern pwm_input_t pwm_ch5;

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


#endif /* INC_PWM_INPUT_H_ */
