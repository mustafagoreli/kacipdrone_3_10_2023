/*
 * esc.h
 *
 *  Created on: Sep 28, 2023
 *      Author: CAN
 */

#ifndef INC_ESC_H_
#define INC_ESC_H_

#include "stm32f4xx_hal.h"
#include "tim.h"
#include "power_module.h"

extern TIM_HandleTypeDef htim1;

typedef struct{
	float pwm_esc1;
	float pwm_esc2;
	float pwm_esc3;
	float pwm_esc4;
}esc_t;


//BURALAR degisecek yeni katsayılar belirlenecek remaining voltage adc den alınan değere koyulacak
//#define BATTERY_REMAINING_VOLTAGE 1	//parametre olarak veya global değişkenle veririz
#define BATTERY_START_VOLTAGE 16.8
#define BATTERY_COMPENSATION_COEFFICIENT 12.7
#define MAX_PWM_OUTPUT 1950
#define MIN_PWM_OUTPUT 1100

void init_esc();

//pwm1Val şeklinde yazılmasına gerek yok ama scope'tan dolayı fark etmez
void drive_motor_1(int pwm1Val);
void drive_motor_2(int pwm2Val);
void drive_motor_3(int pwm3Val);
void drive_motor_4(int pwm4Val);

void set_pwm(float roll_pid, float pitch_pid, float yaw_pid, uint16_t throttle,
		battery_t battery, esc_t esc);

#endif /* INC_ESC_H_ */
