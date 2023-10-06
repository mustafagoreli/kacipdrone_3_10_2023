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

//BURALAR degisecek yeni katsayılar belirlenecek remaining voltage adc den alınan değere koyulacak
//#define BATTERY_REMAINING_VOLTAGE 1	//parametre olarak veya global değişkenle veririz
#define BATTERY_START_VOLTAGE 16.8
#define BATTERY_COMPENSATION_COEFFICIENT 1
#define MAX_PWM_OUTPUT 1900
#define MIN_PWM_OUTPUT 1100


void init_esc();

//pwm1Val şeklinde yazılmasına gerek yok ama scope'tan dolayı fark etmez
void drive_motor_1(uint8_t pwm1Val);
void drive_motor_2(uint8_t pwm2Val);
void drive_motor_3(uint8_t pwm3Val);
void drive_motor_4(uint8_t pwm4Val);

void set_pwm(float roll_pid, float pitch_pid, float yaw_pid, uint16_t throttle, battery_t battery);

#endif /* INC_ESC_H_ */
