/*
 * esc.c
 *
 *  Created on: Sep 28, 2023
 *      Author: Hamit-Mustafa
 */
#include "esc.h"

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!NOTLAR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//motor for döngüsüyle hızlandırıldı ve ayrı ayrı değerler verilerek çalıştırıldı.

//ŞU AN 50-100 ARASI DEĞER VERİYORUZ, MAP EDEBİLİRSİN VEYA ARALIĞI TİMER HESABINI 3360 YERİNE YARISINI,
//1000 YERİNE 2 KATINI VEREREK 2 KATINA ÇIKARTIRSIN

//config(pwm başlat) ve kalibrasyon(mainde whileın öncesinde çağır)

void init_esc() {
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void drive_motor_1(uint8_t pwm1Val) {

	TIM1->CCR1 = pwm1Val;
}

void drive_motor_2(uint8_t pwm2Val) {

	TIM1->CCR2 = pwm2Val;
}

void drive_motor_3(uint8_t pwm3Val) {

	TIM1->CCR3 = pwm3Val;
}

void drive_motor_4(uint8_t pwm4Val) {

	TIM1->CCR4 = pwm4Val;
}

void set_pwm(float roll_pid, float pitch_pid, float yaw_pid, uint16_t throttle,
		battery_t battery) {

	float pulse_length_esc1 = 0.0, pulse_length_esc2 = 0.0, pulse_length_esc3 =
			0.0, pulse_length_esc4 = 0.0;

	pulse_length_esc1 = throttle - roll_pid - pitch_pid + yaw_pid;
	pulse_length_esc2 = throttle + roll_pid - pitch_pid - yaw_pid;
	pulse_length_esc3 = throttle - roll_pid + pitch_pid - yaw_pid;
	pulse_length_esc4 = throttle + roll_pid + pitch_pid + yaw_pid;

	/*
	pulse_length_esc1 += pulse_length_esc1
			* ((BATTERY_START_VOLTAGE - battery.voltage)
					/ (float) BATTERY_COMPENSATION_COEFFICIENT);
	pulse_length_esc2 += pulse_length_esc2
			* ((BATTERY_START_VOLTAGE - battery.voltage)
					/ (float) BATTERY_COMPENSATION_COEFFICIENT);
	pulse_length_esc3 += pulse_length_esc3
			* ((BATTERY_START_VOLTAGE - battery.voltage)
					/ (float) BATTERY_COMPENSATION_COEFFICIENT);
	pulse_length_esc4 += pulse_length_esc4
			* ((BATTERY_START_VOLTAGE - battery.voltage)
					/ (float) BATTERY_COMPENSATION_COEFFICIENT);
*/
	//************************************ESC MAX OUTPUT LIMITING*********************************//
	if (pulse_length_esc1 > MAX_PWM_OUTPUT)
		pulse_length_esc1 = MAX_PWM_OUTPUT;

	if (pulse_length_esc2 > MAX_PWM_OUTPUT)
		pulse_length_esc2 = MAX_PWM_OUTPUT;

	if (pulse_length_esc3 > MAX_PWM_OUTPUT)
		pulse_length_esc3 = MAX_PWM_OUTPUT;

	if (pulse_length_esc4 > MAX_PWM_OUTPUT)
		pulse_length_esc4 = MAX_PWM_OUTPUT;
	//************************************ESC MIN OUTPUT LIMITING*********************************//

	if (pulse_length_esc1 < MIN_PWM_OUTPUT)
		pulse_length_esc1 = MIN_PWM_OUTPUT;

	if (pulse_length_esc2 < MIN_PWM_OUTPUT)
		pulse_length_esc2 = MIN_PWM_OUTPUT;

	if (pulse_length_esc3 < MIN_PWM_OUTPUT)
		pulse_length_esc3 = MIN_PWM_OUTPUT;

	if (pulse_length_esc4 < MIN_PWM_OUTPUT)
		pulse_length_esc4 = MIN_PWM_OUTPUT;

	drive_motor_1(pulse_length_esc1); // motor sürme fonksiyonları
	drive_motor_2(pulse_length_esc2);
	drive_motor_3(pulse_length_esc3);
	drive_motor_4(pulse_length_esc4);

}
