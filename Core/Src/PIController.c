#include "PIController.h"
#include "stdio.h"
#include "pwm_input.h"

char ptr3[100];
int pid_debug_counter;
float debug_yaw_speed_once = 0.0;
float yaw_speed, IMUYaw;
//burada pid limmax ve lim min silinmeli çünkü biz throttle-pitch+yaw-roll vs gibi formüller kullanıyoruz..
//..bunların tamamının limiti konulmalı(pwm max ve pwm min olmalı bu da)

//sadece roll ve pitch için pid yapıyoruz. yaw ve yükseklik işini sonraya bırakacağız.

void PIDController_Init(PIDController *pid) {

	/* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError = 0.0f;

	pid->differentiator = 0.0f;
	pid->prevMeasurement = 0.0f;
	pid->T = 3000;

	pid->out = 0.0f;
	pid_debug_counter = 0;

}

float PIDController_Update(PIDController *pid, float setpoint,
		float measurement) {
	/*
	 * Error signal
	 */
	/*
	 gcvt(measurement, 8, ptr3);
	 printf("measurement = ");
	 printf(ptr3);
	 printf("\n");
	 */

	if(pid->ID == ROLL_ID){
		if(measurement < 0){
			measurement = measurement * 0.95;
		}else{
			measurement = measurement * 1.05;
		}
	}

	float error = 0.0;
	if (pid->T == 0)
		pid->T = 3000; //sample time değişince değiştir!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	if (pid->ID == PITCH_ID) {
		error = -(setpoint - measurement); //setpoint 1k-2k arası, measurement min-180max180=>error 920-2080 arası
	} else if (pid->ID == ROLL_ID) {
		error = setpoint - measurement; //setpoint 1k-2k arası, measurement min-180max180=>error 920-2080 arası
	}
	// Konumdan Hız Hesaplıyoruz
	else if (pid->ID == YAW_ID) {

		IMUYaw = MAP(measurement, 0.0, 360.0, -180.0, 180.0);

		/*
		 gcvt(IMUYaw, 8, ptr3);
		 printf("IMUYaw = ");
		 printf(ptr3);
		 printf("\n");
		 */

		yaw_speed = (measurement - pid->prevMeasurement);
		if (yaw_speed <= 300 || yaw_speed >= 300) {
			if (pid->prevMeasurement < 0 && measurement > 0) {
				pid->prevMeasurement = -pid->prevMeasurement;
				yaw_speed =
						-((180 - measurement) + (180 - pid->prevMeasurement));

			} else if (pid->prevMeasurement > 0 && measurement < 0) {
				measurement = -measurement;
				yaw_speed = (180 - measurement) + (180 - pid->prevMeasurement);
			}
		}
		yaw_speed = yaw_speed / pid->T;
		debug_yaw_speed_once = yaw_speed;

		/*
		 gcvt(yaw_speed, 8, ptr3);
		 printf("YAW Speed Map oncesi = ");
		 printf(ptr3);
		 printf("\n");
		 */

		yaw_speed = MAP(yaw_speed, -0.003, 0.003, -180.0, 180.0);

		/*
		 gcvt(yaw_speed, 8, ptr3);
		 printf("YAW Speed Map sonrasi = ");
		 printf(ptr3);
		 printf("\n");
		 */

		//setpoint = MAP(setpoint, -0.045, 0.045, -180.0, 180.0);
		error = setpoint - yaw_speed;
		if (error < 1.5 && error > -1.8) { //dead zone
			error = 0;
			pid->integrator = 0;
		}
	}
	/*
	 gcvt(IMUYaw, 8, ptr3);
	 printf("IMUYaw = ");
	 printf(ptr);
	 printf("\n");
	 */
	/*

	 if (error < 1.5 && error > -1.5) { //dead zone
	 error = 0;
	 }
	 */

	/*
	 * Proportional
	 */
	float proportional = pid->Kp * error;

	pid->proportional = proportional; //sadece debug için
	/*
	 * Integral
	 */
	pid->integrator = pid->integrator
			+ 0.5f * pid->Ki * pid->T * (error + pid->prevError);

	/* Anti-wind-up via integrator clamping */
	if (pid->integrator > pid->limMaxInt) {

		pid->integrator = pid->limMaxInt;

	} else if (pid->integrator < pid->limMinInt) {

		pid->integrator = pid->limMinInt;

	}

	/* BU PID ORIJINAL, BUNU ANLAMADIK
	 * Derivative (band-limited differentiator)
	 */
	/*
	 pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)
	 + (2.0f * pid->tau - pid->T) * pid->differentiator)
	 / (2.0f * pid->tau + pid->T);
	 */
	//https://www.robotsforroboticists.com/pid-control/ //derivative = (error – error_prior) / iteration_time
	pid->differentiator = pid->Kd * ((error - pid->prevError) / pid->T);

	/*
	 * Compute output and apply limits
	 */
	pid->out = proportional + pid->integrator + pid->differentiator;

	if (pid->out > pid->limMax) {

		pid->out = pid->limMax;

	} else if (pid->out < pid->limMin) {

		pid->out = pid->limMin;

	}

	/* Store error and measurement for later use */
	pid->prevError = error;
	pid->prevMeasurement = measurement;

	float d_pid_out = pid->out;
	float d_pid_proportional = proportional;
	float d_pid_integrator = pid->integrator;
	float d_pid_diff = pid->differentiator;
	float d_delta_t = (float) pid->T;

	pid_debug_counter++;
	if (pid_debug_counter == 25) {

		/*
		 gcvt(measurement, 8, ptr3);
		 printf("measurement = ");
		 printf(ptr3);
		 printf("\n");

		 gcvt(debug_yaw_speed_once, 8, ptr3);
		 printf("MAP ONCESI yaw_speed = ");
		 printf(ptr3);
		 printf("\n");

		 gcvt(yaw_speed, 8, ptr3);
		 printf("MAP SONRASI yaw_speed = ");
		 printf(ptr3);
		 printf("\n");

		 printf("--------------------------------\n");

		 pid_debug_counter = 0;
		 }
		 */
		if (pid->ID == PITCH_ID) {
			printf("\n------------PITCH DATALAR \n");
		} else if (pid->ID == ROLL_ID) {
			printf("\n------------ROLL DATALAR \n");
		} else if (pid->ID == YAW_ID) {
			printf("\n------------YAW DATALAR \n");
			gcvt(yaw_speed, 8, ptr3);
			printf("yaw_speed = ");
			printf(ptr3);
			printf("\n");

			gcvt(measurement, 8, ptr3);
			printf("measurement = ");
			printf(ptr3);
			printf("\n");

		}
		//debug
		printf("\n");
		gcvt(d_pid_out, 8, ptr3);
		printf("pid_out = ");
		printf(ptr3);
		printf("\n");

		gcvt(d_pid_proportional, 8, ptr3);
		printf("d_pid_proportional = ");
		printf(ptr3);
		printf("\n");

		gcvt(d_pid_integrator, 8, ptr3);
		printf("d_pid_integrator =           ");
		printf(ptr3);
		printf("\n");

		gcvt(d_pid_diff, 8, ptr3);
		printf("d_pid_diff = ");
		printf(ptr3);
		printf("\n");

		gcvt(d_delta_t, 8, ptr3);
		printf("d_delta_t = ");
		printf(ptr3);
		printf("\n\n");

		gcvt(setpoint, 8, ptr3);
		printf("setpoint = ");
		printf(ptr3);
		printf("\n");

		gcvt(measurement, 8, ptr3);
		printf("measurement = ");
		printf(ptr3);
		printf("\n");

		gcvt(error, 8, ptr3);
		printf("error = ");
		printf(ptr3);
		printf("\n");
		printf("--------------------------\n");
		HAL_GPIO_TogglePin(led_2_GPIO_Port, led_2_Pin);
		pid_debug_counter = 0;

	}
	/* Return controller output */

	return pid->out;

}
