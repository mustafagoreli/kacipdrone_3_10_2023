#include "PIController.h"
#include "stdio.h"
char ptr3[100];
int pid_debug_counter;

//burada pid limmax ve lim min silinmeli çünkü biz throttle-pitch+yaw-roll vs gibi formüller kullanıyoruz..
//..bunların tamamının limiti konulmalı(pwm max ve pwm min olmalı bu da)

//sadece roll ve pitch için pid yapıyoruz. yaw ve yükseklik işini sonraya bırakacağız.

void PIDController_Init(PIDController *pid) {

	/* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError = 0.0f;

	pid->differentiator = 0.0f;
	pid->prevMeasurement = 0.0f;
	pid->T = 3.0f;

	pid->out = 0.0f;
	pid_debug_counter = 0;

}

float PIDController_Update(PIDController *pid, float setpoint,
		float measurement) {
	/*
	 * Error signal
	 */

	if (pid->T == 0)
		pid->T = 3;

	float error = setpoint - measurement; //setpoint 1k-2k arası, measurement min-180max180=>error 920-2080 arası

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
		printf("d_pid_integrator = ");
		printf(ptr3);
		printf("\n");

		gcvt(d_pid_diff, 8, ptr3);
		printf("d_pid_diff = ");
		printf(ptr3);
		printf("\n");

		gcvt(d_delta_t, 8, ptr3);
		printf("d_delta_t = ");
		printf(ptr3);
		printf("\n");

		pid_debug_counter = 0;

	}

	/* Return controller output */
	return pid->out;

}
