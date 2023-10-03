/*
 * init.c
 *
 *  Created on: Oct 3, 2023
 *      Author: Hamit-Mustafa
 */




/*
GPS
barometre
jetsona giden
jetsondan gelen
*/


/*

BatteryInit();							//batarya
init_esc();								//esc
PIDController_Init(&pitch_pid_t);		//pid pitch
PIDController_Init(&roll_pid_t);		    //pid roll
init_pwm_read();						//kumanda

bno055_assignI2C(&hi2c3);				//imu
bno055_setup();
bno055_setOperationModeNDOF();

//pitch ve roll pid parametreleri x konfigurasyonunda aynı olabilir
pitch_pid_t.Kp = PITCH_ROLL_KP;
pitch_pid_t.Ki = PITCH_ROLL_KI;
pitch_pid_t.Kd = PITCH_ROLL_KD;

roll_pid_t.Kp = PITCH_ROLL_KP;
roll_pid_t.Ki = PITCH_ROLL_KI;
roll_pid_t.Kd = PITCH_ROLL_KD;
*/
