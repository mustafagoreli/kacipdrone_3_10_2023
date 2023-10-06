/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BME280_STM32.h"
#include "math.h"
#include "stdio.h"
#include "usart.h"
#include "stdlib.h"
#include "string.h"
#include "stdbool.h"
#include "bno055_stm32.h"
#include "power_module.h"
#include "pwm_input.h"
#include "PIController.h"
#include "tim.h"
#include "NMEA.h"
#include "esc.h"
#include "uartRingBuffer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//TUM
//BUNLARDAN BAZILARINI MUHTEMELEN YORUMA CEKECEGIZ, AYRICA HER BIRINE IDEAL DEGER ATANACAK
#define SAMPLE_TIME_BAROMETER 1000
#define SAMPLE_TIME_POWER_MODULE 1000

#define SAMPLE_TIME_GPS 1000

#define SAMPLE_TIME_TX 200
#define SAMPLE_TIME_IMU 100//100
#define SAMPLE_TIME_ESC 100//100
//#define SAMPLE_TIME_PWM_INPUT 500

#define FAILSAFE_VOLTAGE 15.2	//DEGISECEK
#define DEAD_VOLTAGE 14.2

#define PITCH_ROLL_KP 1	//DEGISECEK, PID PARAMETRELERI
#define PITCH_ROLL_KI 1
#define PITCH_ROLL_KD 1

#define PWM_POS_PITCH 1700//jetsondan gelen verilere göre pozitif veya negatif pitch/roll vereceğiz
#define PWM_NEG_PITCH 1300//bunlar da onların sabit pwmleri, sabit olmazsa değiştiririz

#define PWM_POS_ROLL 1700
#define PWM_NEG_ROLL 1300

#define MANUAL_MODE 0//otonom manual modda olduğunu gösterecek
#define AUTONOMOUS_MODE 1

#define TRUE '1'
#define FALSE '0'
#define START 's'
#define FINISH 'f'

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// ************************   PID start   *********************************
PIDController pitch_pid_t;
PIDController roll_pid_t;
//PIDController height_pid_t;
uint16_t deltaT = 0;
bno055_vector_t imu;
// ************************   PID finish   *********************************

// ************************   ESC start   *********************************
uint16_t PID_ROLL = 0, PID_PITCH = 0, PID_YAW = 0, THROTTLE = 0;
uint8_t drive_mode = MANUAL_MODE; //otonom olacağı zaman 1de başlat
//otonom manual modda olduğunu gösterecek
// ************************   ESC finish   *********************************

// ************************   UART start   *********************************
extern uartBuffer_t uartBuffer;
char ptr[100];
// ************************   UART finish   *********************************

// ************************   Battery start   *********************************
battery_t battery;
// ************************   Battery finish   *********************************

// ************************   Barometre start   *********************************
bme280_t bme280;
// ************************   Barometre finish   *********************************

// ************************   GPS start   *********************************
char GGA[100];
char RMC[100];
GPSSTRUCT gpsData;
int flagGGA = 0, flagRMC = 0;
int VCCTimeout = 5000;
char GGA[] = "GGA";
// ************************   GPS finish   *********************************

/* USER CODE END Variables */
osThreadId sendDataTaskHandle;
osThreadId barometerTaskHandle;
osThreadId escTaskHandle;
osThreadId imuTaskHandle;
osThreadId batteryTaskHandle;
osThreadId gpsTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartSendDataTask(void const *argument);
void startBarometerTask(void const *argument);
void startEscTask(void const *argument);
void StartImuTask(void const *argument);
void StartBatteryTask(void const *argument);
void StartGPSTask(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
		StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
		StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
	*ppxIdleTaskStackBuffer = &xIdleStack[0];
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
	/* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of sendDataTask */
	osThreadDef(sendDataTask, StartSendDataTask, osPriorityNormal, 0, 128);
	sendDataTaskHandle = osThreadCreate(osThread(sendDataTask), NULL);

	/* definition and creation of barometerTask */
	osThreadDef(barometerTask, startBarometerTask, osPriorityIdle, 0, 128);
	barometerTaskHandle = osThreadCreate(osThread(barometerTask), NULL);

	/* definition and creation of escTask */
	osThreadDef(escTask, startEscTask, osPriorityIdle, 0, 128);
	escTaskHandle = osThreadCreate(osThread(escTask), NULL);

	/* definition and creation of imuTask */
	osThreadDef(imuTask, StartImuTask, osPriorityIdle, 0, 128);
	imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

	/* definition and creation of batteryTask */
	osThreadDef(batteryTask, StartBatteryTask, osPriorityIdle, 0, 128);
	batteryTaskHandle = osThreadCreate(osThread(batteryTask), NULL);

	/* definition and creation of gpsTask */
	osThreadDef(gpsTask, StartGPSTask, osPriorityIdle, 0, 128);
	gpsTaskHandle = osThreadCreate(osThread(gpsTask), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartSendDataTask */
/**
 * @brief Function implementing the sendDataTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSendDataTask */
void StartSendDataTask(void const *argument) {
	/* USER CODE BEGIN StartSendDataTask */

	/* Infinite loop */
	for (;;) {
		//  Datalar S + IMU (pitch roll yaw) + Barometre + Sürüş Mode + Power Status + F
		/*
		 printf(data);
		 printf(ptr);
		 printf(newLine);
		 */
		printf("s");
		gcvt(imu.y, 8, ptr);
		printf(ptr);
		printf(",");

		gcvt(imu.z, 8, ptr);
		printf(ptr);
		printf(",");

		gcvt(imu.x, 8, ptr);
		printf(ptr);
		printf(",");

		gcvt(bme280.altitude, 8, ptr);
		printf(ptr);
		printf(",");

		printf("%d", drive_mode);
		printf(",");

		gcvt(battery.voltage, 8, ptr);
		printf(ptr);
		printf("f\n");
		/*
		 gcvt(bme280.altitude, 8, ptr);
		 printf("BAROMETRE = ");
		 printf(ptr);
		 printf("\n");
		 gcvt(imu.x, 8, ptr);
		 printf("YAW = ");
		 printf(ptr);
		 printf("\n");
		 gcvt(imu.y, 8, ptr);
		 printf("PITCH = ");
		 printf(ptr);
		 printf("\n");
		 gcvt(imu.z, 8, ptr);
		 printf("ROLL = ");
		 printf(ptr);
		 printf("\n");
		 gcvt(gpsData.ggastruct.lcation.latitude, 8, ptr);
		 printf("Lat = ");
		 printf(ptr);
		 printf("\n");
		 gcvt(gpsData.ggastruct.lcation.longitude, 8, ptr);
		 printf("Long = ");
		 printf(ptr);
		 printf("\n");
		 */
		/*
		 gcvt(pwm_ch1.dutyCycle, 8, ptr);
		 printf("Ch1 dutyCycle = ");
		 printf(ptr);
		 printf("\n");
		 gcvt(pwm_ch2.dutyCycle, 8, ptr);
		 printf("Ch2 dutyCycle = ");
		 printf(ptr);
		 printf("\n");
		 gcvt(pwm_ch3.dutyCycle, 8, ptr);
		 printf("Ch3 dutyCycle = ");
		 printf(ptr);
		 printf("\n");
		 gcvt(pwm_ch4.dutyCycle, 8, ptr);
		 printf("Ch4 dutyCycle = ");
		 printf(ptr);
		 printf("\n");
		 gcvt(pwm_ch5.dutyCycle, 8, ptr);
		 printf("Ch5 dutyCycle = ");
		 printf(ptr);
		 printf("\n");*/
		osDelay(SAMPLE_TIME_TX);
	}
	/* USER CODE END StartSendDataTask */
}

/* USER CODE BEGIN Header_startBarometerTask */
/**
 * @brief Function implementing the barometerTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startBarometerTask */
void startBarometerTask(void const *argument) {
	/* USER CODE BEGIN startBarometerTask */
	BME280_Config(OSRS_2, OSRS_16, OSRS_1, MODE_NORMAL, T_SB_0p5, IIR_16);
	BME280_Measure();
	bme280.initialAltitude = 44330
			* (1
					- (pow(((float) bme280.pressure / (float) atmPress),
							0.19029495718)));
	/* Infinite loop */
	for (;;) {
		BME280_Measure();
		bme280.altitude = 44330
				* (1
						- (pow(((float) bme280.pressure / (float) atmPress),
								0.19029495718)));
		bme280.finalAltitude = bme280.altitude - bme280.initialAltitude;
		osDelay(SAMPLE_TIME_BAROMETER);
	}
	/* USER CODE END startBarometerTask */
}

/* USER CODE BEGIN Header_startEscTask */
/**
 * @brief Function implementing the escTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startEscTask */
void startEscTask(void const *argument) {
	/* USER CODE BEGIN startEscTask */
	//init_esc();
	/* Infinite loop */
	for (;;) {

		if (AUTONOMOUS_MODE == drive_mode) {
			set_pwm(PID_ROLL, PID_PITCH, PID_YAW, THROTTLE, battery);
		} else {
			set_pwm(PID_ROLL, PID_PITCH, PID_YAW, pwm_ch3.dutyCycle, battery); //ch3 throttle
		}

		osDelay(SAMPLE_TIME_ESC);

	}
	/* USER CODE END startEscTask */
}

/* USER CODE BEGIN Header_StartImuTask */
/**
 * @brief Function implementing the imuTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartImuTask */
void StartImuTask(void const *argument) {
	/* USER CODE BEGIN StartImuTask */
	bno055_assignI2C(&hi2c3);
	bno055_setup();
	bno055_setOperationModeNDOF();

	PIDController_Init(&pitch_pid_t); //pid sıfırlama
	PIDController_Init(&roll_pid_t);

	//pitch ve roll pid parametreleri x konfigurasyonunda aynı olabilir
	pitch_pid_t.Kp = PITCH_ROLL_KP;
	pitch_pid_t.Ki = PITCH_ROLL_KI;
	pitch_pid_t.Kd = PITCH_ROLL_KD;

	roll_pid_t.Kp = PITCH_ROLL_KP;
	roll_pid_t.Ki = PITCH_ROLL_KI;
	roll_pid_t.Kd = PITCH_ROLL_KD;

	for (;;) {
		imu = bno055_getVectorEuler();

		roll_pid_t.T = deltaT;
		pitch_pid_t.T = deltaT;

		__HAL_TIM_SET_COUNTER(&htim5, 0);
		if (AUTONOMOUS_MODE == drive_mode) {

			//jetsondan gelen veriye göre wanted setpoint değerini burada güncelleyeceğiz
			//imuda x yaw y pitch z roll
			if (uartBuffer.movementData.pitchPositive == TRUE) {
				PID_PITCH = PIDController_Update(&pitch_pid_t, PWM_POS_PITCH,
						imu.y);	//pitch +180 -180 aralığında

			} else if (uartBuffer.movementData.pitchNegative == TRUE) {
				PID_PITCH = PIDController_Update(&pitch_pid_t, PWM_NEG_PITCH,
						imu.y);	//pitch +180 -180 aralığında

			}
			if (uartBuffer.movementData.rollPositive == TRUE) {
				PID_ROLL = PIDController_Update(&roll_pid_t, PWM_POS_ROLL,
						imu.z);	//roll +90 -90 aralığında
			} else if (uartBuffer.movementData.rollNegative == TRUE) {
				PID_ROLL = PIDController_Update(&roll_pid_t, PWM_NEG_ROLL,
						imu.z);	//roll +90 -90 aralığında
			}
		}

		else {	//kumanda bölümü 		//ch5roll sol yan	//ch2 pitch
			PID_PITCH = PIDController_Update(&pitch_pid_t, pwm_ch2.dutyCycle,
					imu.y);	//pitch +180 -180 aralığında
			PID_ROLL = PIDController_Update(&roll_pid_t, pwm_ch5.dutyCycle,
					imu.z);	//roll +90 -90 aralığında
		}
		deltaT = __HAL_TIM_GET_COUNTER(&htim5);
		deltaT = deltaT / 2;	//şu anda milisaniye olarak ölçüyoruz
		osDelay(SAMPLE_TIME_IMU);

	}
	/* USER CODE END StartImuTask */
}

/* USER CODE BEGIN Header_StartBatteryTask */
/**
 * @brief Function implementing the batteryTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBatteryTask */
void StartBatteryTask(void const *argument) {
	/* USER CODE BEGIN StartBatteryTask */
	//BatteryInit();
	/* Infinite loop */
	for (;;) {
		/*
		 battery.voltage = getBatteryVoltage();

		 if (battery.voltage < FAILSAFE_VOLTAGE
		 && battery.voltage >= DEAD_VOLTAGE) {
		 battery.isBatteryLow = true;

		 } else if (battery.voltage < DEAD_VOLTAGE) {
		 battery.isBatteryDead = true;
		 battery.isBatteryLow = false;
		 } else {
		 battery.isBatteryDead = false;
		 battery.isBatteryLow = false;
		 }*/
		osDelay(SAMPLE_TIME_POWER_MODULE);
	}
	/* USER CODE END StartBatteryTask */
}

/* USER CODE BEGIN Header_StartGPSTask */
/**
 * @brief Function implementing the gpsTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartGPSTask */
void StartGPSTask(void const *argument) {
	/* USER CODE BEGIN StartGPSTask */
	/* Infinite loop */
	for (;;) {
		if (Wait_for(GGA) == 1) {

			VCCTimeout = 5000; // Reset the VCC Timeout indicating the GGA is being received

			Copy_upto("*", GGA);
			if (decodeGGA(GGA, &gpsData.ggastruct) == 0)
				flagGGA = 2;  // 2 indicates the data is valid
			else
				flagGGA = 1;  // 1 indicates the data is invalid
		}

		if (Wait_for("RMC") == 1) {

			VCCTimeout = 5000; // Reset the VCC Timeout indicating the RMC is being received

			Copy_upto("*", RMC);
			if (decodeRMC(RMC, &gpsData.rmcstruct) == 0)
				flagRMC = 2;  // 2 indicates the data is valid
			else
				flagRMC = 1;  // 1 indicates the data is invalid
		}

		if ((flagGGA == 2) | (flagRMC == 2)) {
		}

		else if ((flagGGA == 1) | (flagRMC == 1)) {
			// Instead of clearing the display, it's better if we print spaces.
			// This will avoid the "refreshing" part
		}

		if (VCCTimeout <= 0) {
			VCCTimeout = 5000;  // Reset the timeout

			//reset flags
			flagGGA = flagRMC = 0;

			// You are here means the VCC is less, or maybe there is some connection issue
			// Check the VCC, also you can try connecting to the external 5V
		}

		osDelay(SAMPLE_TIME_GPS);
	}
	/* USER CODE END StartGPSTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
