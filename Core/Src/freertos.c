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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//TUM
//BUNLARDAN BAZILARINI MUHTEMELEN YORUMA CEKECEGIZ, AYRICA HER BIRINE IDEAL DEGER ATANACAK
#define SAMPLE_TIME_BAROMETER 500
#define SAMPLE_TIME_POWER_MODULE 2000
#define SAMPLE_TIME_IMU 500//100
#define SAMPLE_TIME_GPS 500
#define SAMPLE_TIME_TX 500
#define SAMPLE_TIME_CONTROL 500//100
#define SAMPLE_TIME_ESC 500//100
#define SAMPLE_TIME_PWM_INPUT 500

#define FAILSAFE_VOLTAGE 15.2	//DEGISECEK
#define DEAD_VOLTAGE 14.2

#define PITCH_ROLL_KP 0	//DEGISECEK, PID PARAMETRELERI
#define PITCH_ROLL_KI 0
#define PITCH_ROLL_KD 0

#define PWM_POS_PITCH 1700//jetsondan gelen verilere göre pozitif veya negatif pitch/roll vereceğiz
#define PWM_NEG_PITCH 1200//bunlar da onların sabit pwmleri, sabit olmazsa değiştiririz

#define PWM_POS_ROLL 1700
#define PWM_NEG_ROLL 1200

#define MANUAL_MODE 0//otonom manual modda olduğunu gösterecek
#define AUTONOMOUS_MODE 1


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
uint16_t PID_ROLL = 0, PID_PITCH = 0, PID_YAW = 0, THROTTLE = 0;
uint8_t drive_mode = MANUAL_MODE; //otonom olacağı zaman 1de başlat
//otonom manual modda olduğunu gösterecek

// ************************   PID finish   *********************************

// ************************   UART start   *********************************
extern uartBuffer_t uartBuffer;
extern bool dataReceived;
char comma[] = ",";
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
// ************************   GPS finish   *********************************

/* USER CODE END Variables */
osThreadId controlTaskHandle;
osThreadId barometerTaskHandle;
osThreadId heartbeatTaskHandle;
osThreadId pwmTaskHandle;
osThreadId rcTaskHandle;
osThreadId imuTaskHandle;
osThreadId sendDataTaskHandle;
osThreadId batteryTaskHandle;
osThreadId gpsTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartControlTask(void const *argument);
void startBarometerTask(void const *argument);
void starHeartbeatTask(void const *argument);
void startPwmTask(void const *argument);
void StartRCTask(void const *argument);
void StartImuTask(void const *argument);
void StartSendDataTask(void const *argument);
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
	/* definition and creation of controlTask */
	osThreadDef(controlTask, StartControlTask, osPriorityNormal, 0, 128);
	controlTaskHandle = osThreadCreate(osThread(controlTask), NULL);

	/* definition and creation of barometerTask */
	osThreadDef(barometerTask, startBarometerTask, osPriorityIdle, 0, 128);
	barometerTaskHandle = osThreadCreate(osThread(barometerTask), NULL);

	/* definition and creation of heartbeatTask */
	osThreadDef(heartbeatTask, starHeartbeatTask, osPriorityIdle, 0, 128);
	heartbeatTaskHandle = osThreadCreate(osThread(heartbeatTask), NULL);

	/* definition and creation of pwmTask */
	osThreadDef(pwmTask, startPwmTask, osPriorityIdle, 0, 128);
	pwmTaskHandle = osThreadCreate(osThread(pwmTask), NULL);

	/* definition and creation of rcTask */
	osThreadDef(rcTask, StartRCTask, osPriorityIdle, 0, 128);
	rcTaskHandle = osThreadCreate(osThread(rcTask), NULL);

	/* definition and creation of imuTask */
	osThreadDef(imuTask, StartImuTask, osPriorityIdle, 0, 128);
	imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

	/* definition and creation of sendDataTask */
	osThreadDef(sendDataTask, StartSendDataTask, osPriorityIdle, 0, 128);
	sendDataTaskHandle = osThreadCreate(osThread(sendDataTask), NULL);

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

/* USER CODE BEGIN Header_StartControlTask */
/**
 * @brief  Function implementing the controlTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartControlTask */
void StartControlTask(void const *argument) {
	/* USER CODE BEGIN StartControlTask */
	/* Infinite loop */
	for (;;) {
		osDelay(SAMPLE_TIME_CONTROL);
	}
	/* USER CODE END StartControlTask */
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
	/* Infinite loop */
	for (;;) {
		/*
		 BME280_Measure();
		 Altitude = 44330
		 * (1
		 - (pow(((float) Pressure / (float) atmPress),
		 0.19029495718)));
		 Altitude = round(Altitude - initialAltitude);
		 osDelay(SAMPLE_TIME_BAROMETER);
		 */
	}
	/* USER CODE END startBarometerTask */
}

/* USER CODE BEGIN Header_starHeartbeatTask */
/**
 * @brief Function implementing the heartbeatTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_starHeartbeatTask */
void starHeartbeatTask(void const *argument) {
	/* USER CODE BEGIN starHeartbeatTask */
	/* Infinite loop */
	for (;;) {
		/*
		 gcvt(newAltitude1, 6, ptr);
		 strcat(ptr,comma);
		 gcvt(newAltitude2, 6, ptr + 8);
		 printf("Hello World = ");
		 printf(ptr);
		 printf("\n");
		 */
		osDelay(1000);
	}
	/* USER CODE END starHeartbeatTask */
}

/* USER CODE BEGIN Header_startPwmTask */
/**
 * @brief Function implementing the pwmTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startPwmTask */
void startPwmTask(void const *argument) {
	/* USER CODE BEGIN startPwmTask */
	/* Infinite loop */
	for (;;) {

		//set_pwm(PID_ROLL, PID_PITCH, PID_YAW, THROTTLE, battery);

		osDelay(SAMPLE_TIME_ESC);
	}
	/* USER CODE END startPwmTask */
}

/* USER CODE BEGIN Header_StartRCTask */
/**
 * @brief Function implementing the rcTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartRCTask */
void StartRCTask(void const *argument) {
	/* USER CODE BEGIN StartRCTask */
	/* Infinite loop */
	for (;;) {
		osDelay(SAMPLE_TIME_PWM_INPUT);
	}
	/* USER CODE END StartRCTask */
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

		bno055_vector_t imu = bno055_getVectorEuler();

		roll_pid_t.T = deltaT;
		pitch_pid_t.T = deltaT;

		__HAL_TIM_SET_COUNTER(&htim5, 0);


		if (drive_mode == AUTONOMOUS_MODE) {

			/*
			 * jetsondan gelen veriye göre wanted setpoint değerini burada güncelleyeceğiz
			 *
			PID_PITCH = PIDController_Update(&pitch_pid_t, wanted_setpoint,
					imu.x);	//pitch +180 -180 aralığında
			PID_ROLL = PIDController_Update(&roll_pid_t, wanted_setpoint,
					imu.y);//roll +90 -90 aralığında
		*/}


		else
		{
			PID_PITCH = PIDController_Update(&pitch_pid_t, pwm_ch2.dutyCycle,
					imu.x);	//pitch +180 -180 aralığında
			PID_ROLL = PIDController_Update(&roll_pid_t, pwm_ch3.dutyCycle,
					imu.y);	//roll +90 -90 aralığında
		}


		deltaT = __HAL_TIM_GET_COUNTER(&htim5);
		osDelay(SAMPLE_TIME_IMU);

	}
	/* USER CODE END StartImuTask */
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
		osDelay(SAMPLE_TIME_TX);
	}
	/* USER CODE END StartSendDataTask */
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
	BatteryInit();

	/* Infinite loop */
	for (;;) {
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
		}

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
		if (Wait_for("GGA") == 1) {

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
