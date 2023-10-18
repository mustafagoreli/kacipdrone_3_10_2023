/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
	uint8_t start;
	uint8_t landing;
	uint8_t takeoff;
	uint8_t pitchPositive;
	uint8_t pitchNegative;
	uint8_t rollPositive;
	uint8_t rollNegative;
	uint8_t finish;
} movementData_t;

typedef union {
	char buffer[8];
	movementData_t movementData;
} uartBuffer_t;

typedef struct {
	bool flag;
	uint16_t counter;
} failsafe_t;


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define pwm_input_1_Pin GPIO_PIN_5
#define pwm_input_1_GPIO_Port GPIOE
#define pwm_input_2_Pin GPIO_PIN_0
#define pwm_input_2_GPIO_Port GPIOA
#define power_module_adc_Pin GPIO_PIN_1
#define power_module_adc_GPIO_Port GPIOA
#define gps_uart_tx_Pin GPIO_PIN_2
#define gps_uart_tx_GPIO_Port GPIOA
#define gps_uart_rx_Pin GPIO_PIN_3
#define gps_uart_rx_GPIO_Port GPIOA
#define pwm_input_3_Pin GPIO_PIN_6
#define pwm_input_3_GPIO_Port GPIOA
#define pwm_output_1_Pin GPIO_PIN_9
#define pwm_output_1_GPIO_Port GPIOE
#define pwm_output_2_Pin GPIO_PIN_11
#define pwm_output_2_GPIO_Port GPIOE
#define pwm_output_3_Pin GPIO_PIN_13
#define pwm_output_3_GPIO_Port GPIOE
#define pwm_output_4_Pin GPIO_PIN_14
#define pwm_output_4_GPIO_Port GPIOE
#define debug_ttl_tx_Pin GPIO_PIN_10
#define debug_ttl_tx_GPIO_Port GPIOB
#define debug_ttl_rx_Pin GPIO_PIN_11
#define debug_ttl_rx_GPIO_Port GPIOB
#define pwm_input_5_Pin GPIO_PIN_14
#define pwm_input_5_GPIO_Port GPIOB
#define pwm_input_4_Pin GPIO_PIN_12
#define pwm_input_4_GPIO_Port GPIOD
#define led_2_Pin GPIO_PIN_13
#define led_2_GPIO_Port GPIOD
#define led_1_Pin GPIO_PIN_14
#define led_1_GPIO_Port GPIOD
#define imu_sda_i2c3_Pin GPIO_PIN_9
#define imu_sda_i2c3_GPIO_Port GPIOC
#define imu_scl_i2c3_Pin GPIO_PIN_8
#define imu_scl_i2c3_GPIO_Port GPIOA
#define jetson_uart_tx_Pin GPIO_PIN_9
#define jetson_uart_tx_GPIO_Port GPIOA
#define jetson_uart_rx_Pin GPIO_PIN_10
#define jetson_uart_rx_GPIO_Port GPIOA
#define barometre_scl_Pin GPIO_PIN_6
#define barometre_scl_GPIO_Port GPIOB
#define barometre_sda_Pin GPIO_PIN_7
#define barometre_sda_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
