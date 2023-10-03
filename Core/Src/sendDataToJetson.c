/*
 * sendDataToJetson.c
 *
 *  Created on: Sep 28, 2023
 *      Author: CAN
 */
#include "sendDataToJetson.h"

void sendData(uint8_t* data, uint8_t size){
	HAL_UART_Transmit(&huart2, (uint8_t *)data, size, 100);
}
