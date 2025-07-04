/*
 * uartCom.c
 *
 *  Created on: Jul 2, 2025
 *      Author: boran
 */
#include <stdio.h>
#include <string.h>
#include "uartCom.h"

extern UART_HandleTypeDef huart2;


void print2sh(int32_t x, uint8_t mode){
	uint8_t buffer[20];
	if (mode == 0)
		sprintf(buffer, "temp: %.2f °C\r\n", x/10.0);
	else
		sprintf(buffer, "pres: %ld Pa\r\n", x);

	HAL_UART_Transmit_DMA(&huart2, buffer, strlen(buffer));
}
