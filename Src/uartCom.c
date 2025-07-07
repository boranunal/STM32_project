/*
 * uartCom.c
 *
 *  Created on: Jul 2, 2025
 *      Author: boran
 */
#include <stdio.h>
#include <string.h>
#include "uartCom.h"


void print2sh(int32_t x, uint8_t mode){
	uint8_t buffer[32];
	if (mode == 0)
		sprintf(buffer, "temp: %.2f °C\t\t\t", x/10.0);
	else
		sprintf(buffer, "pres: %ld Pa\t\t\r", x);

	HAL_NVIC_DisableIRQ(TIM4_IRQn);
	HAL_UART_Transmit_DMA(&huart2, buffer, strlen(buffer));
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART2){
		HAL_NVIC_EnableIRQ(TIM4_IRQn);
	}
}
