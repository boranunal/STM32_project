/*
 * uartCom.c
 *
 *  Created on: Jul 2, 2025
 *      Author: boran
 */
#include <stdio.h>
#include <string.h>
#include "uartCom.h"

HAL_StatusTypeDef status;
volatile uint8_t tx_finished = 0;

void printTP2sh(int32_t x, uint8_t mode){
	char buffer[32];
	if (mode == 0)
		snprintf(buffer, sizeof(buffer), "temp: %.2f °C\t\t\t", x/10.0);
	else
		snprintf(buffer, sizeof(buffer), "pres: %ld Pa\t\t\r", x);

	while(tx_finished == 1);
	tx_finished = 1;
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)buffer, strlen(buffer));
}

void print2sh(char * str){

	while(tx_finished == 1);
	tx_finished = 1;
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)str, strlen(str));

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART2){
		tx_finished = 0;
	}
}
