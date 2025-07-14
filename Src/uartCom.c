/*
 * uartCom.c
 *
 *  Created on: Jul 2, 2025
 *      Author: boran
 */

#include "uartCom.h"

HAL_StatusTypeDef status;
uint8_t tx_finished = 0;

void printTP2sh(double T, int32_t P){
	char buffer[56];

	snprintf(buffer, sizeof(buffer), "\t\ttemp: %.2f °C\npres: %ld Pa\r\n", T, P);

	while(tx_finished == 1);
	tx_finished = 1;
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)buffer, strlen(buffer));
}

void print2sh(char * str){
	int len = strlen(str);
	len = (len > 512) ? 512 : len;
	str[len] = '\0';
	while(tx_finished == 1);
	tx_finished = 1;
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*)str, len);

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART2){
		tx_finished = 0;
	}
}
