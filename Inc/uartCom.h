/*
 * uartCom.h
 *
 *  Created on: Jul 2, 2025
 *      Author: boran
 */

#ifndef INC_UARTCOM_H_
#define INC_UARTCOM_H_


#include "stm32f4xx_hal.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

void printTP2sh(double  T, int32_t P);
void print2sh(char * buffer);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

#endif /* INC_UARTCOM_H_ */
