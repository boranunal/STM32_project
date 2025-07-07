/*
 * uartCom.h
 *
 *  Created on: Jul 2, 2025
 *      Author: root
 */

#ifndef INC_UARTCOM_H_
#define INC_UARTCOM_H_


#include "stm32f4xx_hal.h"
#include "main.h"

void print2sh(int32_t x, uint8_t mode);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

#endif /* INC_UARTCOM_H_ */
