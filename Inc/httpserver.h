/*
 * httpserver.h
 *
 *  Created on: Jul 10, 2025
 *      Author: boran
 */

#ifndef INC_HTTPSERVER_H_
#define INC_HTTPSERVER_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"

typedef struct {
    float temperature;
    int32_t pressure;
    uint32_t timestamp;
} SensorData_t;

int getResHttp(char *req, char *res, SensorData_t *sensor_data);

#endif /* INC_HTTPSERVER_H_ */
