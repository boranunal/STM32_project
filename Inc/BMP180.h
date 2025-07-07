#ifndef BMP180_H
#define BMP180_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include "uartCom.h"

#define BMP180ReadAddr 0xEF
#define BMP180WrAddr 0xEE

extern volatile uint8_t rawDataReady;
extern volatile int32_t UD;
typedef struct BMP180_EEPROM {
	short AC1;
	short AC2;
	short AC3;
	unsigned short AC4;
	unsigned short AC5;
	unsigned short AC6;
	short B1;
	short B2;
	short MB;
	short MC;
	short MD;
} BMP180_EEPROM;

void BMP180_init();
void readRawData(uint8_t mode);
int32_t calcTemp(int32_t UT);
int32_t calcPres(int32_t UP);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);



#endif
