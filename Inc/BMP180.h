#ifndef BMP180_H
#define BMP180_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include "uartCom.h"

#define BMP180ReadAddr 0xEF
#define BMP180WrAddr 0xEE


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

typedef struct{
	BMP180_EEPROM calib;
	TIM_HandleTypeDef *htim;
	I2C_HandleTypeDef *hi2c;
	uint8_t *raw_data;
	int32_t ut, up;
	int32_t b5;
	uint8_t state;
	uint8_t temp_ready;
	uint8_t pres_ready;
} BMP180_Handle_t;

void BMP180_init(BMP180_Handle_t *hbmp180);
void readRawData(BMP180_Handle_t *hbmp180);
void BMP180_readRawData(BMP180_Handle_t *hbmp180);
void BMP180_waitData(BMP180_Handle_t *hbmp180);
void BMP180_dataReadyToGet(BMP180_Handle_t *hbmp180);
void BMP180_getData(BMP180_Handle_t *hbmp180);
int32_t BMP180_calcTemp(BMP180_Handle_t *hbmp180);
int32_t BMP180_calcPres(BMP180_Handle_t *hbmp180);
void BMP180_I2C_TxCpltCallback(BMP180_Handle_t *hbmp180);
void BMP180_I2C_RxCpltCallback(BMP180_Handle_t *hbmp180);

#endif
