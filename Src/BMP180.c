#include <BMP180.h>
#include <string.h>

// If this variable oss is changed pressure reading scheme changes.
// Also data should be waited accordingly. Refer to the datasheet.
uint8_t oss = 0;
BMP180_EEPROM _bmp180_calib;
/*
 * Read device id it should read 0x55.
 * If not read function is not working check connections.
 * Then read calibration data.
 */
void BMP180_init(BMP180_Handle_t *hbmp180){
	uint8_t calib_buff[22];
	uint8_t i2cRxBuffer[1] = {0};
	uint8_t rst_sq[] = {0xE0, 0xB6};
	HAL_I2C_Master_Transmit(hbmp180->hi2c, BMP180WrAddr, rst_sq, 2, 200);
	HAL_Delay(100);
	HAL_I2C_Mem_Read(hbmp180->hi2c, BMP180ReadAddr, 0xD0, 1, i2cRxBuffer, 1, 100);
	if(i2cRxBuffer[0] == 0x55){
		print2sh("BMP180: INIT DONE!\r\n");
	}
	else print2sh("BMP180: INIT ERROR \r\n");


	hbmp180->ut = 0;
	hbmp180->up = 0;
	hbmp180->b5 = 0;
	hbmp180->temp_ready = 0;
	hbmp180->pres_ready = 0;

	HAL_I2C_Mem_Read(hbmp180->hi2c, BMP180ReadAddr, 0xAA, 1, calib_buff, 22, 1000);
	_bmp180_calib.AC1 = calib_buff[0] << 8 | calib_buff[1];
	_bmp180_calib.AC2 = calib_buff[2] << 8 | calib_buff[3];
	_bmp180_calib.AC3 = calib_buff[4] << 8 | calib_buff[5];
	_bmp180_calib.AC4 = calib_buff[6] << 8 | calib_buff[7];
	_bmp180_calib.AC5 = calib_buff[8] << 8 | calib_buff[9];
	_bmp180_calib.AC6 = calib_buff[10] << 8 | calib_buff[11];
	_bmp180_calib.B1 = calib_buff[12] << 8 | calib_buff[13];
	_bmp180_calib.B2 = calib_buff[14] << 8 | calib_buff[15];
	_bmp180_calib.MB = calib_buff[16] << 8 | calib_buff[17];
	_bmp180_calib.MC = calib_buff[18] << 8 | calib_buff[19];
	_bmp180_calib.MD = calib_buff[20] << 8 | calib_buff[21];
	memcpy(&hbmp180->calib, &_bmp180_calib, sizeof(BMP180_EEPROM));
}
/*
 * Request data to read.
 * Write the control register,
 * according to the value BMP180 will read temperature or pressure.
 */
void BMP180_readRawData(BMP180_Handle_t *hbmp180){
	uint8_t tx[2];
	tx[0] = 0xF4;	// ctrl reg
	if(hbmp180->state == 1)	tx[1] = 0x2E;
	else if(hbmp180->state == 5) tx[1] = 0x34+(oss << 6);
	HAL_I2C_Master_Transmit_IT(hbmp180->hi2c, BMP180WrAddr, tx, 2);

}
/*
 * Start timer and wait 5 msec for data to be ready.
 * After writing the control register it takes some time for data to be ready.
 */
void BMP180_waitData(BMP180_Handle_t *hbmp180){
	__HAL_TIM_SET_COUNTER(hbmp180->htim, 5000);
	HAL_TIM_Base_Start_IT(hbmp180->htim);
}
/*
 * When timer stops this function can be called to start reading data.
 * Transmit which address to read.
 */
void BMP180_dataReadyToGet(BMP180_Handle_t *hbmp180){
	HAL_TIM_Base_Stop_IT(hbmp180->htim);
	uint8_t readAddr[] = {0xF6};
	HAL_I2C_Master_Transmit_IT(hbmp180->hi2c, BMP180WrAddr, readAddr, 1);
}
/*
 * Start receiving data.
 * Handle the uncompensated data in I2C_RxCpltCallback function.
 */
void BMP180_getData(BMP180_Handle_t *hbmp180){
	HAL_I2C_Master_Receive_IT(hbmp180->hi2c, BMP180ReadAddr, hbmp180->raw_data, 2);
}
/*
 * Calculate temperature with uncompensated temperature.
 */
int32_t BMP180_calcTemp(BMP180_Handle_t *hbmp180){
	int32_t x1 = (hbmp180->ut - hbmp180->calib.AC6) * hbmp180->calib.AC5 / (1 << 15);
	int32_t x2 = (hbmp180->calib.MC * (1 << 11)) / (x1 + hbmp180->calib.MD);
	// b5 value is used to calculate pressure,
	// to calculate pressure first calculate temperature.
	hbmp180->b5 = x1 + x2;
	return (hbmp180->b5+8)/(1<<4);
}
/*
 * Calculate pressure with uncompansated pressure.
 */
int32_t BMP180_calcPres(BMP180_Handle_t *hbmp180){
	int32_t b6 = hbmp180->b5-4000;
	int32_t x1 = (hbmp180->calib.B2*(b6*b6>>12))>>11;
	int32_t x2 = hbmp180->calib.AC2*b6 >> 11;
	int32_t x3 = x1+x2;
	int32_t b3 = (((hbmp180->calib.AC1*4+x3)<<oss)+2)>>2;
	x1 = hbmp180->calib.AC3*b6>>13;
	x2 = (hbmp180->calib.B1*(b6*b6>>12))>>16;
	x3 = ((x1+x2)+2)>>2;
	unsigned long b4 = hbmp180->calib.AC4*(unsigned long)(x3+32768)>>15;
	unsigned long b7 = ((unsigned long)hbmp180->up-b3)*(50000>>oss);
	int32_t p;
	if(b7 < 0x80000000)	p = (b7*2)/b4;
	else p = (b7/b4)*2;
	x1 = (p >> 8)*(p >> 8);
	x1 = (x1*3038)>>16;
	x2 = (-7357*p)>>16;
	p=p+(x1+x2+3791)/16;
	return p;
}

/*
 *  Function to be called when I2C transmission is completed.
 */
void BMP180_I2C_TxCpltCallback(BMP180_Handle_t *hbmp180){
	if(hbmp180->state == 1){
		hbmp180->state = 0x22;
	}
	else if(hbmp180->state == 3){
		hbmp180->state = 0x24;
	}
	else if(hbmp180->state == 5){
		hbmp180->state = 0x26;
	}
	else if(hbmp180->state == 7){
		hbmp180->state = 0x28;
	}

}
/*
 * Function to be called when I2C receive completed.
 */
void BMP180_I2C_RxCpltCallback(BMP180_Handle_t *hbmp180){
	if(hbmp180->temp_ready == 2){
		hbmp180->ut = (((hbmp180->raw_data[0]) << 8) & 0xFF00) | (hbmp180->raw_data[1] & 0xFF);
		hbmp180->temp_ready = 1;
		hbmp180->state = 0x25;
	}
	else if(hbmp180->pres_ready == 2){
		hbmp180->up = (((hbmp180->raw_data[0]) << 8) & 0xFF00) | (hbmp180->raw_data[1] & 0xFF);
		hbmp180->pres_ready = 1;
	}
}



