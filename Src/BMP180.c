#include <BMP180.h>

BMP180_EEPROM _bmp180_calib;
uint8_t rawData[2];
volatile uint8_t rawDataReady = 0;
volatile int32_t UD;
uint8_t oss = 0;
volatile uint8_t wait = 0;
volatile uint8_t rcv = 0;
int32_t b5;
uint8_t readAddr[] = {0xF6};
const uint8_t msgOK[] = "BMP180: INIT DONE!\r\n";
const uint8_t msgER[] = "BMP180: INIT ERROR \r\n";
void BMP180_init(){
	uint8_t calib_buff[22];
	uint8_t i2cRxBuffer[1] = {0};
	HAL_I2C_Mem_Read(&hi2c3, BMP180ReadAddr, 0xD0, 1, i2cRxBuffer, 1, 100);
	if(i2cRxBuffer[0] == 0x55){
		print2sh("BMP180: INIT DONE!\r\n");
	}
	else print2sh("BMP180: INIT ERROR \r\n");

	HAL_I2C_Mem_Read(&hi2c3, BMP180ReadAddr, 0xAA, 1, calib_buff, 22, 1000);
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
}

void readRawData(uint8_t mode){
	uint8_t tx[2];
	tx[0] = 0xF4;	// ctrl reg
	tx[1] = mode ? 0x34+(oss << 6) : 0x2E;	// read press or temp
	wait = 1;
	HAL_I2C_Master_Transmit_IT(&hi2c3, BMP180WrAddr, tx, 2);
}

int32_t calcTemp(int32_t UT){
	int32_t x1 = (UT - _bmp180_calib.AC6) * _bmp180_calib.AC5 / (1 << 15);
	int32_t x2 = (_bmp180_calib.MC * (1 << 11)) / (x1 + _bmp180_calib.MD);
	b5 = x1 + x2;
	return (b5+8)/(1<<4);
}

int32_t calcPres(int32_t UP){
	int32_t b6 = b5-4000;
	int32_t x1 = (_bmp180_calib.B2*(b6*b6>>12))>>11;
	int32_t x2 = _bmp180_calib.AC2*b6 >> 11;
	int32_t x3 = x1+x2;
	int32_t b3 = (((_bmp180_calib.AC1*4+x3)<<oss)+2)>>2;
	x1 = _bmp180_calib.AC3*b6>>13;
	x2 = (_bmp180_calib.B1*(b6*b6>>12))>>16;
	x3 = ((x1+x2)+2)>>2;
	unsigned long b4 = _bmp180_calib.AC4*(unsigned long)(x3+32768)>>15;
	unsigned long b7 = ((unsigned long)UP-b3)*(50000>>oss);
	int32_t p;
	if(b7 < 0x80000000)	p = (b7*2)/b4;
	else p = (b7/b4)*2;
	x1 = (p >> 8)*(p >> 8);
	x1 = (x1*3038)>>16;
	x2 = (-7357*p)>>16;
	p=p+(x1+x2+3791)/16;
	return p;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if (htim->Instance == TIM3){
    	HAL_TIM_Base_Stop_IT(&htim3);
    	rcv = 1;
    	HAL_I2C_Master_Transmit_IT(&hi2c3, BMP180WrAddr, readAddr, 1);
    }
    if (htim->Instance == TIM4){
    	HAL_TIM_Base_Stop_IT(&htim4);
       	read = 1;
       }
}


void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c){
    if (hi2c->Instance == I2C3){
    	if(rcv == 1){
        	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    		HAL_I2C_Master_Receive_IT(&hi2c3, BMP180ReadAddr, rawData, 2);
    		rcv = 0;
    	}
    	if(wait == 1){
    		__HAL_TIM_SET_COUNTER(&htim3, 5000);
    		HAL_TIM_Base_Start_IT(&htim3);
    		wait = 0;
    	}

    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c){
    if (hi2c->Instance == I2C3){
    	UD = (rawData[0]) << 8 | rawData[1];
    	rawDataReady = 1;
    }
}
