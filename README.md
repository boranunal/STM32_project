# STM32 BMP180 Weather Server (W5500)

Reads temperature and pressure using BMP180 and serves it via HTTP on STM32 using the W5500 Ethernet chip.
Developed using HAL libraries and CubeIDE.

## Hardware
- STM32F401RE - Nucleo Board
- BMP180 (I2C)
- W5500 (SPI)

## File Structure and Modules

- `main.c`: Entry point. Initializes peripherals and runs the main loop. (Using main.c directly is not recommended. Generate code using `.ioc` project configuration file.)
- `BMP180.c/h`: For communication with the BMP180 sensor over I2C using interrupts. Uses a non-blocking state machine.
- `w5500_dma.c/h`: Custom library for W5500. SPI communication using DMA or blocking functions(for initialization and control) and register-level control functions for the W5500 Ethernet chip. The controller is only used as a TCP server in a private network, other functionality might be missing.
- `httpserver.c/h`: Basic HTML template and HTTP response handling. 
- `uartCom.c/h`: Print to terminal using UART for debugging. (*Optional; not required for core functionality)

## Important Handle types

// BMP180 Handle type
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
} BMP180\_Handle\_t;

Call bmp180.c functions using BMP180\_Handle\_t. Initialize the htim, hi2c, the array raw\_data, and its ready.

// W5500 Configuration Structure					// example
typedef struct {							W5500_Config_t w5500_net_config = {
    SPI_HandleTypeDef *hspi;									.hspi = &hspi2,
    GPIO_TypeDef *cs_port;									.cs_port = W5500_CS_GPIO_Port,
    uint16_t cs_pin;										.cs_pin = W5500_CS_Pin,
    GPIO_TypeDef *reset_port;									.reset_port = W5500_RESET_GPIO_Port,
    uint16_t reset_pin;										.reset_pin = W5500_RESET_Pin,
    uint8_t mac[6];										.mac = {0x02, 0x00, 0x00, 0x00, 0x00, 0x07},
    uint8_t ip[4];										.ip = {192, 168, 1, 101},
    uint8_t subnet[4];										.subnet = {255, 255, 255, 0},
    uint8_t gateway[4];										.gateway = {0, 0, 0, 0}
} W5500_Config_t;										};

Ensure the IP and MAC addresses do not conflict with other devices on your network. After setup, call W5500_Init()

typedef struct{
	char *req;
	W5500_Sock_Interrupt_Status_t it;
	uint16_t rxrd_next;
	uint16_t txwr_next;
	uint8_t Sn;
	uint8_t protocol;
	uint16_t port;
	uint8_t rxbuf_size;	// in kb
	uint8_t txbuf_size; 	// in kb
	uint8_t tos;		// type of service 0 for standart operation, reference: http://www.iana.org/assignments/ip-parameters/ip-parameters.xhtml
} W5500_Sock_Handle_t;

Make sure to initialize before calling W5500_Socket_Open() function. W5500_Sock_Interrupt_Status_t is used to keep track of the socket interrupts and the state machine.

## BMP180 State Machine

stateDiagram-v2
    [*] --> S0_INIT : BMP180_init()
    S0_INIT --> S1_Transmit_0x2E_to_read_temperature : Trigger Sensor Read
    S1_Transmit_0x2E_to_read_temperature --> S2_Start_timer : I2C Tx Interrupt
    S2_Start_timer --> S3_Transmit_read_address : Timer Interrupt
    S3_Transmit_read_address --> S4_Start_receiving : I2C Tx Interrupt
    S4_Start_receiving --> Calculate_Temperature : I2C Rx Interrupt
    S4_Start_receiving --> S5_Transmit_0x34_to_read_pressure : I2C Rx Interrupt
    S5_Transmit_0x34_to_read_pressure --> S6_Start_timer : I2C Tx Interrupt
    S6_Start_timer --> S7_Transmit_read_address : Timer Interrupt
    S7_Transmit_read_address --> S8_Start_receiving : I2C Tx Interrupt
    S8_Start_receiving --> S0_INIT
    S8_Start_receiving --> Calculate_Pressure : I2C Rx Interrupt

## W5500 State Machine

stateDiagram-v2
    [*] --> S0_INIT : Socket_Open()
    S0_INIT --> S1_Listen : 
    S1_Listen --> S2_Established : W5500 CON Interrupt
    S2_Established --> S3_Start_SPI_Rx : W5500 RECV Interrupt
    S3_Start_SPI_Rx --> S4_Finish_SPI_Rx_Start_SPI_Tx : SPI Interrupt
    S4_Finish_SPI_Rx_Start_SPI_Tx --> S5_Finish_SPI_Tx : SPI Interrupt
    S5_Finish_SPI_Tx --> S0_INIT : I2C Rx Interrupt



## Known Issues

- If connection somehow dies in the established state, socket gets stuck in that state and cannot accept any more connections. 
