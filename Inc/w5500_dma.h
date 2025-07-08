/*
 * w5500_dma.h
 *
 *  Created on: Jul 7, 2025
 *      Author: root
 */

#ifndef INC_W5500_DMA_H_
#define INC_W5500_DMA_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "uartCom.h"
#include <string.h>

#define SPI_TIMEOUT 5000

#define STANDART_TCP 0x01

// W5500 Register Addresses
#define W5500_COMMON_REG_BASE   0x0000
#define W5500_SOCKET_REG_BASE   0x0008
#define W5500_TX_BUF_BASE       0x0010
#define W5500_RX_BUF_BASE       0x0018

// Common Register Offsets
#define MR          0x0000  // Mode Register
#define GAR         0x0001  // Gateway Address Register
#define SUBR        0x0005  // Subnet Mask Register
#define SHAR        0x0009  // Source Hardware Address Register
#define SIPR        0x000F  // Source IP Address Register
#define INTLEVEL    0x0013  // Interrupt Low Level Timer Register
#define IR          0x0015  // Interrupt Register
#define IMR         0x0016  // Interrupt Mask Register
#define SIR         0x0017  // Socket Interrupt Register
#define SIMR        0x0018  // Socket Interrupt Mask Register
#define RTR         0x0019  // Retry Time Register
#define RCR         0x001B  // Retry Count Register
#define PTIMER      0x001C  // PPP LCP Request Timer Register
#define PMAGIC      0x001D  // PPP LCP Magic Number Register
#define PHAR        0x001E  // PPP Destination MAC Address Register
#define PSID        0x0024  // PPP Session Identification Register
#define PMRU        0x0026  // PPP Maximum Receive Unit Register
#define UIPR        0x0028  // Unreachable IP Address Register
#define UPORTR      0x002C  // Unreachable Port Register
#define PHYCFGR     0x002E  // PHY Configuration Register
#define VERSIONR    0x0039  // Chip Version Register

// Socket Register Offsets (relative to socket base)
#define Sn_MR       0x0000  // Socket Mode Register
#define Sn_CR       0x0001  // Socket Command Register
#define Sn_IR       0x0002  // Socket Interrupt Register
#define Sn_SR       0x0003  // Socket Status Register
#define Sn_PORT     0x0004  // Socket Source Port Register
#define Sn_DHAR     0x0006  // Socket Destination Hardware Address Register
#define Sn_DIPR     0x000C  // Socket Destination IP Address Register
#define Sn_DPORT    0x0010  // Socket Destination Port Register
#define Sn_MSSR     0x0012  // Socket Maximum Segment Size Register
#define Sn_TOS      0x0015  // Socket Type of Service Register
#define Sn_TTL      0x0016  // Socket Time to Live Register
#define Sn_RXBUF_SIZE 0x001E  // Socket Receive Buffer Size Register
#define Sn_TXBUF_SIZE 0x001F  // Socket Transmit Buffer Size Register
#define Sn_TX_FSR   0x0020  // Socket TX Free Size Register
#define Sn_TX_RD    0x0022  // Socket TX Read Pointer Register
#define Sn_TX_WR    0x0024  // Socket TX Write Pointer Register
#define Sn_RX_RSR   0x0026  // Socket RX Received Size Register
#define Sn_RX_RD    0x0028  // Socket RX Read Pointer Register
#define Sn_RX_WR    0x002A  // Socket RX Write Pointer Register
#define Sn_IMR      0x002C  // Socket Interrupt Mask Register
#define Sn_FRAG     0x002D  // Socket Fragment Offset in IP Header Register
#define Sn_KPALVTR  0x002F  // Socket Keep Alive Timer Register

// Socket Commands
#define SOCK_OPEN       0x01
#define SOCK_LISTEN     0x02
#define SOCK_CONNECT    0x04
#define SOCK_DISCON     0x08
#define SOCK_CLOSE      0x10
#define SOCK_SEND       0x20
#define SOCK_SEND_MAC   0x21
#define SOCK_SEND_KEEP  0x22
#define SOCK_RECV       0x40

// Socket Mode Register Values
#define Sn_MR_CLOSE     0x00
#define Sn_MR_TCP       0x01
#define Sn_MR_UDP       0x02
#define Sn_MR_IPRAW     0x03
#define Sn_MR_MACRAW    0x04
#define Sn_MR_PPPOE     0x05
#define Sn_MR_ND        0x20
#define Sn_MR_MULTI     0x80

// Socket Status Register Values
#define SOCK_CLOSED_Status     0x00
#define SOCK_INIT_Status       0x13
#define SOCK_LISTEN_Status     0x14
#define SOCK_SYNSENT_Status    0x15
#define SOCK_SYNRECV_Status    0x16
#define SOCK_ESTABLISHED_Status 0x17
#define SOCK_FIN_WAIT_Status   0x18
#define SOCK_CLOSING_Status    0x1A
#define SOCK_TIME_WAIT_Status  0x1B
#define SOCK_CLOSE_WAIT_Status 0x1C
#define SOCK_LAST_ACK_Status   0x1D
#define SOCK_UDP_Status        0x22
#define SOCK_IPRAW_Status      0x32
#define SOCK_MACRAW_Status     0x42
#define SOCK_PPPOE_Status      0x5F

// DMA Transfer States
typedef enum {
    W5500_DMA_IDLE,
    W5500_DMA_BUSY,
    W5500_DMA_COMPLETE,
    W5500_DMA_ERROR
} W5500_DMA_State_t;

// W5500 Configuration Structure
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    GPIO_TypeDef *reset_port;
    uint16_t reset_pin;
    uint8_t mac[6];
    uint8_t ip[4];
    uint8_t subnet[4];
    uint8_t gateway[4];
} W5500_Config_t;

// W5500 Handle Structure
typedef struct {
    W5500_Config_t config;
    W5500_DMA_State_t dma_state;
    uint8_t *tx_buffer;
    uint8_t *rx_buffer;
    uint16_t buffer_size;
    void (*transfer_complete_callback)(void);
    void (*transfer_error_callback)(void);
} W5500_Handle_t;

typedef struct{
	uint8_t Sn;
	uint8_t protocol;
	uint16_t port;
	uint8_t rxbuffer;	// in kb
	uint8_t txbuffer; 	// in kb
	uint8_t tos;		// type of service 0 for standart operation, reference: http://www.iana.org/assignments/ip-parameters/ip-parameters.xhtml
} W5500_Sock_Config_t;
// Function Prototypes
HAL_StatusTypeDef W5500_Init(W5500_Handle_t *hw5500, W5500_Config_t *config);
HAL_StatusTypeDef W5500_Reset(W5500_Handle_t *hw5500);
HAL_StatusTypeDef W5500_ReadReg_DMA(W5500_Handle_t *hw5500, uint32_t addr, uint8_t *data, uint16_t len);
HAL_StatusTypeDef W5500_WriteReg_DMA(W5500_Handle_t *hw5500, uint32_t addr, uint8_t *data, uint16_t len);
HAL_StatusTypeDef W5500_ReadReg_Blocking(W5500_Handle_t *hw5500, uint32_t addr, uint8_t bsb, uint8_t *data, uint16_t len);
HAL_StatusTypeDef W5500_WriteReg_Blocking(W5500_Handle_t *hw5500, uint32_t addr, uint8_t bsb, uint8_t *data, uint16_t len);

// Socket Management
HAL_StatusTypeDef W5500_Socket_Open(W5500_Handle_t *hw5500, W5500_Sock_Config_t *sock_config);
HAL_StatusTypeDef W5500_Socket_Close(W5500_Handle_t *hw5500, uint8_t sock);
HAL_StatusTypeDef W5500_Socket_Listen(W5500_Handle_t *hw5500, W5500_Sock_Config_t *sock_config);
uint8_t W5500_Socket_GetStatus(W5500_Handle_t *hw5500, uint8_t sock);
uint16_t W5500_Socket_GetRxSize(W5500_Handle_t *hw5500, uint8_t sock);
uint16_t W5500_Socket_GetTxFreeSize(W5500_Handle_t *hw5500, uint8_t sock);

// Data Transfer
HAL_StatusTypeDef W5500_Socket_Send(W5500_Handle_t *hw5500, uint8_t sock, uint8_t *data, uint16_t len);
HAL_StatusTypeDef W5500_Socket_Recv(W5500_Handle_t *hw5500, uint8_t sock, uint8_t *data, uint16_t len);

// Interrupt Callbacks
void W5500_SPI_TxRxCpltCallback(W5500_Handle_t *hw5500);
void W5500_SPI_ErrorCallback(W5500_Handle_t *hw5500);

// Utility Functions
uint32_t W5500_GetSocketRegAddr(uint8_t sock, uint16_t reg);
uint32_t W5500_GetSocketTxBufAddr(uint8_t sock);
uint32_t W5500_GetSocketRxBufAddr(uint8_t sock);


#endif /* INC_W5500_DMA_H_ */
