/*
 * w5500_dma.c
 *
 *  Created on: Jul 7, 2025
 *      Author: root
 */
#include "w5500_dma.h"
#include <stdio.h>

/*
 * Static variables for DMA operations.
 * Buffer should be large enough for a
 * single transaction. about 1500 bytes
 * for an Ethernet frame.
 */

static uint8_t spi_tx_buffer[2048];
static uint8_t spi_rx_buffer[2048];

// CS Control Macros
#define W5500_CS_LOW(hw5500)  HAL_GPIO_WritePin(hw5500->config.cs_port, hw5500->config.cs_pin, GPIO_PIN_RESET)
#define W5500_CS_HIGH(hw5500) HAL_GPIO_WritePin(hw5500->config.cs_port, hw5500->config.cs_pin, GPIO_PIN_SET)
#define BSB_GEN(sn, sr)	(((sn<<2)|(sr)) & 0x1F)
/**
 * Hardware Reset, nRST PIN must be held LOW at least for 0.5 msec
 */
HAL_StatusTypeDef W5500_Reset(W5500_Handle_t *hw5500)
{
    HAL_GPIO_WritePin(hw5500->config.reset_port, hw5500->config.reset_pin, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(hw5500->config.reset_port, hw5500->config.reset_pin, GPIO_PIN_SET);
    HAL_Delay(5);
    return HAL_OK;
}

/**
 * @brief Initialize W5500 with network configuration
 */
HAL_StatusTypeDef W5500_Init(W5500_Handle_t *hw5500, W5500_Config_t *config)
{
    // Copy configuration
    memcpy(&hw5500->config, config, sizeof(W5500_Config_t));

    // Initialize hw5500 parameters
    hw5500->dma_state = W5500_DMA_IDLE;
    hw5500->tx_buffer = spi_tx_buffer;
    hw5500->rx_buffer = spi_rx_buffer;
    hw5500->buffer_size = sizeof(spi_tx_buffer);

    // Reset W5500
    HAL_StatusTypeDef status = W5500_Reset(hw5500);
    if (status != HAL_OK) return status;

    // Wait for reset to complete, could be smaller value
    HAL_Delay(50);

    // Software Reset
    uint8_t rst_cmd = 0x80;
    status = W5500_WriteReg_Blocking(hw5500, MR, BSB_GEN(0,0), &rst_cmd, 1);
    if (status != HAL_OK) return status;

    print2sh("W5500: RESET DONE!\r\n");

    // Chip version check
    uint8_t version;
    status = W5500_ReadReg_Blocking(hw5500, VERSIONR, BSB_GEN(0,0), &version, 1);
    if (status != HAL_OK || version != 0x04) return HAL_ERROR;
    uint8_t print_buffer[40];
    snprintf(print_buffer, sizeof(print_buffer), "W5500: Version: 0x%02X\r\n", version);
    print2sh(print_buffer);

    // Configure network settings
    status = W5500_WriteReg_Blocking(hw5500, GAR, BSB_GEN(0,0), hw5500->config.gateway, 4);
    if (status != HAL_OK) return status;

    status = W5500_WriteReg_Blocking(hw5500, SUBR, BSB_GEN(0,0), hw5500->config.subnet, 4);
    if (status != HAL_OK) return status;

    status = W5500_WriteReg_Blocking(hw5500, SHAR, BSB_GEN(0,0), hw5500->config.mac, 6);
    if (status != HAL_OK) return status;

    status = W5500_WriteReg_Blocking(hw5500, SIPR, BSB_GEN(0,0), hw5500->config.ip, 4);
    if (status != HAL_OK) return status;

    print2sh("W5500: INIT DONE!\r\n");
    return HAL_OK;
}

/**
 * Write register using blocking mode (for initialization)
 */
HAL_StatusTypeDef W5500_WriteReg_Blocking(W5500_Handle_t *hw5500, uint32_t addr, uint8_t bsb, uint8_t *data, uint16_t len)
{
    uint8_t tx_buf[len + 3];

    tx_buf[0] = (addr >> 8) & 0xFF;
    tx_buf[1] = addr & 0xFF;
    tx_buf[2] = (bsb << 3 | 0x4) & 0xFF;
    memcpy(&tx_buf[3], data, len);

    W5500_CS_LOW(hw5500);

    HAL_StatusTypeDef status = HAL_SPI_Transmit(hw5500->config.hspi, tx_buf, len + 3, SPI_TIMEOUT);

    W5500_CS_HIGH(hw5500);

    return status;
}

/**
 * Read register using blocking mode (for initialization)
 */
HAL_StatusTypeDef W5500_ReadReg_Blocking(W5500_Handle_t *hw5500, uint32_t addr, uint8_t bsb, uint8_t *data, uint16_t len)
{
    uint8_t tx_buf[3];

    tx_buf[0] = (addr >> 8) & 0xFF;
    tx_buf[1] = addr & 0xFF;
    tx_buf[2] = (bsb << 3) & 0xF8;

    W5500_CS_LOW(hw5500);

    HAL_StatusTypeDef status = HAL_SPI_Transmit(hw5500->config.hspi, tx_buf, 3, SPI_TIMEOUT);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(hw5500->config.hspi, data, len, SPI_TIMEOUT);
    }

    W5500_CS_HIGH(hw5500);

    return status;
}

HAL_StatusTypeDef W5500_Socket_Open(W5500_Handle_t *hw5500, W5500_Sock_Config_t *sock_config){

	if(sock_config->Sn >= 8) return HAL_ERROR;

	// Initialize the Sn_MR with the protocol
	HAL_StatusTypeDef status = W5500_WriteReg_Blocking(hw5500, Sn_MR, BSB_GEN(sock_config->Sn, 1), &sock_config->protocol, 1);
	if (status != HAL_OK) return status;

	// Write PORT number
	uint8_t port_bytes[2] = {(sock_config->port >> 8) & 0xFF, sock_config->port & 0xFF};
	status = W5500_WriteReg_Blocking(hw5500, Sn_PORT, BSB_GEN(sock_config->Sn, 1), port_bytes, 2);
	if (status != HAL_OK) return status;

	// Set RX buffer of socket
	status = W5500_WriteReg_Blocking(hw5500, Sn_RXBUF_SIZE, BSB_GEN(sock_config->Sn, 1), &sock_config->rxbuffer, 1);
	if (status != HAL_OK) return status;

	// Set TX buffer of socket
	status = W5500_WriteReg_Blocking(hw5500, Sn_TXBUF_SIZE, BSB_GEN(sock_config->Sn, 1), &sock_config->txbuffer, 1);
	if (status != HAL_OK) return status;

	// Set TOS
	status = W5500_WriteReg_Blocking(hw5500, Sn_TOS, BSB_GEN(sock_config->Sn, 1), &sock_config->tos, 1);
	if (status != HAL_OK) return status;

	// Open socket
	uint8_t cmd = SOCK_OPEN;
	status = W5500_WriteReg_Blocking(hw5500, Sn_CR, BSB_GEN(sock_config->Sn, 1), &cmd, 1);
	if (status != HAL_OK) return status;

	// Wait for it to open
	uint8_t timeout = 100;
	while (timeout--) {
		uint8_t cmd_status = 0xFF;
		W5500_ReadReg_Blocking(hw5500, Sn_SR,BSB_GEN(sock_config->Sn, 1), &cmd_status, 1);
		if (cmd_status == SOCK_INIT_Status) break;
		HAL_Delay(1);
	}

	if(timeout > 0)
		return HAL_OK;
	else
		return HAL_ERROR;
}

HAL_StatusTypeDef W5500_Socket_Listen(W5500_Handle_t *hw5500, W5500_Sock_Config_t *sock_config){
    if (sock_config->Sn >= 8) return HAL_ERROR;

    uint8_t cmd = SOCK_LISTEN;
    HAL_StatusTypeDef status = W5500_WriteReg_Blocking(hw5500, Sn_CR, BSB_GEN(sock_config->Sn, 1), &cmd, 1);
    if (status != HAL_OK) return status;

    // Wait for command completion
    uint8_t timeout = 100;
    while (timeout--) {
        uint8_t cmd_status = 0xFF;
        W5500_ReadReg_Blocking(hw5500, Sn_SR, BSB_GEN(sock_config->Sn, 1), &cmd_status, 1);
        if (cmd_status == 0) break;
        HAL_Delay(1);
    }

	if(timeout > 0)
		return HAL_OK;
	else
		return HAL_ERROR;
}

