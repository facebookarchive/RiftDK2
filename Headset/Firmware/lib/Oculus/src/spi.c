/************************************************************************************

Filename    :   spi.c
Content     :   SPI wrapper
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "spi.h"
#include <stdbool.h>
#include "gpio.h"

// To make sure we never get stuck forever
#define SPI_TIMEOUT  (0xFFFF)

static bool spi_wait_write(spi_p spi)
{
    uint32_t timeout = SPI_TIMEOUT;

    while (SPI_I2S_GetFlagStatus(spi->spi_port, SPI_I2S_FLAG_TXE) == RESET) {
        if (!timeout--) {
            // unselect the slave before quitting if we time out
            GPIO_SetBits(spi->ss_port, spi->ss_pin);
            return 0;
        }
    }

    return 1;
}

static bool spi_wait_read(spi_p spi)
{
    uint32_t timeout = SPI_TIMEOUT;

    while (SPI_I2S_GetFlagStatus(spi->spi_port, SPI_I2S_FLAG_RXNE) == RESET) {
        if (!timeout--) {
            GPIO_SetBits(spi->ss_port, spi->ss_pin);
            return 0;
        }
    }

    return 1;
}

static bool spi_wait_complete(spi_p spi)
{
    uint32_t timeout = SPI_TIMEOUT;

    while (SPI_I2S_GetFlagStatus(spi->spi_port, SPI_I2S_FLAG_TXE) == RESET) {
        if (!timeout--) {
            // unselect the slave before quitting if we time out
            GPIO_SetBits(spi->ss_port, spi->ss_pin);
            return 0;
        }
    }

    timeout = SPI_TIMEOUT;
    while (SPI_I2S_GetFlagStatus(spi->spi_port, SPI_I2S_FLAG_BSY) == SET) {
        if (!timeout--) {
            // unselect the slave before quitting if we time out
            GPIO_SetBits(spi->ss_port, spi->ss_pin);
            return 0;
        }
    }

    return 1;
}

static void spi_wait_dma(spi_p spi)
{
    uint32_t timeout = SPI_TIMEOUT;
    
    while (DMA_GetFlagStatus(spi->dma_tx_flag) == RESET) {
        if (!timeout--) {
            break;
        }
    }
    
    if (spi->dma_rx_flag) {
        while (DMA_GetFlagStatus(spi->dma_rx_flag) == RESET) {
            if (!timeout--) {
                break;
            }
        }
    }
    
    // We also need to wait for the SPI peripheral to stop before moving on
    spi_wait_complete(spi);
}

void spi_init(spi_p spi, uint32_t speed, bool use_dma)
{
    // Configure the clocks
#ifdef STM32F0XX
    if (IS_SPI_1_PERIPH(spi->spi_port)) {
        RCC_APB2PeriphClockCmd(spi->rcc, ENABLE);
    } else {
        RCC_APB1PeriphClockCmd(spi->rcc, ENABLE);
    }
#elif defined(STM32L1XX)
    if (IS_SPI_23_PERIPH(spi->spi_port)) {
        RCC_APB1PeriphClockCmd(spi->rcc, ENABLE);
    } else {
        RCC_APB2PeriphClockCmd(spi->rcc, ENABLE);
    }
#endif /* STM32F0XX */

    // Initialize AF for the pins being used
    GPIO_PinAFConfig(spi->gpio_port, spi->sck_source, spi->af);
    if (spi->sdo_pin)
        GPIO_PinAFConfig(spi->gpio_port, spi->sdo_source, spi->af);
    if (spi->sdi_pin)
        GPIO_PinAFConfig(spi->gpio_port, spi->sdi_source, spi->af);

    // Configure out pins for SPI Master
    gpio_config(spi->gpio_port, spi->sck_pin, GPIO_AF);
    if (spi->sdo_pin)
        gpio_config(spi->gpio_port, spi->sdo_pin, GPIO_AF);
    if (spi->sdi_pin)
        gpio_config(spi->gpio_port, spi->sdi_pin, GPIO_AF);

    gpio_config(spi->ss_port, spi->ss_pin, GPIO_PP);
    // Don't select the slave to start with
    GPIO_SetBits(spi->ss_port, spi->ss_pin);

    spi_reconfigure(spi, speed);
    
    // Start configuration of DMA only we use it.  The actual init will
    // occur in the write
    if (use_dma) {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
        
        DMA_DeInit(spi->dma_tx_channel);
        // Set the parameters that don't change for Tx to start
        spi->dma_config.DMA_PeripheralBaseAddr = (uint32_t)&(spi->spi_port->DR);
        spi->dma_config.DMA_DIR = DMA_DIR_PeripheralDST;
        spi->dma_config.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        spi->dma_config.DMA_MemoryInc = DMA_MemoryInc_Enable;
        spi->dma_config.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        spi->dma_config.DMA_MemoryDataSize =  DMA_MemoryDataSize_Byte;
        spi->dma_config.DMA_Mode = DMA_Mode_Normal;
        spi->dma_config.DMA_Priority = DMA_Priority_High;
        spi->dma_config.DMA_M2M = DMA_M2M_Disable;
        
        SPI_I2S_DMACmd(spi->spi_port, SPI_I2S_DMAReq_Tx, ENABLE);
        
        // Enable Rx DMA if we are also getting data back from the slave
        if (spi->dma_rx_channel) {
            SPI_I2S_DMACmd(spi->spi_port, SPI_I2S_DMAReq_Rx, ENABLE);
        }
    }
}

void spi_reconfigure(spi_p spi, uint32_t speed)
{
    SPI_Cmd(spi->spi_port, DISABLE);
    SPI_InitTypeDef   SPI_InitStructure;
    // Set Rx only, Tx only, or Rx/Tx based on pin definitions
    if (spi->sdi_pin && spi->sdo_pin) {
        SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    } else if (spi->sdi_pin) {
        SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
    } else if (spi->sdo_pin) {
        SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Rx;
    }
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = speed;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(spi->spi_port, &SPI_InitStructure);
#ifdef STM32F0XX
    SPI_RxFIFOThresholdConfig(spi->spi_port, SPI_RxFIFOThreshold_QF);
#endif /* STM32F0XX */
    SPI_Cmd(spi->spi_port, ENABLE);
}

uint16_t spi_transfer(spi_p spi, const uint8_t *buf, uint8_t *rx_buf, uint16_t len, bool use_dma)
{
    uint16_t pos;
    GPIO_ResetBits(spi->ss_port, spi->ss_pin);

    if (use_dma) {
        // Configure the DMA Tx
        spi->dma_config.DMA_BufferSize = len;
        spi->dma_config.DMA_MemoryBaseAddr = (uint32_t)buf;
        DMA_Init(spi->dma_tx_channel, &spi->dma_config);
        DMA_ClearFlag(spi->dma_tx_flag);
        
        // Followed by the DMA Rx if we use it
        if (spi->dma_rx_channel) {
            spi->dma_config.DMA_DIR = DMA_DIR_PeripheralSRC;
            spi->dma_config.DMA_MemoryBaseAddr = (uint32_t)rx_buf;
            
            DMA_Init(spi->dma_rx_channel, &spi->dma_config);
            DMA_ClearFlag(spi->dma_rx_flag);
            DMA_Cmd(spi->dma_rx_channel, ENABLE);
            
            // Set the config back for Tx
            spi->dma_config.DMA_DIR = DMA_DIR_PeripheralDST;
        }
        
        // Then kick off the transfer
        DMA_Cmd(spi->dma_tx_channel, ENABLE);
        
        // Wait for the transfer to complete or timeout
        spi_wait_dma(spi);
        
        // Shut off the DMA when done to prepare for next time
        DMA_Cmd(spi->dma_tx_channel, DISABLE);
        if (spi->dma_rx_channel)
            DMA_Cmd(spi->dma_rx_channel, DISABLE);
        
        pos = len;
    } else {
        for (pos = 0; pos < len; pos++) {
#ifdef STM32F0XX
            if (!spi_wait_write(spi)) return pos;
            SPI_SendData8(spi->spi_port, buf[pos]);
            if (spi->sdo_pin) {
                if (!spi_wait_read(spi)) return pos;
                rx_buf[pos] = SPI_ReceiveData8(spi->spi_port);
            }
#elif defined(STM32L1XX)
            if (!spi_wait_write(spi)) return pos;
            SPI_I2S_SendData(spi->spi_port, buf[pos]);
            if (spi->sdo_pin) {
                if (!spi_wait_read(spi)) return pos;
                rx_buf[pos] = SPI_I2S_ReceiveData(spi->spi_port);
            }
#endif /* STM32F0XX */
        }

        if (!spi_wait_complete(spi)) return pos;
    }

    GPIO_SetBits(spi->ss_port, spi->ss_pin);

    // Return the number of bytes successfully written/read
    return pos;
}
