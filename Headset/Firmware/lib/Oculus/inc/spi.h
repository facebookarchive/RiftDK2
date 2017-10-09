/************************************************************************************

Filename    :   spi.h
Content     :   SPI wrapper
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _SPI_H_
#define _SPI_H_

#include <stdint.h>
#include <stdbool.h>
#ifdef STM32F0XX
#include "stm32f0xx.h"
#elif defined(STM32L1XX)
#include "stm32l1xx.h"
#endif /* STM32F0XX */

typedef struct spi_struct {
    SPI_TypeDef *spi_port;
    GPIO_TypeDef *gpio_port;
    GPIO_TypeDef *ss_port;
    DMA_Channel_TypeDef *dma_tx_channel; 
    DMA_Channel_TypeDef *dma_rx_channel;
    DMA_InitTypeDef dma_config;
    uint32_t dma_tx_flag;
    uint32_t dma_rx_flag;
    uint16_t sck_pin;
    uint16_t sdo_pin;
    uint16_t sdi_pin;
    uint16_t ss_pin;
    uint8_t sck_source;
    uint8_t sdo_source;
    uint8_t sdi_source;
    uint8_t af;
    uint32_t rcc;
} spi_t, *spi_p;

void spi_init(spi_p spi, uint32_t speed, bool use_dma);

void spi_reconfigure(spi_p spi, uint32_t speed);

uint16_t spi_transfer(spi_p spi, const uint8_t *buf, uint8_t *rx_buf, uint16_t len, bool use_dma);

#endif /* _SPI_H_ */
