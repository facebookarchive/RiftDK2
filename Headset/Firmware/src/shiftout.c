/************************************************************************************

Filename    :   shiftout.c
Content     :   Shift register interface
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "shiftout.h"
#include "hw_config.h"
#include "gpio.h"
#include "spi.h"
#include <stdbool.h>
#include <stdlib.h>

typedef struct shiftout_struct {
    spi_t spi;
    GPIO_TypeDef *oe_port;
    uint16_t oe_pin;
} shiftout_t, *shiftout_p;

shiftout_t g_shiftout = {{0}};

void shiftout_init(void)
{
    // Set the pins and ports for the shift register interface
    g_shiftout.spi.spi_port = SR_SPI;
    g_shiftout.spi.gpio_port = SR_PORT;
    g_shiftout.spi.ss_port = SR_RCLK_PORT;
    g_shiftout.spi.sck_pin = SR_SPI_SCK_PIN;
    // Use DMA for this
    g_shiftout.spi.dma_tx_channel = SR_SPI_DMA;
    g_shiftout.spi.dma_tx_flag = SR_SPI_DMA_FLAG;
    // We're not reading data back from the slave
    g_shiftout.spi.sdo_pin = 0;
    g_shiftout.spi.sdi_pin = SR_SPI_SDI_PIN;
    g_shiftout.spi.ss_pin = SR_RCLK_PIN;
    g_shiftout.spi.sck_source = SR_SPI_SCK_SOURCE;
    g_shiftout.spi.sdo_source = 0;
    g_shiftout.spi.sdi_source = SR_SPI_SDI_SOURCE;
    g_shiftout.spi.rcc = SR_SPI_RCC;
    g_shiftout.spi.af = SR_SPI_AF;
    g_shiftout.oe_port = SR_OE_PORT;
    g_shiftout.oe_pin = SR_OE_PIN;

    // Initialize at 16 MHz
    spi_init(&g_shiftout.spi, SPI_BaudRatePrescaler_2, 1);
}

void shiftout_shift(const uint8_t *bytes, uint8_t num_bytes)
{
    // We can use a NULL on the Rx buffer since the spi was initialized without
    // an SDO pin
    spi_transfer(&g_shiftout.spi, bytes, NULL, num_bytes, 1);
}
