/************************************************************************************

Filename    :   lis3mdl.c
Content     :   ST LIS3MDL Magnetometer interface
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "lis3mdl.h"
#include "lis3mdl_register_map.h"
#include "spi.h"
#include "hw_config.h"
#include "gpio.h"

// bit 7 for read, 6 for address incrementing
#define R(a) (0xC0 | a)

#define NUM_RANGES 4
static const uint16_t ranges[NUM_RANGES] = {4000, 8000, 12000, 16000};
static const uint8_t bitmasks[NUM_RANGES] = {GN_4, GN_8, GN_12, GN_16};
static const int32_t conversions[NUM_RANGES] = {GN_SCALE_4, GN_SCALE_8, GN_SCALE_12, GN_SCALE_16};

typedef struct lis3mdl_struct {
    spi_t spi;
    uint8_t range;
    bool suspended;
    volatile uint32_t data_ready;
    bool first_read;
} lis3mdl_t, *lis3mdl_p;

static lis3mdl_t g_lis3mdl = {{0}};

void lis3mdl_init(void)
{
    // Set up the data ready interrupt
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    gpio_config(MAG_DRDY_PORT, MAG_DRDY_PIN, GPIO_IN);

    SYSCFG_EXTILineConfig(MAG_DRDY_SOURCE_PORT, MAG_DRDY_SOURCE);

    EXTI_InitStructure.EXTI_Line = MAG_DRDY_LINE;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = MAG_DRDY_CHANNEL;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // The mag isn't timestamped anyway, so allow the interrupt to be a low
    // priority
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;
    NVIC_Init(&NVIC_InitStructure);
    
    // Set the pins and ports for the mag spi interface
    g_lis3mdl.spi.spi_port = MEMS_SPI_NUM;
    g_lis3mdl.spi.gpio_port = MEMS_PORT;
    g_lis3mdl.spi.ss_port = MAG_CS_PORT;
    g_lis3mdl.spi.sck_pin = MEMS_SCK_PIN;
    g_lis3mdl.spi.sdo_pin = MEMS_MISO_PIN;
    g_lis3mdl.spi.sdi_pin = MEMS_MOSI_PIN;
    g_lis3mdl.spi.ss_pin = MAG_CS_PIN;
    g_lis3mdl.spi.sck_source = MEMS_SCK_SOURCE;
    g_lis3mdl.spi.sdo_source = MEMS_MISO_SOURCE;
    g_lis3mdl.spi.sdi_source = MEMS_MOSI_SOURCE;
    g_lis3mdl.spi.rcc = MEMS_SPI_RCC;
    g_lis3mdl.spi.af = MEMS_SPI_AF;
    // Use DMA for Tx and Rx
    g_lis3mdl.spi.dma_rx_channel = MEMS_SPI_DMA_RX;
    g_lis3mdl.spi.dma_rx_flag = MEMS_SPI_DMA_RX_FLAG;
    g_lis3mdl.spi.dma_tx_channel = MEMS_SPI_DMA_TX;
    g_lis3mdl.spi.dma_tx_flag = MEMS_SPI_DMA_TX_FLAG;
    
    // Initialize at 8 MHz, below the part's 10 MHz limit
    spi_init(&g_lis3mdl.spi, MEMS_SPI_READ_SPEED, 1);

    uint8_t rx_buf[2];
    // Reset the part
    static const uint8_t ctrl2[] = {CTRL_REG2, 0x0C};
    spi_transfer(&g_lis3mdl.spi, ctrl2, rx_buf, sizeof(ctrl2), 0);
    
    // Enable "ultra-high performance" and 80 Hz
    static const uint8_t ctrl1[] = {CTRL_REG1, 0x7C};
    spi_transfer(&g_lis3mdl.spi, ctrl1, rx_buf, sizeof(ctrl1), 0);
    
    // Enable z-axis "ultra-high performance"
    static const uint8_t ctrl4[] = {CTRL_REG4, 0x0C};
    spi_transfer(&g_lis3mdl.spi, ctrl4, rx_buf, sizeof(ctrl4), 0);
    
    // Make multibyte sensor reads atomic
    static const uint8_t ctrl5[] = {CTRL_REG5, 0x40};
    spi_transfer(&g_lis3mdl.spi, ctrl5, rx_buf, sizeof(ctrl5), 0);
    
    // Start in power down mode
    lis3mdl_sleep(1);
}

void lis3mdl_sleep(bool sleep)
{
    uint8_t rx_buf[2];

    if (sleep) {
        g_lis3mdl.suspended = 1;
        // Go into power down mode
        static const uint8_t ctrl3[] = {CTRL_REG3, 0x03};
        spi_transfer(&g_lis3mdl.spi, ctrl3, rx_buf, sizeof(ctrl3), 0);
    } else {
        // Go into continuous conversion mode
        static const uint8_t ctrl3[] = {CTRL_REG3, 0x00};
        spi_transfer(&g_lis3mdl.spi, ctrl3, rx_buf, sizeof(ctrl3), 0);
        g_lis3mdl.suspended = 0;
        // The LIS3MDL is set to only interrupt after its registers have been
        // read, so after waking, we need to force the first read
        g_lis3mdl.first_read = 1;
    }
}

static inline int16_t convert_mag(int16_t mag)
{
    // Convert from the mag's native format to 10^-4 gauss
    return 10000 * (int32_t)mag / conversions[g_lis3mdl.range];
}

bool lis3mdl_read(lis3mdl_data_p data, bool raw)
{
    // Only read if we are forced to or if there is new data
    if ((!g_lis3mdl.data_ready && !g_lis3mdl.first_read) || g_lis3mdl.suspended)
        return 0;
    
    if (g_lis3mdl.data_ready)
        g_lis3mdl.data_ready--;
    
    static const uint8_t command[7] = {R(OUT_X_L)};
    uint8_t rx_buf[8];
    // Skip ahead in the read buf by a byte so we get favorable alignment
    // on the 16-bit copies
    if (spi_transfer(&g_lis3mdl.spi, command, rx_buf + 1, sizeof(command), 1) == sizeof(command)) {
        g_lis3mdl.first_read = 0;
     
        // Use OVR Headset coordinates from the lowest level
        // That is, Z-in, Y-up, X-right from the user with the headset on
        if (raw) {
            // Swap X and Y, negate all
            data->mag[1] = -*(int16_t *)(rx_buf + 2);
            data->mag[0] = -*(int16_t *)(rx_buf + 4);
            data->mag[2] = -*(int16_t *)(rx_buf + 6);
        } else {
            // Swap X and Y, negate all
            data->mag[1] = -convert_mag(*(int16_t *)(rx_buf + 2));
            data->mag[0] = -convert_mag(*(int16_t *)(rx_buf + 4));
            data->mag[2] = -convert_mag(*(int16_t *)(rx_buf + 6));
        }
        
        return 1;
    }
    
    return 0;
}

uint16_t lis3mdl_closest_range(uint16_t range)
{
    uint8_t i = NUM_RANGES - 1;
    uint16_t target_range = ranges[i];
    while (i--) {
        if (ranges[i] < range) break;
        target_range = ranges[i];
    }
    return target_range;
}

void lis3mdl_set_range(uint16_t range)
{
    if (g_lis3mdl.suspended)
        return;

    uint8_t i;
    for (i = 0; i < NUM_RANGES; i++) {
        // Set the full scale range if it is a valid one
        if (range == ranges[i]) {
            if (g_lis3mdl.range != i) {
                g_lis3mdl.range = i;
                uint8_t ctrl2[] = {CTRL_REG2, bitmasks[g_lis3mdl.range]};
                uint8_t rx_buf[2];
                spi_transfer(&g_lis3mdl.spi, ctrl2, rx_buf, sizeof(ctrl2), 0);
            }
            break;
        }
    }
}

uint16_t lis3mdl_get_range(void)
{
    return ranges[g_lis3mdl.range];
}

void lis3mdl_set_register(uint8_t reg, uint8_t payload)
{
    uint8_t set_reg[] = {reg, payload};
    uint8_t rx_buf[2];
    spi_transfer(&g_lis3mdl.spi, set_reg, rx_buf, sizeof(set_reg), 0);
}

void lis3mdl_ready(void)
{
    g_lis3mdl.data_ready++;
}
