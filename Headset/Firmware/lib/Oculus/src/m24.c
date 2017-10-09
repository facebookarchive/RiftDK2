/************************************************************************************
Filename    :   m24.h
Content     :   Interface to M24 series EEPROM
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "m24.h"
#include "hw_config.h"
#include "gpio.h"
#include "cpal_i2c.h"
#include <string.h>

#define M24_WRITE_TIME (6)
#define M24_DEBUG (0)

static bool m24_config_i2c(m24_p m24)
{
    // Initialize the I2C interface for the EEPROM
    CPAL_I2C_DeInit(m24->cpal);
    CPAL_I2C_StructInit(m24->cpal);
    m24->cpal->CPAL_Dev = m24->device;
    m24->cpal->CPAL_Mode = CPAL_MODE_MASTER;
    m24->cpal->CPAL_ProgModel = CPAL_PROGMODEL_INTERRUPT;
    m24->cpal->pCPAL_TransferRx = &m24->rx;
    m24->cpal->pCPAL_TransferTx = &m24->tx;
    m24->cpal->wCPAL_Options |= m24->options;
    // The M24 series support 100 kHz, 400 kHz, and 1 MHz
    m24->cpal->pCPAL_I2C_Struct->I2C_ClockSpeed = 400000;
    return (CPAL_I2C_Init(m24->cpal) == CPAL_PASS);
}

static bool m24_cpal_wait_ready(m24_p m24)
{
    while (m24->cpal->CPAL_State != CPAL_STATE_READY) {
        if (m24->cpal->CPAL_State == CPAL_STATE_ERROR) {
            // Reinitialize i2c if there was an error
            m24_config_i2c(m24);
            return 0;
        }
    }
    
    // Once we know the cpal interface is free, load our structs in
    m24->cpal->pCPAL_TransferRx = &m24->rx;
    m24->cpal->pCPAL_TransferTx = &m24->tx;
    
    return 1;
}

bool m24_init(m24_p m24)
{    
    return m24_config_i2c(m24);
}

void m24_deinit(m24_p m24)
{
    // Wait until any pending write is complete before de-initializing I2C.
    // See GitHub issue #165
    while (m24->cpal->CPAL_State != CPAL_STATE_READY);

    CPAL_I2C_DeInit(m24->cpal);
}

bool m24_write(m24_p m24, uint32_t address, const uint8_t *data, uint16_t length)
{
    uint16_t written = 0;    
    
    while (written < length) {
        // Make sure writes only go up to the page boundary, since it wraps
        // within the same page if we write past it
        uint8_t write_len = M24_PAGE_SIZE - ((address + written) & (M24_PAGE_SIZE - 1));
        // Make sure we actually have that many bytes remaining
        if (write_len > length - written)
            write_len = length - written;
        
        // Wait for the i2c interface to be ready
        if (!m24_cpal_wait_ready(m24)) {
            return 0;
        }
        
        // Wait until the last write completed before a new write
        m24->tx.wAddr1 = m24->address | 1;
        while (CPAL_I2C_IsDeviceReady(m24->cpal) != CPAL_PASS);
        
        // Copy data into the CPAL buffer, since we may return before
        // waiting for the write to complete
        memcpy(m24->buf, data + written, write_len);
        m24->tx.pbBuffer = m24->buf;
        m24->tx.wNumData = write_len;
        m24->tx.wAddr2 = address + written;
        
        if (CPAL_I2C_Write(m24->cpal) != CPAL_PASS)
            return 0;
    
        written += write_len;
    }
    
#if M24_DEBUG
    // Do read back verification in debug mode
    uint8_t verify_buf[1024] = {0};
    if (m24_read(m24, address, verify_buf, length)) {
        return !memcmp(data, verify_buf, length);
    } else {
        return 0;
    }
#else /* M24_DEBUG */
    return 1;
#endif /* M24_DEBUG */
}

bool m24_read(m24_p m24, uint32_t address, uint8_t *data, uint16_t length)
{
    // Wait for the i2c interface to be ready
    if (!m24_cpal_wait_ready(m24)) {
        return 0;
    }
    
    // Just use the passed in buffer since we need to block until the read
    // is complete
    m24->rx.wNumData = length;
    m24->rx.wAddr1 = m24->address;
    m24->rx.wAddr2 = address;
    m24->rx.pbBuffer = data;
    // Tx addr is needed for waiting for ready
    m24->tx.wAddr1 = m24->address | 1;
    
    // Wait until the last write completed before performing a read
    while (CPAL_I2C_IsDeviceReady(m24->cpal) != CPAL_PASS);

    if (CPAL_I2C_Read(m24->cpal) != CPAL_PASS)
        return 0;

    if (!m24_cpal_wait_ready(m24)) {
        return 0;
    }
    
    return 1;
}
