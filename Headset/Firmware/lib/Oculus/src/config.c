/************************************************************************************

Filename    :   config.c
Content     :   Persistant application configuration handling.
Created     :
Authors     :   Lyle Bainbridge

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include <stm32l1xx.h>
#include <stdint.h>
#include <string.h>
#include "crc.h"
#include "config.h"
#if USE_INTERNAL_EEPROM
#include "eeprom.h"
#else /* USE_INTERNAL_EEPROM */
#include "m24.h"
#include "gpio.h"
#include "hw_config.h"
#define MAX_DATA_LEN (1024)
#endif /* USE_INTERNAL_EEPROM */

static m24_t g_config_m24 = {{0}};

uint8_t config_init(void)
{
    uint8_t ret = 1;
    
#if !USE_INTERNAL_EEPROM
    // Configure the scl pullup
    gpio_config(I2C_SCL_PU_EN_PORT, I2C_SCL_PU_EN_PIN, GPIO_PP);
    GPIO_WriteBit(I2C_SCL_PU_EN_PORT, I2C_SCL_PU_EN_PIN, Bit_SET);
    
    // Initialize the EEPROM interface
    g_config_m24.address = CONFIG_SLAVE_ADDR;
    g_config_m24.cpal = CONFIG_I2C_CPAL;
    g_config_m24.device = CONFIG_I2C_DEVICE;
    g_config_m24.options = CPAL_OPT_16BIT_REG;
    
    ret = m24_init(&g_config_m24);
#endif /* !USE_INTERNAL_EEPROM */
    
    return ret;
}

uint8_t config_write(uint8_t* p_data, uint16_t len, uint32_t address)
{
    uint8_t result = 0;
    uint16_t crc = crc16(p_data, len);

#if USE_INTERNAL_EEPROM
    FLASH_Status status = FLASH_COMPLETE;
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | 
                    FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | 
                    FLASH_FLAG_OPTVERRUSR);

    status = eeprom_write_halfword(address, crc);

    if (status == FLASH_COMPLETE) {
        status = eeprom_write_data((address + 2), p_data, len);

        if (status == FLASH_COMPLETE) {
            result = 1;
        }
    }
#else /* USE_INTERNAL_EEPROM */
    // Use a big fixed length buffer to avoid bringing in VLA
    if (len + sizeof(uint16_t) < MAX_DATA_LEN) {
        // Create a single buffer with the crc and data since we want to perform
        // the fewest number of writes
        uint8_t buf[MAX_DATA_LEN];
        *(uint16_t *)buf = crc;
        memcpy(buf + sizeof(uint16_t), p_data, len);
        result = m24_write(&g_config_m24, address, buf, len + sizeof(uint16_t));
    }
#endif /* USE_INTERNAL_EEPROM */

    return result;
}

uint8_t config_write_raw(uint8_t* p_data, uint16_t len, uint32_t address)
{
    return m24_write(&g_config_m24, address, p_data, len);
}

uint8_t config_read(uint8_t* p_data, uint16_t len, uint32_t address)
{
    uint8_t result = 0;
    uint16_t crc = 0;

#if USE_INTERNAL_EEPROM
    crc = crc16((uint8_t*)(address + 2), len);

    if (crc == *((uint16_t*)address)) {
        memcpy(p_data, (uint8_t*)(address + 2), len);
        result = 1;
    }
#else /* USE_INTERNAL_EEPROM */
    uint8_t buf[MAX_DATA_LEN];
    // Read the CRC and data
    if ((len < MAX_DATA_LEN) && m24_read(&g_config_m24, address, buf, len + sizeof(uint16_t))) {
        crc = crc16(buf + sizeof(uint16_t), len);
        // Verify that the calculated CRC matches the stored one
        if (crc == *(uint16_t *)buf) {
            memcpy(p_data, buf + sizeof(uint16_t), len);
            result = 1;
        } else {
            result = 0;
        }
    }
#endif /* USE_INTERNAL_EEPROM */

    return result;
}

uint8_t config_read_raw(uint8_t* p_data, uint16_t len, uint32_t address)
{
    return m24_read(&g_config_m24, address, p_data, len);
}
