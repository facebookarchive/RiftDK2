/************************************************************************************

Filename    :   eeprom.c
Content     :   EEPROM access functions
Created     :
Authors     :   Lyle Bainbridge

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "eeprom.h"
#include <stdbool.h>
#include "delay.h"

#define BYTE_SIZE   (sizeof(uint8_t))
#define WORD_SIZE   (sizeof(uint32_t))


FLASH_Status eeprom_write_data(uint32_t address, uint8_t* p_data, uint16_t length)
{
    FLASH_Status status = FLASH_COMPLETE;
    uint8_t bytes_written = 0;

    DATA_EEPROM_Unlock();

    while (length) {

        if (((address % WORD_SIZE) != 0) || (length < WORD_SIZE)) {
            status = DATA_EEPROM_FastProgramByte(address, *p_data);
            bytes_written = BYTE_SIZE;
        }
        else {
            status = DATA_EEPROM_FastProgramWord(address, *((uint32_t*)p_data));
            bytes_written = WORD_SIZE;
        }

        if (status == FLASH_BUSY) {
            delay_ms(1);
            continue;
        }

        if (status != FLASH_COMPLETE) {
            break;
        }

        p_data  += bytes_written;
        address += bytes_written;
        length  -= bytes_written;
    }

    DATA_EEPROM_Lock();

    return status;
}


FLASH_Status eeprom_write_word(uint32_t address, uint32_t data)
{
    FLASH_Status status = FLASH_COMPLETE;

    DATA_EEPROM_Unlock();

    status = DATA_EEPROM_FastProgramWord(address, data);

    DATA_EEPROM_Lock();

    return status;
}


FLASH_Status eeprom_write_halfword(uint32_t address, uint16_t data)
{
    FLASH_Status status = FLASH_COMPLETE;

    DATA_EEPROM_Unlock();

    status = DATA_EEPROM_FastProgramHalfWord(address, data);

    DATA_EEPROM_Lock();

    return status;
}


FLASH_Status eeprom_erase_all(void)
{
    FLASH_Status status = FLASH_COMPLETE;
    uint32_t address = INTERNAL_EEPROM_START_ADDR;

    DATA_EEPROM_Unlock();

    while (address < EEPROM_END_ADDR) {
        status = eeprom_write_word(address, 0xffffffff);

        if (status == FLASH_BUSY) {
            delay_ms(1);
            continue;
        }

        if (status != FLASH_COMPLETE) {
            break;
        }

        address += 4;
    }

    DATA_EEPROM_Lock();

    return status;
}
