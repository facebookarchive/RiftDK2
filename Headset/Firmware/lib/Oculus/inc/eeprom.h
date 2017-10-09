/************************************************************************************

Filename    :   eeprom.h
Content     :   EEPROM access functions
Created     :
Authors     :   Lyle Bainbridge

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _EEPROM_H_
#define _EEPROM_H_

#include <stdint.h>
#include "stm32l1xx_flash.h"

#define INTERNAL_EEPROM_START_ADDR       0x08080000
#define EEPROM_END_ADDR         0x080807ff
#define EEPROM_SIZE     		(EEPROM_END_ADDR - INTERNAL_EEPROM_START_ADDR)
#define INTERNAL_EEPROM_PAGE_SIZE        0x08


FLASH_Status eeprom_write_data(uint32_t address, uint8_t* p_data, uint16_t length);

FLASH_Status eeprom_write_word(uint32_t address, uint32_t data);

FLASH_Status eeprom_write_halfword(uint32_t address, uint16_t data);

FLASH_Status eeprom_erase_all(void);

#endif /* _EEPROM_H_ */
