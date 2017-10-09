/************************************************************************************

Filename    :   bootloader_config.c
Content     :   Oculus bootloader board specific configuration
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "bootloader_config.h"
#include "hw_config.h"
#include "eeprom.h"
#include "m24.h"
#include <stdlib.h>

memory_region_p memory_regions[BOOTLOADER_NUM_REGIONS];
memory_region_s internal_flash = {0};
memory_region_s internal_eeprom = {0};
memory_region_s external_eeprom = {0};

void bootloader_config_init(void)
{
    internal_flash.region_num = 0;
    internal_flash.region_type = REGION_TYPE_FLASH;
    // Everything but the initial 16 KB of bootloader is internal flash
    internal_flash.num_blocks = (128 - 16) * (1024 / 256); // (128 - 16) * 4
    internal_flash.block_size = 256; // On STM32L100B
    // 4 ms to erase, 4 ms for each of two half pages, plus some buffer
    internal_flash.write_time = 15;
    internal_flash.address = APPLICATION_ADDRESS;
    memory_regions[0] = &internal_flash;

    // The STM32L100B also has 2 KB of real EEPROM, though we won't use it for
    // REV3 and later
    internal_eeprom.region_num = 1;
    internal_eeprom.region_type = REGION_TYPE_EEPROM;
    internal_eeprom.num_blocks = 512;
    internal_eeprom.block_size = 4;
    // 4 ms to erase, 4 ms to write, plus some buffer
    internal_eeprom.write_time = 10;
    internal_eeprom.address = INTERNAL_EEPROM_START_ADDR;
    memory_regions[1] = &internal_eeprom;
    
    // We use 4 KB of i2c EEPROM
    external_eeprom.region_num = 2;
    external_eeprom.region_type = REGION_TYPE_EEPROM;
    external_eeprom.num_blocks = 4096 / M24_PAGE_SIZE;
    external_eeprom.block_size = M24_PAGE_SIZE;
    // 5 ms to erase, 5 ms to write, plus some buffer
    external_eeprom.write_time = 12;
    external_eeprom.address = 0;
    external_eeprom.region_location = REGION_LOCATION_I2C;
    memory_regions[2] = &external_eeprom;
}

memory_region_p bootloader_config_get_region(uint8_t region_num)
{
    if (region_num < BOOTLOADER_NUM_REGIONS)
        return memory_regions[region_num];
    else
        return NULL;
}
