/************************************************************************************

Filename    :   bootloader.h
Content     :   Oculus bootloader operation
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _BOOTLOADER_H_
#define _BOOTLOADER_H_

#include <stdbool.h>
#include <stdint.h>

enum {
    REGION_TYPE_UKNOWN = 0,
    REGION_TYPE_FLASH = 1,
    REGION_TYPE_BOOTLOADER = 2,
    REGION_TYPE_EEPROM = 3,
    REGION_TYPE_VIRTUAL_EEPROM = 4
};

enum {
    REGION_LOCATION_INTERNAL = 0,
    REGION_LOCATION_I2C = 1,
};

typedef struct memory_region_struct {
    uint32_t address;
    uint32_t num_blocks;
    uint16_t block_size;
    uint16_t write_time;
    uint8_t region_num;
    uint8_t region_type;
    uint8_t region_mask;
    uint8_t region_location;
} memory_region_s, *memory_region_p;

bool bootloader_should_use(void);

bool bootloader_reboot(void);

void bootloader_init(void);

void bootloader_get_config(uint16_t *mask, uint8_t *num_regions);

void bootloader_get_command(uint8_t *command, uint32_t *value);

void bootloader_get_memory_region(memory_region_p memory_region);

void bootloader_get_payload(uint8_t *region_num, uint32_t *block_num, uint8_t *payload);

void bootloader_set_command(uint8_t command, uint32_t value);

void bootloader_set_payload(uint8_t region_num, uint16_t block_num, uint8_t *payload);

#endif /* _BOOTLOADER_H_ */
