/************************************************************************************

Filename    :   bootloader.c
Content     :   Oculus bootloader operation
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "bootloader.h"
#include "bootloader_config.h"
#include "bootloader_state.h"
#include "eeprom.h"
#include "hw_config.h"
#include "stm32l1xx_flash.h"
#include "delay.h"
#include "config.h"
#include <stdlib.h>
#include <string.h>

enum {
    COMMAND_START = 0x01,
    COMMAND_COMPLETE = 0x02,
    COMMAND_REBOOT= 0x03,
    COMMAND_SETREGION = 0x04,
    COMMAND_SETBLOCK = 0x05,
    COMMAND_BUSY = 0x80,
    COMMAND_IDLE = 0x81
};

memory_region_p current_region = NULL;
uint32_t current_block = 0;
static bool g_reboot = 0;

bool bootloader_should_use(void)
{
    // Check if the bootloader was set to be used
    uint8_t use_bootloader = bootloader_state_get();

    // Clear the use bootloader setting only if we were manually set
    // to enter the bootloader, rather than being here due to error
    if (use_bootloader == BOOTLOAD_MANUAL) {
        bootloader_state_set(BOOTLOAD_CLEAR);
    }

    return (use_bootloader != BOOTLOAD_CLEAR);
}

bool bootloader_reboot(void)
{
    return g_reboot;
}

void bootloader_init(void)
{
    bootloader_config_init();
    // default to region 0
    current_region = bootloader_config_get_region(0);
}

void bootloader_get_config(uint16_t *mask, uint8_t *num_regions)
{
    *mask = 0;
    *num_regions = BOOTLOADER_NUM_REGIONS;
}

void bootloader_get_command(uint8_t *command, uint32_t *value)
{
    // Just always say idle for now since we only do synchronous writes
    *command = COMMAND_IDLE;
    *value = 0;
}

void bootloader_get_memory_region(memory_region_p memory_region)
{
    memcpy(memory_region, current_region, sizeof(memory_region_s));
}

static void bootloader_read_internal(memory_region_p current_region, uint32_t address, uint8_t *payload)
{
    memcpy(payload, (uint32_t *)address, current_region->block_size);
}

static void bootloader_read_i2c(memory_region_p current_region, uint32_t address, uint8_t *payload)
{
    config_read_raw(payload, current_region->block_size, address);
}

void bootloader_get_payload(uint8_t *region_num, uint32_t *block_num, uint8_t *payload)
{
    uint32_t address;

    switch (current_region->region_type) {
        // Treat virtual eeprom and eeprom like flash for read/write purposes
        case REGION_TYPE_FLASH:
        case REGION_TYPE_VIRTUAL_EEPROM:
        case REGION_TYPE_EEPROM:
            *region_num = current_region->region_num;
            *block_num = current_block;

            // Do the actual copy
            address = current_region->address + current_region->block_size*current_block;
            if (current_region->region_location == REGION_LOCATION_INTERNAL) {
                bootloader_read_internal(current_region, address, payload);
            } else if (current_region->region_location == REGION_LOCATION_I2C) {
                bootloader_read_i2c(current_region, address, payload);
            }

            // Auto increment the read pointer as long as next block is valid
            if (current_block + 1 < current_region->num_blocks)
                current_block++;
            break;

        default:
            break;
    }
}

void bootloader_set_command(uint8_t command, uint32_t value)
{
    memory_region_p new_region;

    switch (command) {
        case COMMAND_START:
            // Mark that we are starting bootloading, so that we reboot into
            // bootloader in the event the device resets partway
            bootloader_state_set(BOOTLOAD_INCOMPLETE);
            break;

        case COMMAND_COMPLETE:
            // Mark that bootloading activity is complete, and the device
            // can reboot to application firmware
            bootloader_state_set(BOOTLOAD_CLEAR);
            break;

        case COMMAND_REBOOT:
            g_reboot = 1;
            break;

        case COMMAND_SETREGION:
            new_region = bootloader_config_get_region(value);
            // make sure the region is valid
            if (new_region) {
                current_region = new_region;
                current_block = 0;
            }
            break;

        case COMMAND_SETBLOCK:
            // make sure the block number is valid
            if (current_region && (value < current_region->num_blocks))
                current_block = value;
            break;

        default:
            break;
    }
}

static void bootloader_write_internal(memory_region_p current_region, uint32_t address, uint8_t *payload)
{
    FLASH_Unlock();
    // Flash needs to have the page erased before being written
    FLASH_ErasePage(address);

    // If this is internal flash and a multiple of half page (128 bytes),
    // use the much faster half page write function
    if ((current_region->region_type == REGION_TYPE_FLASH) && !(current_region->block_size & 127)) {
        // We'll bus fault if any reads occur during the half page write,
        // so disable interrupts
        __disable_irq();
        // Write the page a half page at a time
        for (uint32_t i = 0; i < current_region->block_size; i += 128) {
            FLASH_ProgramHalfPage(address + i, (uint32_t *)(payload + i));
        }
        __enable_irq();
    } else {
        // Write the page one word at a time
        for (uint32_t i = 0; i < current_region->block_size; i += sizeof(uint32_t)) {
            FLASH_FastProgramWord(address + i, *(uint32_t *)(payload + i));
        }
    }
    FLASH_Lock();
}

static void bootloader_write_i2c(memory_region_p current_region, uint32_t address, uint8_t *payload)
{
    config_write_raw(payload, current_region->block_size, address);
}

void bootloader_set_payload(uint8_t region_num, uint16_t block_num, uint8_t *payload)
{
    memory_region_p new_region = bootloader_config_get_region(region_num);
    // make sure the region and block are valid
    if (new_region) {
        if (block_num < new_region->num_blocks) {
            // set the currently used block and region so a read can be done
            // afterwards on the same spot
            current_region = new_region;
            current_block = block_num;

            // compute the actual write address
            uint32_t address = current_region->address + current_region->block_size*current_block;

            if (current_region->region_location == REGION_LOCATION_INTERNAL) {
                bootloader_write_internal(current_region, address, payload);
            } else if (current_region->region_location == REGION_LOCATION_I2C) {
                bootloader_write_i2c(current_region, address, payload);
            }
        }
    }
}
