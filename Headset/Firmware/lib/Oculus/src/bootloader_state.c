/************************************************************************************

Filename    :   bootloader_state.h
Content     :   State enumeration for the bootloader
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2014-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "bootloader_state.h"
#include "config.h"

uint8_t bootloader_state_get(void)
{
    uint8_t state;
    if (!config_read(&state, CFG_SIZE_BOOTLOAD, CFG_ADDR_BOOTLOAD)) {
        return BOOTLOAD_CLEAR;
    }
    
    return state;
}

void bootloader_state_set(uint8_t state)
{
    config_write(&state, CFG_SIZE_BOOTLOAD, CFG_ADDR_BOOTLOAD);
}
