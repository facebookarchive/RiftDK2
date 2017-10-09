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

#ifndef _BOOTLOADER_STATE_
#define _BOOTLOADER_STATE_

#include <stdint.h>

enum {
    BOOTLOAD_CLEAR = 0,
    BOOTLOAD_MANUAL = 1,
    BOOTLOAD_INCOMPLETE = 2,
    BOOTLOAD_HARDFAULT = 3
};

uint8_t bootloader_state_get(void);

void bootloader_state_set(uint8_t state);

#endif /* _BOOTLOADER_STATE_ */
