/************************************************************************************

Filename    :   bootloader_config.h
Content     :   Oculus bootloader board specific configuration
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _BOOTLOADER_CONFIG_H_
#define _BOOTLOADER_CONFIG_H_

#include <stdint.h>
#include "bootloader.h"

// Configuration for DK2
#define BOOTLOADER_NUM_REGIONS 3

void bootloader_config_init(void);

memory_region_p bootloader_config_get_region(uint8_t region_num);

#endif /* _BOOTLOADER_CONFIG_H_ */
