/************************************************************************************

Filename    :   display_info.c
Content     :   Headset display configuration storage
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "display_info.h"
#include "config.h"
#include <stdbool.h>
#include <string.h>

enum {
    DISPLAY_DIMS = 0x01,
    DISPLAY_K = 0x02
};

static void generate_default_display_info(uint8_t *buf)
{
    buf[0] = DISPLAY_DIMS; // DistortionType
    *(uint16_t *)(buf + 1) = 1920; // ResolutionX
    *(uint16_t *)(buf + 3) = 1080; // ResolutionY
    *(uint32_t *)(buf + 5) = 125760; // DisplayX
    *(uint32_t *)(buf + 9) = 70740; // DisplayY
    *(uint32_t *)(buf + 13) = 70740/2; // CenterV
    *(uint32_t *)(buf + 17) = 63500; // LensSeparation
    *(uint32_t *)(buf + 21) = 39620; // LensDistanceL
    *(uint32_t *)(buf + 25) = 39620; // LensDistanceR
    // For now, just keep the distortion at 0's
    memset(buf + 29, 0, 6 * sizeof(float));
}

void display_info_get(uint8_t *buf)
{
    // The display info is stored in EEPROM in the same format as the feature report,
    // so just pass the buffer directly
    uint8_t found_display_info = config_read(buf, CFG_SIZE_DISPLAY_INFO, CFG_ADDR_DISPLAY_INFO);

    // if display info stuff wasn't found in EEPROM, load the defaults
    if (!found_display_info) {
        generate_default_display_info(buf);
    }
}

void display_info_set(uint8_t *buf)
{
    // The display info is stored in EEPROM in the same format as the feature report,
    // so just pass the buffer directly
    config_write(buf, CFG_SIZE_DISPLAY_INFO, CFG_ADDR_DISPLAY_INFO);
}
