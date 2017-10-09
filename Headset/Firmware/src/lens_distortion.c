/************************************************************************************

Filename    :   lens_distortion.c
Content     :   Storage of lens distortion parameters
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2014-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "lens_distortion.h"
#include "config.h"
#include <string.h>

static uint8_t distortion_index = 0;

static void lens_distortion_get_default(uint8_t *buf)
{
    buf[0] = 0; // Bitmask
    *(uint16_t *)(buf + 1) = 0; // LensType
    *(uint16_t *)(buf + 3) = 1; // Version
    *(uint16_t *)(buf + 5) = 17000; // EyeRelief
    *(uint16_t *)(buf + 7) = 16433; // K[0]
    *(uint16_t *)(buf + 9) = 16712; // K[1]
    *(uint16_t *)(buf + 11) = 17072; // K[2]
    *(uint16_t *)(buf + 13) = 17465; // K[3]
    *(uint16_t *)(buf + 15) = 17924; // K[4]
    *(uint16_t *)(buf + 17) = 18448; // K[5]
    *(uint16_t *)(buf + 19) = 19038; // K[6]
    *(uint16_t *)(buf + 21) = 19710; // K[7]
    *(uint16_t *)(buf + 23) = 20480; // K[8]
    *(uint16_t *)(buf + 25) = 21463; // K[9]
    *(uint16_t *)(buf + 27) = 22610; // K[10]
    *(uint16_t *)(buf + 29) = 16384; // MaxR
    *(uint16_t *)(buf + 31) = 18874; // MetersPerTanAngle
    *(uint16_t *)(buf + 33) = 24904; // Chromatic[0]
    *(uint16_t *)(buf + 35) = 22282; // Chromatic[1]
    *(uint16_t *)(buf + 37) = 45875; // Chromatic[2]
    *(uint16_t *)(buf + 39) = 43254; // Chromatic[3]
    memset(buf + 41, 0, 14); // Zero the remaining buffer
}

void lens_distortion_get(uint8_t *buf)
{
    buf[0] = CFG_NUM_LENSDISTORTION;
    buf[1] = distortion_index;
    // Load the report directly out of EEPROM
    if (!config_read(buf + 2, CFG_SIZE_LENSDISTORTION, CFG_ADDR_LENSDISTORTION + CFG_OFFSET_LENSDISTORTION(distortion_index))) {
        // Load defaults for DK2 lens A if no custom config is stored
        lens_distortion_get_default(buf + 2);
    }
    
    // Autoincrement on reads
    distortion_index++;
    if (distortion_index >= CFG_NUM_LENSDISTORTION)
        distortion_index = 0;
}

void lens_distortion_set(uint8_t *buf)
{
    // Make sure this is a valid lens distortion index
    if (buf[1] < CFG_NUM_LENSDISTORTION) {
        // Use the specified offset into the array of lens distortions
        config_write(buf + 2, CFG_SIZE_LENSDISTORTION, CFG_ADDR_LENSDISTORTION + CFG_OFFSET_LENSDISTORTION(buf[1]));
        
        // Set the index on write for quicker read back verification
        distortion_index = buf[1];
    }
}
