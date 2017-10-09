/************************************************************************************

Filename    :   gamma.c
Content     :   AMS568 gamma correction
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2014-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "gamma.h"

#define GAMMA_NUM_POINTS 10

static void gamma_apply_offsets(uint16_t *base_gamma)
{
    // Voltage adjustment at different grey levels
    static const uint8_t gamma_shift[GAMMA_NUM_POINTS] = {
        0,  // 255
        2,  // 203
        4,  // 151
        6,  // 87
        5,  // 51
        4,  // 31
        5,  // 23
        7,  // 11
        12, // 3
        0   // 0
    };
    
    // Color specific shifts at different levels
    static const int8_t color_shift[GAMMA_NUM_POINTS * 3] = {
        -5, 0,  0,  // 255
        1,  1,  0,  // 203
        0,  1,  -1, // 151
        0,  1,  -1, // 87
        -1, 1,  -1, // 51
        0,  2,  -3, // 35
        -1, 2,  -4, // 23
        -2, 6,  -1, // 11
        -2, 12, 8,  // 3
        0,  0,  0   // 0
    };
    
    // Generate the table
    // TODO: Figure out how low persistence effects this
    // TODO: Figure out how to use the OTP shift values
    for (uint8_t i = 0; i < GAMMA_NUM_POINTS; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            base_gamma[i * 3 + j] += gamma_shift[i] + color_shift[i * 3 + j];
        }
    }
}

static void gamma_apply_mtp(uint16_t *base_gamma, uint8_t *mtp)
{
    // FIXME: This formula isn't right
//    // MTP data is stored as one's complement for some reason
//    int16_t mtp_offset[GAMMA_NUM_POINTS * 3];
//    
//    // Convert 9 bit ones complement stored in 16 bit to 16 bit signed
//    for (uint8_t i = 0; i < 3; i++) {
//        if (mtp[i * 2]) {
//            mtp_offset[i] = -1 * mtp[i * 2 + 1];
//        } else {
//            mtp_offset[i] = mtp[i * 2 + 1];
//        }
//    }
//    
//    // Convert 8 bit ones complement stored in 8 bit to 16 bit signed
//    for (uint8_t i = 0; i < (GAMMA_NUM_POINTS - 1) * 3; i++) {
//        if (mtp[i] & 0x80) {
//            mtp_offset[i + 3] = -1 * (mtp[i + 6] & 0x7F);
//        } else {
//            mtp_offset[i + 3] = mtp[i + 6];
//        }
//    }
//    
//    for (uint8_t i = 0; i < GAMMA_NUM_POINTS * 3; i++) {
//        base_gamma[i] -= mtp_offset[i];
//    }
}

void gamma_generate(uint8_t *gamma, uint8_t *mtp)
{
    // Starting values for the gamma adjustment table
    uint16_t base_gamma[GAMMA_NUM_POINTS * 3] = {
        0x0100, 0x0100, 0x0100, // 255
        0x0080, 0x0080, 0x0080, // 203
        0x0080, 0x0080, 0x0080, // 151
        0x0080, 0x0080, 0x0080, // 87
        0x0080, 0x0080, 0x0080, // 51
        0x0080, 0x0080, 0x0080, // 31
        0x0080, 0x0080, 0x0080, // 23
        0x0080, 0x0080, 0x0080, // 11
        0x0080, 0x0080, 0x0080, // 3
        0x0000, 0x0000, 0x0000  // 0
    };
    
    // Apply offsets for the ELVSS and persistence settings
    gamma_apply_offsets(base_gamma);
    
    // Apply the factory calibrated per-panel gamma offsets
    gamma_apply_mtp(base_gamma, mtp);
    
    // Fill in the 16 bit 255 value fields
    for (uint8_t i = 0; i < 3; i++) {
        gamma[i * 2 + 1] = base_gamma[i] >> 8;
        gamma[i * 2 + 2] = base_gamma[i] & 0xFF;
    }
    
    // Fill in the 8 bit fields for the rest of the table
    for (uint8_t i = 0; i < (GAMMA_NUM_POINTS - 1) * 3; i++) {
        gamma[i + 7] = base_gamma[i + 3] > 255 ? 255 : base_gamma[i + 3];
    }
}
