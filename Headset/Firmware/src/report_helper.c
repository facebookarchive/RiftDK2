/************************************************************************************

Filename    :   report_helper.c
Content     :   Defines and functions to help format some of the HID Reports
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "report_helper.h"

// sign extending trick from http://graphics.stanford.edu/~seander/bithacks.html#FixedSignExtend
struct {int32_t x:21;} s;

void report_pack_sensor(uint8_t *buf, int32_t x, int32_t y, int32_t z)
{
    // Pack 3 32 bit integers into 8 bytes
    buf[0] = x >> 13;
    buf[1] = x >> 5;
    buf[2] = (x << 3) | ((y >> 18) & 0x07);
    buf[3] = y >> 10;
    buf[4] = y >> 2;
    buf[5] = (y << 6) | ((z >> 15) & 0x3F);
    buf[6] = z >> 7;
    buf[7] = z << 1;
}

void report_unpack_sensor(const uint8_t *buf, int32_t *x, int32_t *y, int32_t *z)
{
    // Unpack 3 32 bit integers from 8 bytes
    *x = s.x = (buf[0] << 13) | (buf[1] << 5) | ((buf[2] & 0xF8) >> 3);
    *y = s.x = ((buf[2] & 0x07) << 18) | (buf[3] << 10) | (buf[4] << 2) |
                   ((buf[5] & 0xC0) >> 6);
    *z = s.x = ((buf[5] & 0x3F) << 15) | (buf[6] << 7) | (buf[7] >> 1);
}
