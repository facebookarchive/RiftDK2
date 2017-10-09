/************************************************************************************

Filename    :   lis3mdl.h
Content     :   ST LIS3MDL Magnetometer interface
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _LIS3MDL_H_
#define _LIS3MDL_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct lis3mdl_data_struct {
    int16_t mag[3];
} lis3mdl_data_t, *lis3mdl_data_p;

void lis3mdl_init(void);

void lis3mdl_sleep(bool sleep);

bool lis3mdl_read(lis3mdl_data_p data, bool raw);

uint16_t lis3mdl_closest_range(uint16_t range);

void lis3mdl_set_range(uint16_t range);

uint16_t lis3mdl_get_range(void);

void lis3mdl_set_register(uint8_t reg, uint8_t payload);

void lis3mdl_ready(void);

#endif /* _LIS3MDL_H_ */
