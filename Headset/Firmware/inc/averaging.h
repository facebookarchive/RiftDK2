/************************************************************************************

Filename    :   averaging.h
Content     :   Gyro sample averaging helper
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _AVERAGING_H_
#define _AVERAGING_H_

#include <stdint.h>
#include "mpu6500.h"

typedef struct averaging_struct {
    uint32_t sample_count;
    double gyro[3];
    int64_t acc[3];
    int32_t temp;
} averaging_t, *averaging_p;

void averaging_reset(averaging_p avg);

void averaging_update(averaging_p avg, mpu6500_data_p data);

uint32_t averaging_count(averaging_p avg);

void averaging_compute(averaging_p avg, float *gyro, int32_t *accel, int16_t *temp);

#endif /* _AVERAGING_H_ */
