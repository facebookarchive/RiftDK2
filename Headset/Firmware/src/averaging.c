/************************************************************************************

Filename    :   averaging.c
Content     :   Gyro sample averaging helper
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "averaging.h"
#include <string.h>

void averaging_reset(averaging_p avg)
{
    memset(avg, 0, sizeof(averaging_t));
}

void averaging_update(averaging_p avg, mpu6500_data_p data)
{
    // Accumulate the samples
    // TODO: should this all be integer or fixed point?
    avg->gyro[0] += (double)data->gyro[0];
    avg->gyro[1] += (double)data->gyro[1];
    avg->gyro[2] += (double)data->gyro[2];
    avg->acc[0] += data->accel[0];
    avg->acc[1] += data->accel[1];
    avg->acc[2] += data->accel[2];
    avg->temp += data->temperature;

    avg->sample_count++;
}

uint32_t averaging_count(averaging_p avg)
{
    return avg->sample_count;
}

void averaging_compute(averaging_p avg, float *gyro, int32_t *accel, int16_t *temp)
{
    if (gyro) {
        gyro[0] = (float)(avg->gyro[0] / (double)avg->sample_count);
        gyro[1] = (float)(avg->gyro[1] / (double)avg->sample_count);
        gyro[2] = (float)(avg->gyro[2] / (double)avg->sample_count);
    }

    if (accel) {
        accel[0] = avg->acc[0] / (int64_t)avg->sample_count;
        accel[1] = avg->acc[1] / (int64_t)avg->sample_count;
        accel[2] = avg->acc[2] / (int64_t)avg->sample_count;
    }

    if (temp)
        *temp = avg->temp / avg->sample_count;
}
