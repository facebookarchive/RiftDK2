/************************************************************************************

Filename    :   mpu6500.h
Content     :   Invensense MPU-6500 gyro/accel interface
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _MPU6500_H_
#define _MPU6500_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct mpu6500_data_struct {
    uint16_t count;
    uint32_t timestamp;
    int16_t temperature;
    int32_t accel[3];
    float gyro[3];
} mpu6500_data_t, *mpu6500_data_p;

void mpu6500_init(void);

void mpu6500_sleep(bool sleep);

bool mpu6500_motion_interrupt(void);

bool mpu6500_read(mpu6500_data_p data, bool raw);

uint8_t mpu6500_closest_accel_range(uint8_t accel);

uint16_t mpu6500_closest_gyro_range(uint16_t gyro);

void mpu6500_set_ranges(uint8_t accel, uint16_t gyro);

void mpu6500_get_ranges(uint8_t *accel, uint16_t *gyro);

void mpu6500_set_register(uint8_t reg, uint8_t payload);

void mpu6500_ready(bool rolled);

#endif /* _MPU6500_H_ */
