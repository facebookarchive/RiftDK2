/************************************************************************************

Filename    :   calibrate.h
Content     :   Sensor calibration store, fetch, and apply
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _CALIBRATE_H_
#define _CALIBRATE_H_

#include "mpu6500.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct calibrate_struct {
    int16_t temperature;
    float gyro_offset[3];
    int32_t acc_offset[3];
    // scale and cross axis sensitivity
    float gyro_scale[3][3];
    float acc_scale[3][3];
} calibrate_t, *calibrate_p;

typedef struct calibrate_pos_struct {
    int32_t position[3];
    int16_t normal[3];
    int16_t rotation;
    uint16_t index;
    uint16_t num_positions;
    uint16_t type;
    uint8_t version;
} calibrate_pos_t, *calibrate_pos_p;

void calibrate_init(void);

void calibrate_from_factory(calibrate_p cal);

void calibrate_get_calibration_report(uint8_t *report);

void calibrate_set_calibration_report(uint8_t *report);

void calibrate_get_mag_calibration(uint8_t *buf);

void calibrate_set_mag_calibration(uint8_t *buf);

void calibrate_get_pos_calibration(calibrate_pos_p cal_pos);

void calibrate_set_pos_calibration(calibrate_pos_p cal_pos);

void calibrate_set_offset(float *offset, int16_t temperature);

void calibrate_apply(mpu6500_data_p data);

#endif /* _CALIBRATE_H_ */
