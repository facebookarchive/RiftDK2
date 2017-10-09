/************************************************************************************

Filename    :   temperature.h
Content     :   Gyro run time temperature compensation
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _TEMPERATURE_H_
#define _TEMPERATURE_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct offset_bin_struct {
    int16_t temperature_target;
    int16_t temperature_actual;
    int16_t temperature_stored;
    float offset[3];
} offset_bin_s, *offset_bin_p;

void temperature_init(offset_bin_p default_offset);

void temperature_get_report(uint8_t *buf);

void temperature_set_report(uint8_t *buf);

void temperature_calculate_offset(offset_bin_p current_offset, int16_t temperature);

void temperature_apply_offset(float *gyro);

#endif /* _TEMPERATURE_H_ */
