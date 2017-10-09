/************************************************************************************

Filename    :   feature_report.h
Content     :   DK2 feature reports
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _FEATURE_REPORT_H_
#define _FEATURE_REPORT_H_

#include <stdint.h>
#include <stdbool.h>
#include "sequence.h"
#include "panel.h"
#include "calibrate.h"

typedef struct feature_config_struct {
    bool use_raw;
    bool calibrate;
    bool use_calibration;
    bool autocalibration;
    bool motion_keep_alive;
    bool command_keep_alive;
    bool use_sensor_coordinates;
    bool override_power;
    uint8_t interval;
    uint16_t sample_rate;
} feature_config_t, *feature_config_p;

typedef struct feature_dfu_struct {
    uint16_t command_id;
    bool use_dfu;
} feature_dfu_t, *feature_dfu_p;

typedef struct feature_range_struct {
    uint8_t accel_range;
    uint16_t gyro_range;
    uint16_t mag_range;
} feature_range_t, *feature_range_p;

typedef struct feature_reg_struct {
    uint8_t device;
    uint8_t reg;
    uint8_t payload;
} feature_register_t, *feature_register_p;

typedef struct feature_keep_alive_struct {
    uint8_t in_report;
    uint16_t keep_alive;
} feature_keep_alive_t, *feature_keep_alive_p;

enum {
    FEATURE_REGISTER_INV = 1,
    FEATURE_REGISTER_HMC = 2,
    FEATURE_REGISTER_MPU6500 = 3,
    FEATURE_REGISTER_LIS3MDL = 4
};

// Callbacks for USB
uint16_t feature_report_length(uint8_t report_id, uint16_t length);
uint8_t *feature_report_get_buf(uint8_t report_id, uint16_t offset);
uint8_t *feature_report_set_buf(uint8_t report_id, uint16_t offset);

void feature_report_parse(void);

uint32_t feature_report_count(void);

uint16_t feature_report_latest_command_id(void);

uint32_t feature_report_latest_command_time(void);

void feature_report_init_config(feature_config_p config);

void feature_report_init_keep_alive(feature_keep_alive_p keep_alive);

bool feature_report_get_config(feature_config_p config);

bool feature_report_get_range(feature_range_p range);

bool feature_report_get_register(feature_register_p reg);

bool feature_report_get_dfu(feature_dfu_p dfu);

bool feature_report_get_keep_alive(feature_keep_alive_p keep_alive);

bool feature_report_get_tracking(sequence_p sequence);

bool feature_report_get_display(panel_p panel);

bool feature_report_get_pattern(sequence_pattern_p pattern);

#endif /* _FEATURE_REPORT_H_ */
