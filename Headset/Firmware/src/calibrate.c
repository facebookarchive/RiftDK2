/************************************************************************************

Filename    :   calibrate.c
Content     :   Sensor calibration store, fetch, and apply
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "calibrate.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "config.h"
#include "sequence_patterns.h"
#include "calibrate_positions.h"
#include "report_helper.h"
#include "temperature.h"

static calibrate_t g_cal = {0};
static uint8_t g_pos_index = 0;
static offset_bin_s current_offset = {0};

void calibrate_init(void)
{
    // Read the calibration configuration
    if (!config_read((uint8_t*)&g_cal, 
                sizeof(calibrate_t),
                CFG_ADDR_CALIBRATE)) {
        // If no calibration was found, make sure the sensors still provide data
        for (uint8_t i = 0; i < 3; i++) {
            // Make the scale matrices unity for now
            g_cal.gyro_scale[i][i] = 1.0F;
            g_cal.acc_scale[i][i] = 1.0F;
        }
    }
    
    // Start at the factory calibrated gyro offset
    memcpy(current_offset.offset, g_cal.gyro_offset, sizeof(float) * 3);
    current_offset.temperature_actual = g_cal.temperature;

    // Initialize the gyro offset temperature bins
    temperature_init(&current_offset);
}

void calibrate_from_factory(calibrate_p cal)
{
    memcpy(&g_cal, cal, sizeof(calibrate_t));

    // burn the parameters to nonvolatile storage
    config_write((uint8_t*)&g_cal,
                 sizeof(calibrate_t),
                 CFG_ADDR_CALIBRATE);

    // don't store the factory calibration in temperature bins permanently
    memcpy(current_offset.offset, g_cal.gyro_offset, sizeof(float) * 3);
    current_offset.temperature_actual = g_cal.temperature;
}

void calibrate_get_calibration_report(uint8_t *report)
{
    // Format the calibration data into the DK1 packed format for the 
    // calibration report
    // Accelerometer offset is first, and is already in int32_t format
    report_pack_sensor(report, g_cal.acc_offset[0], g_cal.acc_offset[1], g_cal.acc_offset[2]);
    
    // Gyro offset needs to be rounded from float
    report_pack_sensor(report + 8, OFFSET_TO_INT(g_cal.gyro_offset[0]),
                          OFFSET_TO_INT(g_cal.gyro_offset[1]),
                          OFFSET_TO_INT(g_cal.gyro_offset[2]));
    
    // The scale and cross axis matrices have special conversions
    for (uint8_t i = 0; i < 3; i++) {
        int32_t acc[3];
        int32_t gyro[3];
        for (uint8_t j = 0; j < 3; j++) {
            // The diagonal is scale, the rest is cross axis
            if (i == j) {
                acc[j] = SCALE_TO_INT(g_cal.acc_scale[i][j]);
                gyro[j] = SCALE_TO_INT(g_cal.gyro_scale[i][j]);
            } else {
                acc[j] = CROSSAXIS_TO_INT(g_cal.acc_scale[i][j]);
                gyro[j] = CROSSAXIS_TO_INT(g_cal.gyro_scale[i][j]);
            }
        }
        
        report_pack_sensor(report + 16 + i * 8, acc[0], acc[1], acc[2]);
        report_pack_sensor(report + 40 + i * 8, gyro[0], gyro[1], gyro[2]);
    }
    
    // The temperature is already in int16_t format
    *(int16_t *)(report + 64) = g_cal.temperature;
}

void calibrate_set_calibration_report(uint8_t *report)
{
    // Unpack from the DK1 calibration format into the calibration struct
    int32_t x, y, z;

    report_unpack_sensor(report, &x, &y, &z);
    // Accelerometer offset is already integer
    g_cal.acc_offset[0] = x;
    g_cal.acc_offset[1] = y;
    g_cal.acc_offset[2] = z;

    report_unpack_sensor(report + 8, &x, &y, &z);
    // Convert the gyro offset back to float
    g_cal.gyro_offset[0] = INT_TO_OFFSET(x);
    g_cal.gyro_offset[1] = INT_TO_OFFSET(y);
    g_cal.gyro_offset[2] = INT_TO_OFFSET(z);
    
    // parse the scale and cross axis matrices
    for (uint8_t i = 0; i < 3; i++) {
        int32_t acc[3];
        int32_t gyro[3];
        report_unpack_sensor(report + 16 + i * 8, &acc[0], &acc[1], &acc[2]);
        report_unpack_sensor(report + 40 + i * 8, &gyro[0], &gyro[1], &gyro[2]);

        for (uint8_t j = 0; j < 3; j++) {
            // The diagonal is scale, the rest is cross axis
            if (i == j) {
                g_cal.acc_scale[i][j] = INT_TO_SCALE(acc[j]);
                g_cal.gyro_scale[i][j] = INT_TO_SCALE(gyro[j]);
            } else {
                g_cal.acc_scale[i][j] = INT_TO_CROSSAXIS(acc[j]);
                g_cal.gyro_scale[i][j] = INT_TO_CROSSAXIS(gyro[j]);
            }
        }
    }

    // Temperature remains an int16_t
    g_cal.temperature = *(int16_t *)(report + 64);
    
    config_write((uint8_t*)&g_cal,
                 sizeof(calibrate_t),
                 CFG_ADDR_CALIBRATE);
}

void calibrate_get_mag_calibration(uint8_t *buf)
{
    // The mag calibration is stored in EEPROM directly in the feature report
    // format
    if (!config_read(buf, CFG_SIZE_CALIBRATE_MAG, CFG_ADDR_CALIBRATE_MAG)) {
        // Zero the report if the read is unsuccessful
        memset(buf, 0, CFG_SIZE_CALIBRATE_MAG);
    }
}

void calibrate_set_mag_calibration(uint8_t *buf)
{
    // The mag calibration is stored in EEPROM directly in the feature report
    // format
    config_write(buf, CFG_SIZE_CALIBRATE_MAG, CFG_ADDR_CALIBRATE_MAG);
}

static uint16_t calibrate_pos_type(uint16_t index)
{
    if (index < NUM_LEDS) {
        return CALIBRATE_POSITION_LED;
    } else {
        return CALIBRATE_POSITION_IMU;
    }
}

static void calibrate_parse_pos_calibration(calibrate_pos_p cal_pos, uint8_t *buf)
{
    cal_pos->position[0] = *(int32_t *)buf;
    cal_pos->position[1] = *(int32_t *)(buf + 4);
    cal_pos->position[2] = *(int32_t *)(buf + 8);
    cal_pos->normal[0] = *(int16_t *)(buf + 12);
    cal_pos->normal[1] = *(int16_t *)(buf + 14);
    cal_pos->normal[2] = *(int16_t *)(buf + 16);
    cal_pos->rotation = *(int16_t *)(buf + 18);
    cal_pos->version = buf[20];
}

static void calibrate_pack_pos_calibration(calibrate_pos_p cal_pos, uint8_t *buf)
{
    *(int32_t *)buf = cal_pos->position[0];
    *(int32_t *)(buf + 4) = cal_pos->position[1];
    *(int32_t *)(buf + 8) = cal_pos->position[2];
    *(int16_t *)(buf + 12) = cal_pos->normal[0];
    *(int16_t *)(buf + 14) = cal_pos->normal[1];
    *(int16_t *)(buf + 16) = cal_pos->normal[2];
    *(int16_t *)(buf + 18) = cal_pos->rotation;
    buf[20] = cal_pos->version;
}

void calibrate_get_pos_calibration(calibrate_pos_p cal_pos)
{
    cal_pos->index = g_pos_index;
    cal_pos->num_positions = NUM_POSITIONS;
    
    uint8_t buf[CFG_SIZE_CALIBRATE_POS];
    // Try to get the position that was stored in EEPROM during the
    // calibration procedure
    if (!config_read(buf, CFG_SIZE_CALIBRATE_POS, CFG_ADDR_CALIBRATE_POS + CFG_OFFSET_CALIBRATE_POS(g_pos_index))) {
        // Use the default values if not found
        memcpy(cal_pos->position, calibrate_default_positions[cal_pos->index], sizeof(int32_t) * 3);
        memcpy(cal_pos->normal, calibrate_default_normals[cal_pos->index], sizeof(int16_t) * 3);
        cal_pos->rotation = 0;
        cal_pos->version = CALIBRATE_VERSION_DEFAULT;
    } else {
        // Otherwise parse what we grabbed from EEPROM
        calibrate_parse_pos_calibration(cal_pos, buf);
        // If the version was written back as default, also just use the
        // default values
        if (cal_pos->version == CALIBRATE_VERSION_DEFAULT) {
            memcpy(cal_pos->position, calibrate_default_positions[cal_pos->index], sizeof(int32_t) * 3);
            memcpy(cal_pos->normal, calibrate_default_normals[cal_pos->index], sizeof(int16_t) * 3);
            cal_pos->rotation = 0;
        }
    }
    
    // Get the type for this index
    cal_pos->type = calibrate_pos_type(cal_pos->index);
    
    // Increment the read position at every read
    g_pos_index++;
    if (g_pos_index == NUM_POSITIONS)
        g_pos_index = 0;
}

void calibrate_set_pos_calibration(calibrate_pos_p cal_pos)
{
    // Only take valid positions
    if (cal_pos->index < NUM_POSITIONS) {
        // Make sure the set type matches that for the index
        if (cal_pos->type == calibrate_pos_type(cal_pos->index)) {
            uint8_t buf[CFG_SIZE_CALIBRATE_POS];
            // Go from the feature report to a packed buffer format
            calibrate_pack_pos_calibration(cal_pos, buf);
            
            // Store it in EEPROM, which can take 20 or so milliseconds
            config_write(buf, CFG_SIZE_CALIBRATE_POS, CFG_ADDR_CALIBRATE_POS + CFG_OFFSET_CALIBRATE_POS(cal_pos->index));
        }
        
        // Also reset the index to the written one to make read back
        // verification quicker
        g_pos_index = cal_pos->index;
    }
}

void calibrate_set_offset(float *offset, int16_t temperature)
{
    memcpy(current_offset.offset, offset, sizeof(float) * 3);
    current_offset.temperature_actual = temperature;
}

void calibrate_apply(mpu6500_data_p data)
{
    float gyro_axes[3] = {0.0F};
    float accel_axes[3] = {0};
    
    // Get the gyro offset for the current temperature
    temperature_calculate_offset(&current_offset, data->temperature);

    // Apply the offsets before doing cross axis scaling
    temperature_apply_offset(data->gyro);
    
    data->accel[0] -= g_cal.acc_offset[0];
    data->accel[1] -= g_cal.acc_offset[1];
    data->accel[2] -= g_cal.acc_offset[2];

    float accel_floats[3] = {data->accel[0], data->accel[1], data->accel[2]};
    
    // take cross axis sensitivity into account in the scaling
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            gyro_axes[i] += data->gyro[j] * g_cal.gyro_scale[i][j];
            accel_axes[i] += accel_floats[j] * g_cal.acc_scale[i][j];
        }
    }

    memcpy(data->gyro, gyro_axes, sizeof(float) * 3);
    data->accel[0] = lroundf(accel_axes[0]);
    data->accel[1] = lroundf(accel_axes[1]);
    data->accel[2] = lroundf(accel_axes[2]);
}
