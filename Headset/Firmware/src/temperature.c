/************************************************************************************

Filename    :   temperature.c
Content     :   Gyro run time temperature compensation
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "temperature.h"
#include "config.h"
#include "report_helper.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

typedef struct interpolate_struct {
    float slope[3];
    float intercept[3];
} interpolate_s, *interpolate_p;

typedef struct temperature_report_struct {
    uint32_t time;
    int16_t actual;
    uint8_t offset[8];
    uint8_t version;
    uint8_t bin;
    uint8_t sample;
} temperature_report_t, *temperature_report_p;

// These values are all in centidegrees celcius
#define OFFSET_BIN_BASE (1500)
#define OFFSET_BIN_WIDTH (500)
#define TEMP_THRESHOLD (100)
#define BIN_NOT_FOUND (255)

// Gyro offset calibration data in temperature bins
static offset_bin_s offset_bins[CFG_NUM_TEMPERATURE_BINS];
static offset_bin_s temp_offset = {0};
static interpolate_s interp = {{0.0f}};

static uint8_t g_current_bin = 0;
static uint8_t g_current_sample = 0;

static void temperature_parse_feature_report(temperature_report_p report, uint8_t *buf)
{
    report->version = buf[0];
    report->bin = buf[2];
    report->sample = buf[4];
    report->actual = *(int16_t *)(buf + 7);
    report->time = *(uint32_t *)(buf + 9);
    memcpy(report->offset, buf + 13, 8);
}

static void temperature_pack_feature_report(temperature_report_p report, uint8_t *buf)
{
    buf[0] = report->version;
    buf[1] = CFG_NUM_TEMPERATURE_BINS;
    buf[2] = g_current_bin;
    buf[3] = CFG_NUM_TEMPERATURE_SAMPLES;
    buf[4] = g_current_sample;
    *(int16_t *)(buf + 5) = OFFSET_BIN_BASE + OFFSET_BIN_WIDTH * g_current_bin;
    *(int16_t *)(buf + 7) = report->actual;
    *(uint32_t *)(buf + 9) = report->time;
    memcpy(buf + 13, report->offset, 8);
}

static void temperature_parse_eeprom(temperature_report_p report, uint8_t *buf)
{
    report->time = *(uint32_t *)buf;
    report->actual = *(int16_t *)(buf + 4);
    report->version = buf[6];
    memcpy(report->offset, buf + 7, 8);
}

static void temperature_pack_eeprom(temperature_report_p report, uint8_t *buf)
{
    *(uint32_t *)buf = report->time;
    *(int16_t *)(buf + 4) = report->actual;
    buf[6] = report->version;
    memcpy(buf + 7, report->offset, 8);
}

void temperature_init(offset_bin_p default_offset)
{
    int16_t bin_temperature = OFFSET_BIN_BASE;

    // Read out all of the gyro offset bins from different temperatures
    for (uint8_t i = 0; i < CFG_NUM_TEMPERATURE_BINS; i++) {
        // Grab the most recent bin stored by the PC in EEPROM
        uint32_t latest_time = 0;
        for (uint8_t j = 0; j < CFG_NUM_TEMPERATURE_SAMPLES; j++) {
            uint8_t eeprom_buf[CFG_SIZE_TEMPERATURE];
            // Read each sample in the bin
            if (config_read(eeprom_buf, CFG_SIZE_TEMPERATURE, CFG_ADDR_TEMPERATURE + CFG_OFFSET_TEMPERATURE(i, j))) {
                temperature_report_t report;
                // Unpack it if it exists
                temperature_parse_eeprom(&report, eeprom_buf);
                
                // If it is newer than what we have seen so far
                if (report.time > latest_time) {
                    latest_time = report.time;
                    int32_t raw[3];
                    report_unpack_sensor(report.offset, raw, raw + 1, raw + 2);
                    // Copy the offsets and temperature
                    offset_bins[i].offset[0] = (float)raw[0];
                    offset_bins[i].offset[1] = (float)raw[1];
                    offset_bins[i].offset[2] = (float)raw[2];
                    offset_bins[i].temperature_actual = offset_bins[i].temperature_stored = report.actual;
                }
            }
        }
        
        offset_bins[i].temperature_target = bin_temperature;
        bin_temperature += OFFSET_BIN_WIDTH;
    }
}

void temperature_get_report(uint8_t *buf)
{
    temperature_report_t report;
    uint8_t eeprom_buf[CFG_SIZE_TEMPERATURE];
    if (!config_read(eeprom_buf, CFG_SIZE_TEMPERATURE, CFG_ADDR_TEMPERATURE + CFG_OFFSET_TEMPERATURE(g_current_bin, g_current_sample))) {
        // Zero the buffer if we can't find the sample
        memset(eeprom_buf, 0, CFG_SIZE_TEMPERATURE);
    }
    
    temperature_parse_eeprom(&report, eeprom_buf);
    temperature_pack_feature_report(&report, buf);
    
    // Increment the sample on reads
    g_current_sample++;
    if (g_current_sample >= CFG_NUM_TEMPERATURE_SAMPLES) {
        g_current_sample = 0;
        // Go to the next bin on reading all of the samples in the bin
        g_current_bin++;
        if (g_current_bin >= CFG_NUM_TEMPERATURE_BINS)
            g_current_bin = 0;
    }
}

void temperature_set_report(uint8_t *buf)
{
    temperature_report_t report;
    temperature_parse_feature_report(&report, buf);
    
    // Make sure the bin and sample numbers are valid
    if ((report.bin < CFG_NUM_TEMPERATURE_BINS) && (report.sample < CFG_NUM_TEMPERATURE_SAMPLES)) {
        uint8_t eeprom_buf[CFG_SIZE_TEMPERATURE];
        temperature_pack_eeprom(&report, eeprom_buf);
        // Write it at the right offset address for the bin and sample
        config_write(eeprom_buf, CFG_SIZE_TEMPERATURE, CFG_ADDR_TEMPERATURE + CFG_OFFSET_TEMPERATURE(report.bin, report.sample));
        
        // Use this index as the new read index to make read back verification
        // quicker
        g_current_bin = report.bin;
        g_current_sample = report.sample;
    }
}

static void temperature_find_closest_bins(int16_t temperature, uint8_t *closest, uint8_t *second)
{
    int16_t closest_diff = OFFSET_BIN_WIDTH * CFG_NUM_TEMPERATURE_BINS + OFFSET_BIN_BASE;
    int16_t second_diff = closest_diff;

    for (uint8_t i = 0; i < CFG_NUM_TEMPERATURE_BINS; i++) {
        int16_t new_diff = abs(temperature - offset_bins[i].temperature_actual);
        if (new_diff < closest_diff) {
            // Previous best is now second best
            *second = *closest;
            second_diff = closest_diff;

            *closest = i;
            closest_diff = new_diff;
        } else if ((abs(offset_bins[i].temperature_actual - offset_bins[*closest].temperature_actual) > TEMP_THRESHOLD) && (new_diff < second_diff)) {
            // Try to find a second point that is spaced away from the first point
            // to avoid noise causing a crazy slope
            *second = i;
            second_diff = new_diff;
        }
    }
}

static void temperature_generate_interpolate(int16_t temperature)
{
    uint8_t closest = BIN_NOT_FOUND;
    uint8_t second = BIN_NOT_FOUND;

    temperature_find_closest_bins(temperature, &closest, &second);;

    // If we only have one bin, the offset is constant
    if (second == BIN_NOT_FOUND) {
        for (uint8_t i = 0; i < 3; i++) {
            interp.slope[i] = 0.0f;
            interp.intercept[i] = offset_bins[closest].offset[i];
        }
    } else {
        // Otherwise linearly interpolate between the two closest bins
        float temp_diff = (float)(offset_bins[closest].temperature_actual - offset_bins[second].temperature_actual);

        for (uint8_t i = 0; i < 3; i++) {
            interp.slope[i] = (offset_bins[closest].offset[i] - offset_bins[second].offset[i]) / temp_diff;
            interp.intercept[i] = offset_bins[closest].offset[i] - interp.slope[i] * offset_bins[closest].temperature_actual;
        }
    }
}

void temperature_calculate_offset(offset_bin_p current_offset, int16_t temperature)
{
    // TODO: use a short term average of temperature to avoid jumping around
    // due to noise in the temperature sensor, part of #23
    
    // Recalculate the interpolation when the new temperature is more than the
    // threshold away from the last calculation
    if (abs(temp_offset.temperature_actual - temperature) > TEMP_THRESHOLD / 2) {
        temperature_generate_interpolate(temperature);
        temp_offset.temperature_actual = temperature;
    }

    // prefer the autoaveraged offset over temperature generated ones when it is
    // close enough
    if (abs(temperature - current_offset->temperature_actual) > OFFSET_BIN_WIDTH / 2) {
        for (uint8_t i = 0; i < 3; i++) {
            temp_offset.offset[i] = interp.slope[i] * (float)temperature + interp.intercept[i];
        }
    } else {
        // still apply the slope when we use autocalibrated offset
        for (uint8_t i = 0; i < 3; i++) {
            temp_offset.offset[i] = current_offset->offset[i] + interp.slope[i] * (float)(temperature - current_offset->temperature_actual);
        }
    }
}

void temperature_apply_offset(float *gyro)
{
    gyro[0] -= temp_offset.offset[0];
    gyro[1] -= temp_offset.offset[1];
    gyro[2] -= temp_offset.offset[2];
}
