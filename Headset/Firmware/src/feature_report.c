/************************************************************************************

Filename    :   feature_report.c
Content     :   DK2 feature reports
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "feature_report.h"
#include "uuid.h"
#include "lis3mdl.h"
#include "display_info.h"
#include "manufacturing.h"
#include "systick.h"
#include "config.h"
#include "temperature.h"
#include "autocalibrate.h"
#include "lens_distortion.h"
#include <stdlib.h>
#include <string.h>

enum {
    // DK1 Reports
    FEATURE_CONFIG = 2,
    FEATURE_CALIBRATE = 3,
    FEATURE_RANGE = 4,
    FEATURE_REGISTER = 5,
    FEATURE_DFU = 6,
    FEATURE_GPIO = 7,
    FEATURE_KEEP_ALIVE = 8,
    FEATURE_DISPLAY_INFO = 9,
    FEATURE_SERIAL = 10,
    // DK2 Reports
    FEATURE_TRACKING = 12,
    FEATURE_DISPLAY = 13,
    FEATURE_MAGCALIBRATION = 14,
    FEATURE_POSCALIBRATION = 15,
    FEATURE_CUSTOMPATTERN = 16,
    FEATURE_KEEPALIVEMUX = 17,
    FEATURE_MANUFACTURING = 18,
    FEATURE_UUID = 19,
    FEATURE_TEMPERATURE = 20,
    FEATURE_GYROOFFSET = 21,
    FEATURE_LENSDISTORTION = 22,
    FEATURE_MAX
};

// Report ID and Command ID
#define FEATURE_HEADER_SIZE 3
#define FEATURE_MAX_SIZE 69   // FEATURE_CALIBRATE

static const uint8_t report_sizes[FEATURE_MAX] = {
    0, // Index 0, not valid for reports?
    0, // DK1 IN Report
    FEATURE_HEADER_SIZE + 4,  // FEATURE_CONFIG
    FEATURE_HEADER_SIZE + 66, // FEATURE_CALIBRATE
    FEATURE_HEADER_SIZE + 5,  // FEATURE_RANGE
    FEATURE_HEADER_SIZE + 3,  // FEATURE_REGISTER
    FEATURE_HEADER_SIZE + 1,  // FEATURE_DFU
    0, // FEATURE_GPIO - Unsupported on DK2
    FEATURE_HEADER_SIZE + 2,  // FEATURE_KEEP_ALIVE
    FEATURE_HEADER_SIZE + 53, // FEATURE_DISPLAY_INFO
    FEATURE_HEADER_SIZE + 12, // FEATURE_SERIAL
    0, // DK2 IN Report
    FEATURE_HEADER_SIZE + 10, // FEATURE_TRACKING
    FEATURE_HEADER_SIZE + 13, // FEATURE_DISPLAY
    FEATURE_HEADER_SIZE + 49, // FEATURE_MAGCALIBRATION
    FEATURE_HEADER_SIZE + 27, // FEATURE_POSCALIBRATION
    FEATURE_HEADER_SIZE + 9,  // FEATURE_CUSTOMPATTERN
    FEATURE_HEADER_SIZE + 3,  // FEATURE_KEEPALIVEMUX
    FEATURE_HEADER_SIZE + 16, // FEATURE_MANUFACTURING
    FEATURE_HEADER_SIZE + 23, // FEATURE_UUID
    FEATURE_HEADER_SIZE + 21, // FEATURE_TEMPERATURE
    FEATURE_HEADER_SIZE + 15, // FEATURE_GYROOFFSET
    FEATURE_HEADER_SIZE + 61, // FEATURE_LENSDISTORTION
};

// Most recent command ID globally
volatile static uint16_t feature_report_command_id = 0;
volatile static uint32_t g_report_count = 0;
volatile static uint32_t g_last_report_time = 0;

static feature_config_t g_config = {0};
volatile static bool g_config_changed = 0;
static feature_range_t g_range = {0};
volatile static bool g_range_changed = 0;
static feature_register_t g_register = {0};
volatile static bool g_register_changed = 0;
static feature_dfu_t g_dfu = {0};
volatile static bool g_dfu_changed = 0;
static feature_keep_alive_t g_keep_alive = {0};
volatile static bool g_keep_alive_changed = 0;
static sequence_t g_tracking = {0};
volatile static bool g_tracking_changed = 0;
static panel_t g_panel = {0};
volatile static bool g_panel_changed = 0;
static sequence_pattern_t g_pattern = {0};
volatile static bool g_pattern_changed = 0;

// For the largest buffer we need
uint8_t feature_get_buf[FEATURE_MAX_SIZE];
uint8_t feature_set_buf[FEATURE_MAX_SIZE];

uint16_t feature_report_length(uint8_t report_id, uint16_t length)
{
    // Switch based on ReportID
    switch (report_id) {
        case FEATURE_CONFIG:
        case FEATURE_CALIBRATE:
        case FEATURE_RANGE:
        case FEATURE_REGISTER:
        case FEATURE_DFU:
        case FEATURE_KEEP_ALIVE:
        case FEATURE_DISPLAY_INFO:
        case FEATURE_SERIAL:
        case FEATURE_TRACKING:
        case FEATURE_DISPLAY:
        case FEATURE_MAGCALIBRATION:
        case FEATURE_POSCALIBRATION:
        case FEATURE_CUSTOMPATTERN:
        case FEATURE_KEEPALIVEMUX:
        case FEATURE_MANUFACTURING:
        case FEATURE_UUID:
        case FEATURE_TEMPERATURE:
        case FEATURE_GYROOFFSET:
        case FEATURE_LENSDISTORTION:
            return report_sizes[report_id];
        default:
            break;
    }

    return 0;
}

uint8_t *feature_report_get_buf(uint8_t report_id, uint16_t offset)
{
    sequence_t sequence;
    panel_t panel;
    calibrate_pos_t pos;
    sequence_pattern_t pattern;
    
    // Switch based on ReportID
    switch (report_id) {
        case FEATURE_CONFIG:
            feature_get_buf[0] = FEATURE_CONFIG;
            *(uint16_t *)(feature_get_buf + 1) = feature_report_command_id;
            feature_get_buf[3] = g_config.use_raw |
                                 (g_config.calibrate << 1) |
                                 (g_config.use_calibration << 2) |
                                 (g_config.autocalibration << 3) |
                                 (g_config.motion_keep_alive << 4) |
                                 (g_config.command_keep_alive << 5) |
                                 (g_config.use_sensor_coordinates << 6) |
                                 (g_config.override_power << 7);
            feature_get_buf[4] = g_config.interval;
            // We always run at a 1000 Hz sample rate in DK2
            *(uint16_t *)(feature_get_buf+5) = 1000;
            return feature_get_buf;
        
        case FEATURE_CALIBRATE:
            // The DK1 Calibrate Report is over 64 bytes, so we're getting
            // called twice.  We only need to fill the buffer on the first
            // call.
            if (!offset) {
                feature_get_buf[0] = FEATURE_CALIBRATE;
                *(uint16_t *)(feature_get_buf + 1) = feature_report_command_id;
                calibrate_get_calibration_report(feature_get_buf + 3);
            }
            return feature_get_buf + offset;
        
        case FEATURE_RANGE:
            feature_get_buf[0] = FEATURE_RANGE;
            *(uint16_t *)(feature_get_buf + 1) = feature_report_command_id;
            mpu6500_get_ranges(feature_get_buf + 3, (uint16_t *)(feature_get_buf + 4));
            *(uint16_t *)(feature_get_buf + 6) = lis3mdl_get_range();
            return feature_get_buf;
            
        case FEATURE_REGISTER:
            feature_get_buf[0] = FEATURE_REGISTER;
            *(uint16_t *)(feature_get_buf + 1) = feature_report_command_id;
            feature_get_buf[3] = g_register.device;
            feature_get_buf[4] = g_register.reg;
            feature_get_buf[5] = g_register.payload;
            return feature_get_buf;
            
        case FEATURE_DFU:
            feature_get_buf[0] = FEATURE_DFU;
            *(uint16_t *)(feature_get_buf + 1) = feature_report_command_id;
            feature_get_buf[3] = g_dfu.use_dfu;
            return feature_get_buf;
            
        case FEATURE_KEEP_ALIVE:
            feature_get_buf[0] = FEATURE_KEEP_ALIVE;
            *(uint16_t *)(feature_get_buf + 1) = feature_report_command_id;
            *(uint16_t *)(feature_get_buf + 3) = g_keep_alive.keep_alive;
            return feature_get_buf;
            
        case FEATURE_DISPLAY_INFO:
            feature_get_buf[0] = FEATURE_DISPLAY_INFO;
            *(uint16_t *)(feature_get_buf + 1) = feature_report_command_id;
            display_info_get(feature_get_buf + 3);
            return feature_get_buf;
            
        case FEATURE_SERIAL:
            feature_get_buf[0] = FEATURE_SERIAL;
            *(uint16_t *)(feature_get_buf + 1) = feature_report_command_id;
            // Try to grab the set serial if it exists
            if (!config_read(feature_get_buf + 3, CFG_SIZE_SERIAL, CFR_ADDR_SERIAL)) {
                // Otherwise use the first 12 bytes of the UUID
                char uuid[UUID_LEN];
                uuid_get(uuid, 20, 0);
                memcpy(feature_get_buf + 3, uuid, 12);
            }
            return feature_get_buf;
            
        case FEATURE_TRACKING:
            sequence_get_state(&sequence);
            feature_get_buf[0] = FEATURE_TRACKING;
            *(uint16_t *)(feature_get_buf + 1) = feature_report_command_id;
            feature_get_buf[3] = sequence.pattern;
            *(uint16_t *)(feature_get_buf + 4) = sequence.enable | 
                                                (sequence.autoincrement << 1) | 
                                                (sequence.use_carrier << 2) |
                                                (sequence.sync_input << 3) |
                                                (sequence.custom_pattern << 5);
            *(uint16_t *)(feature_get_buf + 6) = sequence.exposure_length;
            *(uint16_t *)(feature_get_buf + 8) = sequence.interval;
            // TODO: do camera/display synchronization
            *(uint16_t *)(feature_get_buf + 10) = 0;
            feature_get_buf[12] = sequence.pulse_duty;
            return feature_get_buf;
            
        case FEATURE_DISPLAY:
            panel_get_state(&panel);
            feature_get_buf[0] = FEATURE_DISPLAY;
            *(uint16_t *)(feature_get_buf + 1) = feature_report_command_id;
            feature_get_buf[3] = panel.brightness;
            *(uint32_t *)(feature_get_buf + 4) = panel.mode | 
                                    (panel.current_limit << 4) | 
                                    (panel.use_rolling << 6) |
                                    (panel.reverse_rolling << 7) |
                                    (panel.high_brightness << 8) |
                                    (panel.self_refresh << 9) |
                                    (panel.read_pixel << 10) |
                                    (panel.direct_pentile << 11);
            *(uint16_t *)(feature_get_buf + 8) = panel.persistence;
            *(uint16_t *)(feature_get_buf + 10) = panel.lighting_offset;
            *(uint16_t *)(feature_get_buf + 12) = panel.pixel_settle;
            *(uint16_t *)(feature_get_buf + 14) = panel.total_rows;
            return feature_get_buf;
            
        case FEATURE_MAGCALIBRATION:
            calibrate_get_mag_calibration(feature_get_buf + 3);
            feature_get_buf[0] = FEATURE_MAGCALIBRATION;
            *(uint16_t *)(feature_get_buf + 1) = feature_report_command_id;
            return feature_get_buf;
            
        case FEATURE_POSCALIBRATION:
            calibrate_get_pos_calibration(&pos);
            feature_get_buf[0] = FEATURE_POSCALIBRATION;
            *(uint16_t *)(feature_get_buf + 1) = feature_report_command_id;
            feature_get_buf[3] = pos.version;
            memcpy(feature_get_buf + 4, pos.position, sizeof(int32_t) * 3);
            memcpy(feature_get_buf + 16, pos.normal, sizeof(int16_t) * 3);
            *(int16_t *)(feature_get_buf + 22) = pos.rotation;
            *(uint16_t *)(feature_get_buf + 24) = pos.index;
            *(uint16_t *)(feature_get_buf + 26) = pos.num_positions;
            *(uint16_t *)(feature_get_buf + 28) = pos.type;
            return feature_get_buf;
            
        case FEATURE_CUSTOMPATTERN:
            sequence_get_custom_pattern(&pattern);
            feature_get_buf[0] = FEATURE_CUSTOMPATTERN;
            *(uint16_t *)(feature_get_buf + 1) = feature_report_command_id;
            feature_get_buf[3] = pattern.sequence_length;
            *(uint32_t *)(feature_get_buf + 4) = pattern.sequence;
            *(uint16_t *)(feature_get_buf + 8) = pattern.led_index;
            *(uint16_t *)(feature_get_buf + 10) = pattern.num_leds;
            return feature_get_buf;
            
        case FEATURE_KEEPALIVEMUX:
            feature_get_buf[0] = FEATURE_KEEPALIVEMUX;
            *(uint16_t *)(feature_get_buf + 1) = feature_report_command_id;
            feature_get_buf[3] = g_keep_alive.in_report;
            *(uint16_t *)(feature_get_buf + 4) = g_keep_alive.keep_alive;
            return feature_get_buf;
            
        case FEATURE_MANUFACTURING:
            feature_get_buf[0] = FEATURE_MANUFACTURING;
            *(uint16_t *)(feature_get_buf + 1) = feature_report_command_id;
            manufacturing_get(feature_get_buf + 3);
            return feature_get_buf;
            
        case FEATURE_UUID:
            feature_get_buf[0] = FEATURE_UUID;
            *(uint16_t *)(feature_get_buf + 1) = feature_report_command_id;
            uuid_get((char *)(feature_get_buf + 3), 20, 0);
            return feature_get_buf;
            
        case FEATURE_TEMPERATURE:
            feature_get_buf[0] = FEATURE_TEMPERATURE;
            *(uint16_t *)(feature_get_buf + 1) = feature_report_command_id;
            temperature_get_report(feature_get_buf + 3);
            return feature_get_buf;
            
        case FEATURE_GYROOFFSET:
            feature_get_buf[0] = FEATURE_GYROOFFSET;
            *(uint16_t *)(feature_get_buf + 1) = feature_report_command_id;
            autocalibrate_get(feature_get_buf);
            return feature_get_buf;
            
        case FEATURE_LENSDISTORTION:
            feature_get_buf[0] = FEATURE_LENSDISTORTION;
            *(uint16_t *)(feature_get_buf + 1) = feature_report_command_id;
            lens_distortion_get(feature_get_buf + 3);
            return feature_get_buf;
            
        default:
            break;
    }

    return NULL;
}

uint8_t *feature_report_set_buf(uint8_t report_id, uint16_t offset)
{
    // Switch based on ReportID
    switch (report_id) {
        case FEATURE_CALIBRATE:
            // Calibrate is over 64 bytes, so there are multiple sets
            return feature_set_buf + offset;
            
        case FEATURE_CONFIG:
        case FEATURE_RANGE:
        case FEATURE_REGISTER:
        case FEATURE_DFU:
        case FEATURE_KEEP_ALIVE:
        case FEATURE_DISPLAY_INFO:
        case FEATURE_SERIAL:
        case FEATURE_TRACKING:
        case FEATURE_DISPLAY:
        case FEATURE_MAGCALIBRATION:
        case FEATURE_POSCALIBRATION:
        case FEATURE_CUSTOMPATTERN:
        case FEATURE_KEEPALIVEMUX:
        case FEATURE_MANUFACTURING:
        case FEATURE_UUID:
        case FEATURE_TEMPERATURE:
        case FEATURE_GYROOFFSET:
        case FEATURE_LENSDISTORTION:
            return feature_set_buf;
        default:
            break;
    }

    return NULL;
}

void feature_report_parse(void)
{
    uint32_t bitmask = 0;
    uint8_t use_uuid = 1;
    calibrate_pos_t pos;
    
    switch (feature_set_buf[0]) {
        case FEATURE_CONFIG:
            g_config.use_raw = feature_set_buf[3] & 1;
            g_config.calibrate = feature_set_buf[3] & (1 << 1);
            g_config.use_calibration = feature_set_buf[3] & (1 << 2);
            g_config.autocalibration = feature_set_buf[3] & (1 << 3);
            g_config.motion_keep_alive = feature_set_buf[3] & (1 << 4);
            g_config.command_keep_alive = feature_set_buf[3] & (1 << 5);
            g_config.use_sensor_coordinates = feature_set_buf[3] & (1 << 6);
            g_config.override_power = feature_set_buf[3] & (1 << 7);
            g_config.interval = feature_set_buf[4];
            // Sample rate is read-only
            g_config_changed = 1;
            break;
        
        case FEATURE_CALIBRATE:
            // Gyro/Accel calibration should never happen during run-time,
            // so set it directly from the interrupt.
            // Note that this can take tens of milliseconds, as it is writing
            // to actual EEPROM
            calibrate_set_calibration_report(feature_set_buf + 3);
            break;
            
        case FEATURE_RANGE:
            g_range.accel_range = mpu6500_closest_accel_range(feature_set_buf[3]);
            g_range.gyro_range = mpu6500_closest_gyro_range(*(uint16_t *)(feature_set_buf + 4));
            g_range.mag_range = lis3mdl_closest_range(*(uint16_t *)(feature_set_buf + 6));
            g_range_changed = 1;
            break;
        
        case FEATURE_REGISTER:
            g_register.device = feature_set_buf[3];
            g_register.reg = feature_set_buf[4];
            g_register.payload = feature_set_buf[5];
            g_register_changed = 1;
            break;
            
        case FEATURE_DFU:
            g_dfu.command_id = *(uint16_t *)(feature_set_buf + 1);
            g_dfu.use_dfu = feature_set_buf[3];
            g_dfu_changed = 1;
            break;
        
        case FEATURE_KEEP_ALIVE:
            // This is the DK1 keep alive report, so set the in report to 1
            g_keep_alive.in_report = 1;
            g_keep_alive.keep_alive = *(uint16_t *)(feature_set_buf + 3);
            g_keep_alive_changed = 1;
            break;
            
        case FEATURE_DISPLAY_INFO:
            display_info_set(feature_set_buf + 3);
            break;
            
        case FEATURE_SERIAL:
            // Only use the real serial number if we have been set to.  This allows these
            // units to go through the production line without re-enumerating on each
            // machine
            config_write(&use_uuid, CFG_SIZE_SERIAL_SET, CFG_ADDR_SERIAL_SET);
            // Also take the specified serial now
            config_write(feature_set_buf + 3, CFG_SIZE_SERIAL, CFR_ADDR_SERIAL);
            break;
            
        case FEATURE_TRACKING:
            g_tracking.pattern = feature_set_buf[3];
            bitmask = *(uint16_t *)(feature_set_buf + 4);
            g_tracking.enable = bitmask & 1;
            g_tracking.autoincrement = bitmask & (1 << 1);
            g_tracking.use_carrier = bitmask & (1 << 2);
            g_tracking.sync_input = bitmask & (1 << 3);
            g_tracking.custom_pattern = bitmask & (1 << 5);
            g_tracking.exposure_length = *(uint16_t *)(feature_set_buf + 6);
            g_tracking.interval = *(uint16_t *)(feature_set_buf + 8);
            // TODO: camera/display sync fields
            g_tracking.pulse_duty = feature_set_buf[12];
            g_tracking_changed = 1;
            break;
            
        case FEATURE_DISPLAY:
            g_panel.brightness = feature_set_buf[3];
            bitmask = *(uint32_t *)(feature_set_buf + 4);
            g_panel.mode = bitmask & 0xF;
            g_panel.current_limit = (bitmask >> 4) & 0x3;
            g_panel.use_rolling = bitmask & (1 << 6);
            g_panel.reverse_rolling = bitmask & (1 << 7);
            g_panel.high_brightness = bitmask & (1 << 8);
            g_panel.self_refresh = bitmask & (1 << 9);
            g_panel.read_pixel = bitmask & (1 << 10);
            g_panel.direct_pentile = bitmask & (1 << 11);
            g_panel.persistence = *(uint16_t *)(feature_set_buf + 8);
            g_panel.lighting_offset = *(uint16_t *)(feature_set_buf + 10);
            g_panel.pixel_settle = *(uint16_t *)(feature_set_buf + 12);
            g_panel.total_rows = *(uint16_t *)(feature_set_buf + 14);
            g_panel_changed = 1;
            break;
            
        case FEATURE_MAGCALIBRATION:
            // Since magnetometer calibration sets should never occur during run-time,
            // write them directly from the handler.
            // Note that this can take up to around 100 milliseconds, as it is writing
            // to actual EEPROM
            calibrate_set_mag_calibration(feature_set_buf + 3);
            break;
            
        case FEATURE_POSCALIBRATION:
            // Since position calibration sets should never occur during run-time,
            // write them directly from the handler.
            // Note that this can take up to around ten milliseconds, as it is writing
            // to actual EEPROM
            pos.version = feature_set_buf[3];
            memcpy(pos.position, feature_set_buf + 4, sizeof(int32_t) * 3);
            memcpy(pos.normal, feature_set_buf + 16, sizeof(int16_t) * 3);
            pos.rotation = *(int16_t *)(feature_set_buf + 22);
            pos.index = *(uint16_t *)(feature_set_buf + 24);
            pos.type = *(uint16_t *)(feature_set_buf + 28);
            // num_positions is ignored, as it is read-only
            calibrate_set_pos_calibration(&pos);
            break;
            
        case FEATURE_CUSTOMPATTERN:
            g_pattern.sequence_length = feature_set_buf[3];
            g_pattern.sequence = *(uint32_t *)(feature_set_buf + 4);
            g_pattern.led_index = *(uint16_t *)(feature_set_buf + 8);
            // num_leds is ignored, as it is read-only
            g_pattern_changed = 1;
            break;
        
        case FEATURE_KEEPALIVEMUX:
            g_keep_alive.in_report = feature_set_buf[3];
            g_keep_alive.keep_alive = *(uint16_t *)(feature_set_buf + 4);
            g_keep_alive_changed = 1;
            break;
            
        case FEATURE_MANUFACTURING:
            // Since setting will only occur in the factory and not at run-time,
            // we can perform the EEPROM write from the interrupt
            manufacturing_set(feature_set_buf + 3);
            break;
            
        case FEATURE_UUID:
            // Only use the real serial number if we have been set to.  This allows these
            // units to go through the production line without re-enumerating on each
            // machine
            config_write(&use_uuid, CFG_SIZE_SERIAL_SET, CFG_ADDR_SERIAL_SET);
            break;
            
        case FEATURE_TEMPERATURE:
            // This shouldn't occur at run time, so write it directly from here.
            // Since this goes to EEPROM, it can take up to 30 ms.
            temperature_set_report(feature_set_buf + 3);
            break;
            
        case FEATURE_GYROOFFSET:
            // Setting does nothing
            break;
            
        case FEATURE_LENSDISTORTION:
            lens_distortion_set(feature_set_buf + 3);
            break;
            
        default:
            break;
    }

    // Get the latest set command ID regardless of report type
    feature_report_command_id = *(uint16_t *)(feature_set_buf+1);
    g_last_report_time = systick_get_tick_count();
    g_report_count++;
}

uint32_t feature_report_count(void)
{
    return g_report_count;
}

uint16_t feature_report_latest_command_id(void)
{
    return feature_report_command_id;
}

uint32_t feature_report_latest_command_time(void)
{
    return g_last_report_time;
}

void feature_report_init_config(feature_config_p config)
{
    // Set both the passed in config and the global one to default settings
    config->use_raw = 0;
    config->calibrate = 0;
    config->use_calibration = 1;
    config->autocalibration = 1;
    config->motion_keep_alive = 0;
    config->command_keep_alive = 1;
    config->use_sensor_coordinates = 0;
    // DK2 defaults to 1000 Hz
    config->interval = 0;
    // The sampling rate is always 1000 Hz
    config->sample_rate = 1000;
    
    // Read the power override from EEPROM
    uint8_t overrides = 0;
    config_read(&overrides, CFG_SIZE_OVERRIDES, CFG_ADDR_OVERRIDES);
    config->override_power = overrides & 1;
    
    // Also copy to the global one used for feature report reads
    memcpy(&g_config, config, sizeof(feature_config_t));
}

void feature_report_init_keep_alive(feature_keep_alive_p keep_alive)
{
    // Default to DK2 report
    keep_alive->in_report = 11;
    // Default to 10 seconds
    keep_alive->keep_alive = 10000;
    
    // Also copy to the global one used for feature report reads
    memcpy(&g_keep_alive, keep_alive, sizeof(feature_keep_alive_t));
}

bool feature_report_get_config(feature_config_p config)
{
    // FIXME: There is potential here for an interrupt to come in while
    // g_config is being copied that will modify g_config
    if (g_config_changed) {
        g_config_changed = 0;
        memcpy(config, &g_config, sizeof(feature_config_t));
        return 1;
    }

    return 0;
}

bool feature_report_get_range(feature_range_p range)
{
    // FIXME: There is potential here for an interrupt to come in while
    // g_range is being copied that will modify g_range
    if (g_range_changed) {
        g_range_changed = 0;
        memcpy(range, &g_range, sizeof(feature_range_t));
        return 1;
    }
    
    return 0;
}

bool feature_report_get_register(feature_register_p reg)
{
    // FIXME: There is potential here for an interrupt to come in while
    // g_register is being copied that will modify g_register
    if (g_register_changed) {
        g_register_changed = 0;
        memcpy(reg, &g_register, sizeof(feature_register_t));
        return 1;
    }
    
    return 0;
}

bool feature_report_get_dfu(feature_dfu_p dfu)
{
    // FIXME: There is potential here for an interrupt to come in while
    // g_dfu is being copied that will modify g_dfu
    if (g_dfu_changed) {
        g_dfu_changed = 0;
        memcpy(dfu, &g_dfu, sizeof(feature_dfu_t));
        return 1;
    }
    
    return 0;
}

bool feature_report_get_keep_alive(feature_keep_alive_p keep_alive)
{
    // FIXME: There is potential here for an interrupt to come in while
    // g_keep_alive is being copied that will modify g_keep_alive
    if (g_keep_alive_changed) {
        g_keep_alive_changed = 0;
        memcpy(keep_alive, &g_keep_alive, sizeof(feature_keep_alive_t));
        return 1;
    }
    
    return 0;
}

bool feature_report_get_tracking(sequence_p sequence)
{
    // FIXME: There is potential here for an interrupt to come in while
    // g_tracking is being copied that will modify g_tracking
    if (g_tracking_changed) {
        g_tracking_changed = 0;
        memcpy(sequence, &g_tracking, sizeof(sequence_t));
        return 1;
    }
    
    return 0;
}

bool feature_report_get_display(panel_p panel)
{
    // FIXME: There is potential here for an interrupt to come in while
    // g_panel is being copied that will modify g_panel
    if (g_panel_changed) {
        g_panel_changed = 0;
        memcpy(panel, &g_panel, sizeof(panel_t));
        return 1;
    }
    
    return 0;
}

bool feature_report_get_pattern(sequence_pattern_p pattern)
{
    // FIXME: There is potential here for an interrupt to come in while
    // g_pattern is being copied that will modify g_pattern
    if (g_pattern_changed) {
        g_pattern_changed = 0;
        memcpy(pattern, &g_pattern, sizeof(sequence_pattern_t));
        return 1;
    }
    
    return 0;
}
