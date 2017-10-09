/************************************************************************************

Filename    :   in_report.c
Content     :   DK2 IN reports
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "in_report.h"
#include "usb_lib.h"
#include "usb_istr.h"
#include "usb_pwr.h"
#include "report_helper.h"
#include "hub.h"
#include <string.h>
#include <math.h>

#define MAX_IN_REPORT_SIZE 64
#define MAX_NUM_SAMPLES 3
#define SAMPLE_SIZE 16

typedef struct report_format_struct {
    uint8_t report_size;
    uint8_t num_samples_offset;
    uint8_t sample_count_offset;
    uint8_t command_id_offset;
    uint8_t temperature_offset;
    uint8_t sample_offset;
    uint8_t mag_offset;
    uint8_t timestamp_offset;
    uint8_t max_samples;
} report_format_t;

static const report_format_t g_format_dk1 = {
    62, // report_size
    1,  // num_samples_offset
    2,  // sample_count_offset
    4,  // command_id_offset
    6,  // temperature_offset
    8,  // sample_offset
    56, // mag_offset
    0,  // timestamp_offset
    3   // max_samples
};
    
static const report_format_t g_format_dk2 = {
    64, // report_size
    3,  // num_samples_offset
    4,  // sample_count_offset
    1,  // command_id_offset
    6,  // temperature_offset
    12, // sample_offset
    44, // mag_offset
    8,  // timestamp_offset
    2   // max_samples
};

static uint8_t report_buf[MAX_IN_REPORT_SIZE] = {0};
static const report_format_t *g_format;
static bool g_updated = 0;
static uint8_t g_report_id = 0;
static uint8_t num_samples = 0;
static mpu6500_data_t samples[MAX_NUM_SAMPLES] = {{0}};

static void in_report_add_average(mpu6500_data_p sample)
{
    // Just add here.  We will divide out to get the average before sending
    samples[0].gyro[0] += sample->gyro[0];
    samples[0].gyro[1] += sample->gyro[1];
    samples[0].gyro[2] += sample->gyro[2];
    // This is ok as 32-bit because our accelerometer samples are actually
    // only 21-bit and we accumulate a maximum of 254 samples
    samples[0].accel[0] += sample->accel[0];
    samples[0].accel[1] += sample->accel[1];
    samples[0].accel[2] += sample->accel[2];
}

static void in_report_compute_average(void)
{
    // Don't count the samples that are still separate
    int32_t num_avg = num_samples - (g_format->max_samples - 1);

    // calculate the actual averages
    samples[0].gyro[0] /= (float)num_avg;
    samples[0].gyro[1] /= (float)num_avg;
    samples[0].gyro[2] /= (float)num_avg;
    samples[0].accel[0] /= num_avg;
    samples[0].accel[1] /= num_avg;
    samples[0].accel[2] /= num_avg;
}

void in_report_update_id(uint8_t in_report)
{
    // If the in_report is different and valid, change the report format
    if ((in_report != g_report_id) && 
        ((in_report == IN_REPORT_DK1) || (in_report == IN_REPORT_DK2))) {
            
         g_report_id = in_report;
         // Clear the existing report
         in_report_reset();
         memset(report_buf, 0, MAX_IN_REPORT_SIZE);
         report_buf[0] = g_report_id;
         
         // Use the right buffer offsets for new information
         switch (g_report_id) {
             case IN_REPORT_DK1:
                g_format = &g_format_dk1;
                break;
                
             case IN_REPORT_DK2:
                g_format = &g_format_dk2;
                break;
         }  
    }
}

void in_report_update_command_id(uint16_t command_id)
{
    *(uint16_t *)(report_buf + g_format->command_id_offset) = command_id;
}

void in_report_update_mpu(mpu6500_data_p mpu)
{
    // We have two slots to store samples in the IN Report.  The second slot
    // is filled with the new sample if we have one already.  Once we have
    // two or more samples and we get a new one, the first slot becomes a
    // running average, and the new sample goes in the second slot.
    
    if (!num_samples) {
        // Set the sample count off of the first sample
        *(uint16_t *)(report_buf + g_format->sample_count_offset) = mpu->count;
    } else if (num_samples == 254) {
        // Dump the average at 254 samples and just keep the non-averaged samples we have
        num_samples = g_format->max_samples - 1;
        *(uint16_t *)(report_buf + g_format->sample_count_offset) = mpu->count - num_samples;
        // Move the non-averaged samples, overwriting the buffer that was being averaged to
        for (uint8_t i = 0; i < num_samples; i++) {
            memcpy(&samples[i], &samples[i + 1], sizeof(mpu6500_data_t));
        }
    }
    
    // If we already have all sample sample slots full, start averaging
    if (num_samples >= g_format->max_samples) {
        // Add sample 2 to the average
        in_report_add_average(&samples[1]);
        
        // Move samples down the slots
        for (uint8_t i = 1; i < g_format->max_samples - 1; i++) {
            memcpy(&samples[i], &samples[i + 1], sizeof(mpu6500_data_t));
        }
        
        // Move the new sample to the final slot
        memcpy(&samples[g_format->max_samples - 1], mpu, sizeof(mpu6500_data_t));
    } else {
        // If we aren't full, copy into the first available slot
        memcpy(&samples[num_samples], mpu, sizeof(mpu6500_data_t));
    }
    
    // The microsecond timestamp is always that of the newest sample
    if (g_format->timestamp_offset) {
        *(uint32_t *)(report_buf + g_format->timestamp_offset) = mpu->timestamp;
    }
    // Same for temperature
    *(uint16_t *)(report_buf + g_format->temperature_offset) = mpu->temperature;

    num_samples++;    
    g_updated |= 1;
}

void in_report_update_mag(lis3mdl_data_p mag)
{
    // The headset is Z-in, Y-up, X-right from the user, which we convert to
    // at reading the data off the sensor
    memcpy(report_buf + g_format->mag_offset, mag->mag, sizeof(int16_t) * 3);
    g_updated |= 2;
}

void in_report_update_display(vsync_p vsync)
{
    // Only DK2 has display information in the report
    if (g_report_id == IN_REPORT_DK2) {
        *(uint16_t *)(report_buf + 50) = vsync->frame;
        *(uint32_t *)(report_buf + 52) = vsync->timestamp;
        report_buf[56] = vsync->color;
        g_updated |= 3;
    }
}

void in_report_update_tracking(exposure_p exposure)
{
    // Only DK2 has position tracking information in the report
    if (g_report_id == IN_REPORT_DK2) {
        report_buf[57] = exposure->pattern;
        *(uint16_t *)(report_buf + 58) = exposure->frame;
        *(uint32_t *)(report_buf + 60) = exposure->timestamp;
        g_updated |= 4;
    }
}

bool in_report_ready(uint8_t interval)
{
    // We use interval to cut down in the HID IN Report rate.  Since
    // the gyro samples are what we care most about, we use the number ready
    // as the value to check against before sending the packet
    return (GetEPTxStatus(ENDP1) == EP_TX_NAK) && (bDeviceState == CONFIGURED) && 
        g_updated && (num_samples > interval);
}

void in_report_reset(void)
{
    g_updated = 0;
    num_samples = 0;
}

bool in_report_send(void)
{
    report_buf[g_format->num_samples_offset] = num_samples;
    
    if (num_samples > g_format->max_samples) {
        // Compute the average if we've added multiple samples
        in_report_compute_average();
    } else if (num_samples == 1) {
        for (uint8_t i = 1; i < g_format->max_samples; i++) {
            // Clear the remaining sample slots if we only have one
            memset(report_buf + g_format->sample_offset + i * SAMPLE_SIZE, 0, SAMPLE_SIZE);
        }
    }
    
    // Store the gyro/accelerometer samples into the slots in the packet
    for (uint8_t i = 0; i < (num_samples > g_format->max_samples ? g_format->max_samples : num_samples); i++) {
        int32_t x = OFFSET_TO_INT(samples[i].gyro[0]);
        int32_t y = OFFSET_TO_INT(samples[i].gyro[1]);
        int32_t z = OFFSET_TO_INT(samples[i].gyro[2]);
        
        // The headset is Z-in, Y-up, X-right from the user, which we convert to
        // at reading the data off the sensor
        report_pack_sensor(report_buf + g_format->sample_offset + i * SAMPLE_SIZE, samples[i].accel[0], samples[i].accel[1], samples[i].accel[2]);
        report_pack_sensor(report_buf + g_format->sample_offset + i * SAMPLE_SIZE + 8, x, y, z);
    }
    
    USB_SIL_Write(EP1_IN, report_buf, g_format->report_size);
    // Mark that the endpoint has valid data
    SetEPTxValid(ENDP1);
    
    in_report_reset();
    
    return 1;
}
