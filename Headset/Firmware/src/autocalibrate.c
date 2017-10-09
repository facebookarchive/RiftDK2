/************************************************************************************

Filename    :   autocalibrate.c
Content     :   Gyro run time zero-rate calibration
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "autocalibrate.h"
#include "averaging.h"
#include "calibrate.h"
#include "timestamper.h"
#include "report_helper.h"
#include <string.h>
#include <math.h>

enum {
    OFFSET_NONE = 0,
    OFFSET_MOVEMENT = 1,
    OFFSET_TIMEOUT = 2
};

#define AUTOCALIBRATE_NUM_SAMPLES 6000
#define AUTOCALIBRATE_LIMIT 3490.66f
#define AUTOCALIBRATE_NOISE 300.0f
#define AUTOCALIBRATE_ALPHA 0.4f

static uint32_t no_motion_count = 0;
static averaging_t avg = {0};
static float moving_avg[3] = {0.0f};

typedef struct autocalibrate_offset_struct {
    float offset[3];
    uint32_t timestamp;
    int16_t temperature;
    uint8_t version;
} autocalibrate_offset_t, *autocalibrate_offset_p;

static autocalibrate_offset_t last_offset[2] = {{0}};
uint32_t last_offset_count = 0;

uint32_t autocalibrate_motion_count(void)
{
    return no_motion_count;
}

void autocalibrate_get(uint8_t *buf)
{
    int32_t rounded[3];
    // Get the pointer to the last full struct in the ring buffer
    autocalibrate_offset_p offset = &last_offset[last_offset_count & 1];
    // Round gyro offset to the nearest int
    rounded[0] = lroundf(offset->offset[0]);
    rounded[1] = lroundf(offset->offset[1]);
    rounded[2] = lroundf(offset->offset[2]);
    // Pack it into the 8 byte field
    report_pack_sensor(buf + 4, rounded[0], rounded[1], rounded[2]);
    
    buf[3] = offset->version;
    *(uint32_t *)(buf + 12) = offset->timestamp;
    *(int16_t *)(buf + 16) = offset->temperature;
}

static void autocalibrate_complete(bool autocalibration, uint8_t version)
{
    // Get the next struct in the ring buffer
    autocalibrate_offset_p offset = &last_offset[(last_offset_count + 1) & 1];
    
    // Compute the gyro offset by averaging the samples
    averaging_compute(&avg, offset->offset, NULL, &offset->temperature);
    offset->timestamp = timestamper_get_time();
    offset->version = version;
    // Increment the ring buffer read counter
    last_offset_count++;
    
    // Use the offset for the IN Reports only if autocalibration is enabled
    if (autocalibration) {
        calibrate_set_offset(offset->offset, offset->temperature);
    }
}

bool autocalibrate_update(mpu6500_data_p data, bool autocalibration, bool allow_store)
{
    if (!avg.sample_count) {
        // start the calibration with the sample
        averaging_update(&avg, data);
    } else {
        uint8_t i;
        // check if any gyro axis is seeing motion
        for (i = 0; i < 3; i++) {
            // do a moving average to reject short term noise
            moving_avg[i] = data->gyro[i] * AUTOCALIBRATE_ALPHA + moving_avg[i] * (1.0f - AUTOCALIBRATE_ALPHA);

            // Make sure the absolute value is below what is likely motion
            if (fabsf(moving_avg[i]) < AUTOCALIBRATE_LIMIT) {
                float a = (float)(avg.gyro[i] / (double)avg.sample_count);
                // Make sure it is close enough to the current average
                // that it is probably noise and not motion
                if (fabsf(moving_avg[i] - a) < AUTOCALIBRATE_NOISE) {
                    continue;
                }
            }

            // Calculate the offset if we had enough samples, but indicate
            // that it ended due to movement rather than time
            if (avg.sample_count > (AUTOCALIBRATE_NUM_SAMPLES / 2)) {
                autocalibrate_complete(autocalibration, OFFSET_MOVEMENT);
            }

            // if it failed either threshold, reset calibration and return
            averaging_reset(&avg);
            no_motion_count = 0;
            return 0;
        }

        // add the probable no motion sample
        averaging_update(&avg, data);
        no_motion_count++;

        // After ~6 seconds of no motion calculate the zero rate offset
        // to avoid having the offset cover too wide of a temperature range
        if (avg.sample_count > AUTOCALIBRATE_NUM_SAMPLES) {
            autocalibrate_complete(autocalibration, OFFSET_TIMEOUT);
            averaging_reset(&avg);

            return 1;
        }
    }

    return 0;
}
