/************************************************************************************

Filename    :   factory.c
Content     :   Tracker factory calibration interface
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "factory.h"
#include "averaging.h"
#include "calibrate.h"
#include "autocalibrate.h"
#include "gpio.h"
#include "hw_config.h"
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>

// Gravity in m/s^2 * 10^4, in Dongguan, China
#define GRAVITY 97858.8f
#define RPM_TO_RAD(r) (0.104719755f * 10000.0f * r)
#define RPM_TO_NUM_SAMPLES(r) lroundf(((1000.0f * 60.0f) / r) * ceilf(r / (60.0f / 5.0f)))
// RPM in rad/s * 10^4
#define DEFAULT_RPM 78.01f
#define DEFAULT_TURNTABLE RPM_TO_RAD(DEFAULT_RPM)
#define DEFAULT_NUM_SAMPLES 5384

// TODO: make sure these thresholds are appropriate acceptance criteria
// gyro scale should be within 3% of correct
#define GYRO_SCALE_VALID(x) (fabsf(x - 1.0f) < 0.03f)

// cross axis sensitivity should be under 3%
#define GYRO_CROSS_AXIS_VALID(x) (fabsf(x) < 0.03f)

// gyro zero rate should be under 10 degrees/sec
#define GYRO_ZERO_RATE_VALID(x) (fabsf(x) < 1745.33f)

// be more lenient on the accelerometer, since it matters less
// accelerometer scale should be within 8% of correct
#define ACC_SCALE_VALID(x, up) (fabsf((float)x / GRAVITY - (up ? 1.0f : -1.0f)) < 0.08f)

// Be even more lenient on Z, according to David Floco, Z offset can happen
// from board stress
#define ACC_Z_SCALE_VALID(x, up) (fabsf((float)x / GRAVITY - (up ? 1.0f : -1.0f)) < 0.15f)

// accelerometer zero g should be within 120 mg
#define ACC_ZERO_G_VALID(x) (fabsf((float)x) < (GRAVITY * 120.0f / 1000.0f))

// accelerometer Z zero g should be within 180 mg
#define ACC_Z_ZERO_G_VALID(x) (fabsf((float)x) < (GRAVITY * 180.0f / 1000.0f))

enum factory_state {
    FACTORY_NONE = 0,
    FACTORY_ZDOWN = 1,
    FACTORY_ZUP = 2,
    FACTORY_ZROT = 3,
    FACTORY_YDOWN = 4,
    FACTORY_YUP = 5,
    FACTORY_YROT = 6,
    FACTORY_XDOWN = 7,
    FACTORY_XUP = 8,
    FACTORY_XROT = 9,
    FACTORY_COMPLETE = 'c'
};

enum factory_commands {
    // Commands to the Tracker
    FACTORY_STATUS = 's',
    FACTORY_DEBUG = 'd',
    FACTORY_TABLE = 't',
    // Responses from the Tracker
    FACTORY_BUSY = 'b',
    FACTORY_OUT_OF_ORDER = 'o',
    FACTORY_OUT_OF_RANGE = 'r',
    FACTORY_UNKNOWN = 'u',
    FACTORY_FINISHED = 'f',
};

static uint8_t current_stage = FACTORY_NONE;
static uint16_t stage_bitmask = 0;
static bool debug_mode = 0;

// variables for getting the rpm in over serial
static bool table_command_mode = 0;
static float table_rpm = DEFAULT_TURNTABLE;
static uint32_t desired_samples = DEFAULT_NUM_SAMPLES;
#define SCRATCH_LEN 16
static uint8_t table_scratch[SCRATCH_LEN] = {0};
static uint8_t table_scratch_pos = 0;
static bool table_saw_period = 0;

static averaging_t avg = {0};
static calibrate_t cal = {0};
static int32_t accel_scratch[6][3] = {{0}};

static bool factory_static_stage(uint8_t axis, bool up)
{
    int32_t accel[3];
    // Recompute the gyro offset each time as it may be changing
    // as the gyro warms up during the test
    averaging_compute(&avg, cal.gyro_offset, accel, &cal.temperature);

    for (uint8_t i = 0; i < 3; i++) {
        // always check if the gyro was still
        if (!GYRO_ZERO_RATE_VALID(cal.gyro_offset[i]))
            return 0;

        // check both the 1 g and 0 g axes for valid range
        if (i == axis) {
            if ((i == 2) ? !ACC_Z_SCALE_VALID(accel[axis], up) : !ACC_SCALE_VALID(accel[axis], up))
                return 0;
        } else {
            if ((i == 2) ? !ACC_Z_ZERO_G_VALID(accel[i]) : !ACC_ZERO_G_VALID(accel[i]))
                return 0;
        }
    }

    memcpy(accel_scratch[axis * 2 + !up], accel, sizeof(int32_t) * 3);

    return 1;
}

static bool factory_gyro_scale_stage(uint8_t axis)
{
    float gyro[3];
    float sign = 1.0f;

    averaging_compute(&avg, gyro, NULL, NULL);
    // flip the sign if it got rotated backwards
    if (gyro[axis] < 0.0f)
        sign = -1.0f;

    for (uint8_t i = 0; i < 3; i++) {
        if (i == axis) {
            // the diagonal of the matrix is gyro scale
            cal.gyro_scale[axis][axis] = table_rpm / (sign * (gyro[axis] - cal.gyro_offset[axis]));
            // make sure the value is sane
            if (!GYRO_SCALE_VALID(cal.gyro_scale[axis][axis]))
                return 0;
        } else {
            // the rest is cross axis sensitivity
            cal.gyro_scale[i][axis] = -(gyro[i] - cal.gyro_offset[i]) / gyro[axis];
            if (!GYRO_CROSS_AXIS_VALID(cal.gyro_scale[i][axis]))
                return 0;
        }
    }

    return 1;
}

static bool factory_stage_complete(void)
{
    bool ret = 1;

    switch (current_stage) {
        case FACTORY_ZDOWN:
            ret = factory_static_stage(2, 0);
            break;

        case FACTORY_ZUP:
            ret = factory_static_stage(2, 1);
            break;

        case FACTORY_ZROT:
            ret = factory_gyro_scale_stage(2);
            break;

        case FACTORY_YDOWN:
            ret = factory_static_stage(1, 0);
            break;

        case FACTORY_YUP:
            ret = factory_static_stage(1, 1);
            break;

        case FACTORY_YROT:
            ret = factory_gyro_scale_stage(1);
            break;

        case FACTORY_XDOWN:
            ret = factory_static_stage(0, 0);
            break;

        case FACTORY_XUP:
            ret = factory_static_stage(0, 1);
            break;

        case FACTORY_XROT:
            ret = factory_gyro_scale_stage(0);
            break;

        default:
            break;
    }

    return ret;
}

static uint8_t map_ascii(uint8_t stage)
{
    // lowercase any uppercase command
    stage = tolower(stage);

    // if we get ascii numbers for the stage, convert to actual int
    if ((stage >= '0') && (stage <= '9')) {
        stage = stage - (uint8_t)'0';
    }

    return stage;
}

static uint8_t factory_new_stage(uint8_t new_stage, uint8_t original_command)
{
    // ack with the stage that is being set, preserving ascii/int choice
    uint8_t ret = original_command;

    switch (new_stage) {
        // all stages require Z up or down to have been completed
        case FACTORY_ZROT:
        case FACTORY_YDOWN:
        case FACTORY_YUP:
        case FACTORY_YROT:
        case FACTORY_XDOWN:
        case FACTORY_XUP:
        case FACTORY_XROT:
            if (!(stage_bitmask & ((1 << FACTORY_ZDOWN) | (1 << FACTORY_ZUP))))
                ret = FACTORY_OUT_OF_ORDER;
            else
                current_stage = new_stage;
            break;

        default:
            current_stage = new_stage;
            break;
    }

    return ret;
}

static void compute_accel_matrix(void)
{
    for (uint8_t i = 0; i < 3; i++) {
        // Average the +1 and -1 g measurements
        int32_t up = accel_scratch[i * 2][i];
        int32_t down = accel_scratch[i * 2 + 1][i];

        cal.acc_offset[i] =  (up + down) / 2;
    }

    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            if (i == j) {
                // Inverse of scale error
                cal.acc_scale[i][j] = GRAVITY / (float)(accel_scratch[i * 2][i] - cal.acc_offset[i]);
            } else {
                // Negative of cross-axis error
                cal.acc_scale[i][j] = (float)-(accel_scratch[j * 2][i] - cal.acc_offset[i]) / (float)accel_scratch[j * 2][j];
            }
        }
    }
}

static uint8_t factory_complete(uint8_t command)
{
    uint8_t ret = command;

    if (stage_bitmask != 0x3FE) {
        ret = FACTORY_OUT_OF_ORDER;
    } else {
        compute_accel_matrix();
        calibrate_from_factory(&cal);
        current_stage = FACTORY_COMPLETE;
    }

    return ret;
}

static void reset_table_rpm(void)
{
    table_command_mode = 0;
    table_scratch_pos = 0;
    table_saw_period = 0;
    memset(table_scratch, 0, SCRATCH_LEN);
}

static uint8_t parse_table_rpm(uint8_t new_command)
{
    uint8_t ret = 0;

    if ((new_command == '\n') || (new_command == '\r')) {
        float rpm = atof((char *)table_scratch);

        // make sure it is some sane number
        if ((rpm > 75.0f) && (rpm < 2010.0f)) {
            table_rpm = RPM_TO_RAD(rpm);
            desired_samples = RPM_TO_NUM_SAMPLES(rpm);
            ret = FACTORY_FINISHED;
        } else {
            ret = FACTORY_OUT_OF_RANGE;
        }

        reset_table_rpm();
    } else if (isdigit(new_command) || (!table_saw_period && (new_command == '.'))) {
        // if it is a valid character and there is space in the buffer, append it
        // only allow one period
        if (new_command == '.')
            table_saw_period = 1;

        if (table_scratch_pos < SCRATCH_LEN)
            table_scratch[table_scratch_pos++] = new_command;
    } else {
        // fail if we got an unexpected character
        reset_table_rpm();
        ret = FACTORY_UNKNOWN;
    }

    return ret;
}

bool factory_check_enable(void)
{
    // Configure the enable calibration pin, check if it is being pulled high
    // by a calibration fixture, and then set it back to floating
    gpio_config(CAL_EN_PORT, CAL_EN_PIN, GPIO_IN_PD);
    bool ret = GPIO_ReadInputDataBit(CAL_EN_PORT, CAL_EN_PIN);
    gpio_config(CAL_EN_PORT, CAL_EN_PIN, GPIO_AN);
    
    return ret;
}

void factory_set_command(uint8_t new_command)
{
    uint8_t ret = FACTORY_UNKNOWN;
    uint8_t original_command = new_command;
    new_command = map_ascii(new_command);

    // handle the rpm parsing seperately
    if (table_command_mode) {
        ret = parse_table_rpm(original_command);
    } else if (new_command == FACTORY_TABLE) {
        ret = original_command;
        table_command_mode = 1;
    } else if ((new_command == '\n') || (new_command == '\r')) {
        // echo carriage returns and line feeds
        ret = original_command;
    } else if (new_command == FACTORY_STATUS) {
        if (current_stage == FACTORY_COMPLETE) {
            ret = FACTORY_COMPLETE;
        } else {
            // return as ascii
            ret = current_stage + (uint8_t)'0';
        }
    } else if (new_command == FACTORY_DEBUG) {
        debug_mode = !debug_mode;
        ret = new_command;
    } else if (current_stage != FACTORY_NONE) {
        // send busy if we don't have enough samples
        ret = FACTORY_BUSY;
    } else if (new_command == FACTORY_COMPLETE) {
        ret = factory_complete(original_command);
    } else if (new_command <= FACTORY_XROT) {
        ret = factory_new_stage(new_command, original_command);
    }

    if (ret)
        putchar(ret);
}

static void factory_debug_print(float *gyro, int32_t *acc, int16_t temp)
{
    printf("Debug Output at %f\r\nGyro:\r\n[", temp / 100.0f);
    // print out the gyro axes
    for (uint8_t i = 0; i < 3; i++) {
        printf("%f, ", gyro[i]);
    }

    printf("]\r\nAccel:\r\n[");
    // print out the accel axes
    for (uint8_t i = 0; i < 3; i++) {
        // print in G only for the axis facing gravity
        if (abs(acc[i]) > GRAVITY / 2.0f)
            printf("%f, ", (float)acc[i] / GRAVITY);
        else 
            printf("%d, ", acc[i]);
    }

    printf("]\r\n");
}

static void factory_debug_out_of_range(void)
{
    float gyro[3];
    int32_t acc[3];
    int16_t temp;
    averaging_compute(&avg, gyro, acc, &temp);

    for (uint8_t i = 0; i < 3; i++) {
        // print in ratio to actual only for the spinning axis
        if (fabsf(gyro[i]) > table_rpm / 2.0f)
            gyro[i] = gyro[i] / table_rpm;
    }

    factory_debug_print(gyro, acc, temp);
}

static void factory_debug_rpm(mpu6500_data_p data)
{
    static averaging_t debug_avg = {0};

    mpu6500_data_t converted;
    memcpy(&converted, data, sizeof(mpu6500_data_t));
    autocalibrate_update(&converted, 1, 0);
    calibrate_apply(&converted);

    averaging_update(&debug_avg, &converted);

    if (averaging_count(&debug_avg) < 2000)
        return;

    float gyro[3];
    int32_t acc[3];
    int16_t temp;
    averaging_compute(&debug_avg, gyro, acc, &temp);
    averaging_reset(&debug_avg);

    for (uint8_t i = 0; i < 3; i++) {
        // Convert all axes to RPM
        gyro[i] = gyro[i] * 0.00095492965f;
    }

    factory_debug_print(gyro, acc, temp);
}

void factory_update(mpu6500_data_p data)
{
    if ((current_stage == FACTORY_NONE) || (current_stage == FACTORY_COMPLETE)) {
        if (debug_mode)
            factory_debug_rpm(data);
        return;
    }

    averaging_update(&avg, data);

    if (averaging_count(&avg) >= desired_samples) {
        uint8_t command = FACTORY_FINISHED;

        if (factory_stage_complete()) {
            // mark that this stage has been done
            stage_bitmask |= (1 << current_stage);
        } else {
            command = FACTORY_OUT_OF_RANGE;
            if (debug_mode)
                factory_debug_out_of_range();
        }

        current_stage = FACTORY_NONE;

        averaging_reset(&avg);

        // tell the host that the stage is complete or failed
        putchar(command);
    }
}
