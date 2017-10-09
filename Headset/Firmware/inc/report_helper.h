/************************************************************************************

Filename    :   report_helper.h
Content     :   Defines and functions to help format some of the HID Reports
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _REPORT_HELPER_H_
#define _REPORT_HELPER_H_

#include <stdint.h>
#include <math.h>

#define REPORT_SENSOR_MAX ((float)((1 << 20) - 1))

#define OFFSET_TO_INT(x) ((int32_t)lroundf(x))
#define SCALE_TO_INT(x) ((int32_t)lroundf((x - 1.0f) * REPORT_SENSOR_MAX))
#define CROSSAXIS_TO_INT(x) ((int32_t)lroundf(x * REPORT_SENSOR_MAX))

#define INT_TO_OFFSET(x) ((float)x)
#define INT_TO_SCALE(x) (((float)x / REPORT_SENSOR_MAX) + 1.0f)
#define INT_TO_CROSSAXIS(x) ((float)x / REPORT_SENSOR_MAX)

void report_pack_sensor(uint8_t *buf, int32_t x, int32_t y, int32_t z);

void report_unpack_sensor(const uint8_t *buf, int32_t *x, int32_t *y, int32_t *z);

#endif /* _REPORT_HELPER_H_ */
