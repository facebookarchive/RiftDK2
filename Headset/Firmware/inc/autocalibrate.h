/************************************************************************************

Filename    :   autocalibrate.h
Content     :   Gyro run time zero-rate calibration
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _AUTOCALIBRATE_H_
#define _AUTOCALIBRATE_H_

#include <stdint.h>
#include <stdbool.h>
#include "mpu6500.h"

uint32_t autocalibrate_motion_count(void);

void autocalibrate_get(uint8_t *buf);

bool autocalibrate_update(mpu6500_data_p data, bool autocalibration, bool allow_store);

#endif /* _AUTOCALIBRATE_H_ */
