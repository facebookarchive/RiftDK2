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

#ifndef _IN_REPORT_H_
#define _IN_REPORT_H_

#include <stdbool.h>
#include "sequence.h"
#include "vsync.h"
#include "lis3mdl.h"
#include "mpu6500.h"

enum {
    IN_REPORT_DK1 = 1,
    IN_REPORT_DK2 = 11
};

void in_report_update_id(uint8_t in_report);

void in_report_update_command_id(uint16_t command_id);

void in_report_update_mpu(mpu6500_data_p mpu);

void in_report_update_mag(lis3mdl_data_p mag);

void in_report_update_display(vsync_p vsync);

void in_report_update_tracking(exposure_p exposure);

bool in_report_ready(uint8_t interval);

void in_report_reset(void);

bool in_report_send(void);

#endif /* _IN_REPORT_H_ */
