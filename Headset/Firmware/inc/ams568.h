/************************************************************************************

Filename    :   ams568.h
Content     :   Samsung AMS568 configuration
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _AMS568_H_
#define _AMS568_H_

#include <stdbool.h>
#include "panel.h"

// Support for the Samsung AMS568 5.7" AMOLED
#define DRIVER_S6E3FA0

#define PANEL_H           71
#define PANEL_V           126

enum {
    AMS568_ACL_OFF = 0,
    AMS568_ACL_30 = 1,
    AMS568_ACL_25 = 2,
    AMS568_ACL_50 = 3
};

bool ams568_init(const panel_timing_p timing);

uint8_t ams568_get_default_refresh_rate(void);

uint8_t ams568_num_supported_timings(void);

void ams568_get_supported_timings(uint8_t index, panel_timing_p timing);

bool ams568_configure_brightness(bool use_rolling, bool reverse_rolling, uint16_t lit_rows, uint16_t total_rows, uint8_t brightness, bool use_hbm, uint8_t current_limit);

uint16_t ams568_get_persistence(void);

uint8_t ams568_get_brightness(void);

bool ams568_set_direct_pentile(bool enable);

bool ams568_set_video_mode(bool enable);

#endif /* _AMS568_H_ */
