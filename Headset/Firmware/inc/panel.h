/************************************************************************************

Filename    :   panel.h
Content     :   Panel configuration wrapper
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _PANEL_H_
#define _PANEL_H_

#include <stdint.h>
#include <stdbool.h>
#include "vsync.h"

enum {
    PANEL_16BPP = 0x50,
    PANEL_18BPP = 0x60,
    PANEL_24BPP = 0x70
};

// From OVR_DeviceConstants.h
enum {
    LIGHT_MODE_GLOBAL,
    LIGHT_MODE_ROLLING_TOP_BOTTOM,
    LIGHT_MODE_ROLLING_LEFT_RIGHT,
    LIGHT_MODE_ROLLING_RIGHT_LEFT,
    LIGHT_MODE_MAX
};

typedef struct panel_timing_struct {
    uint16_t h_active; // in pixels
    uint16_t h_front;
    uint16_t h_pulse;
    uint16_t h_back;
    uint16_t h_total;
    uint16_t v_active;
    uint16_t v_front;
    uint16_t v_pulse;
    uint16_t v_back;
    uint16_t v_total;
    uint16_t refresh;
    uint16_t pwm;
} panel_timing_t, *panel_timing_p;

typedef struct panel_struct {
    uint16_t persistence;
    uint16_t lighting_offset;
    uint16_t pixel_settle;
    uint16_t total_rows;
    uint8_t brightness;
    uint8_t mode;
    uint8_t current_limit;
    bool use_rolling;
    bool reverse_rolling;
    bool high_brightness;
    bool self_refresh;
    bool read_pixel;
    bool direct_pentile;
} panel_t, *panel_p;

void panel_init(void);

void panel_reset_state(void);

void panel_get_state(panel_p panel);

bool panel_set_state(panel_p panel);

void panel_power_on(void);

void panel_power_off(void);

uint8_t panel_num_timings(void);

void panel_get_timing(uint8_t index, panel_timing_p timing);

uint8_t panel_current_timing(panel_timing_p timing);

uint8_t panel_get_closest_timing(uint16_t refresh, uint16_t h, uint16_t v);

uint8_t panel_update_timing(uint16_t refresh, uint16_t h, uint16_t v);

bool panel_resolution_changed(uint16_t h, uint16_t v);

void panel_get_size(uint8_t *h, uint8_t *v);

bool panel_enable(bool state);

bool panel_enabled(void);

bool panel_get_vsync(vsync_p vsync);

uint16_t panel_get_refresh(void);

#endif /* _PANEL_H_ */
