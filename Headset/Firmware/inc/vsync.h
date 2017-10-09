/************************************************************************************
Filename    :   vsync.h
Content     :   Measurement of frame timing and average refresh rate
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _VSYNC_H_
#define _VSYNC_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct vsync_struct {
    uint32_t timestamp;
    uint16_t frame;
    uint8_t color;
} vsync_t, *vsync_p;

void vsync_init(void);

void vsync_enable(bool state);

bool vsync_get(vsync_p vsync);

bool vsync_get_frame(vsync_p vsync, uint16_t frame);

void vsync_update(bool rolled);

uint32_t vsync_get_average(void);

void vsync_reset_average(void);

void vsync_update_average(void);

uint16_t vsync_get_refresh(void);

#endif /* _VSYNC_H_ */
