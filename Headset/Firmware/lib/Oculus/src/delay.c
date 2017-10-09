/************************************************************************************

Filename    :   delay.c
Content     :   Millisecond delay function
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "delay.h"

static volatile uint32_t g_delay_ticks = 0;


void delay_ms(uint32_t ms)
{
    // TODO: Add 1 to avoid not waiting long enough due to tick alignment?
    g_delay_ticks = ms;

    while (g_delay_ticks > 0) {
    }//__WFI();
}

void delay_update(void)
{
    if (g_delay_ticks > 0) {
        g_delay_ticks--;
    }
}
