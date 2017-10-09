/************************************************************************************
Filename    :   vsync.c
Content     :   Measurement of frame timing and average refresh rate
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "vsync.h"
#include "timestamper.h"
#include "gpio.h"
#include "hw_config.h"
#include <string.h>
#include <math.h>

// This is a power of two to make the modulo just a bitwise AND
#define VSYNC_BUFFER_SLOTS 4
#define SLOT_FOR_FRAME(x) ((x) & (VSYNC_BUFFER_SLOTS - 1))

// Use a ring buffer to store vsyncs to avoid memory access issues between
// the main thread and the interrupt that stores the data
static vsync_t vsync_buffer[VSYNC_BUFFER_SLOTS] = {{0}};
static volatile uint32_t g_last_frame = 0;
static uint32_t g_average_count = 0;
static uint32_t g_average_interval = 0;

void vsync_init(void)
{
    // Enable input capture of vsync
    gpio_config(PANEL_VSYNC_PORT, PANEL_VSYNC_PIN, GPIO_AF);
    GPIO_PinAFConfig(PANEL_VSYNC_PORT, PANEL_VSYNC_SOURCE, PANEL_VSYNC_AF);
    
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICInitStructure.TIM_Channel = PANEL_VSYNC_CHANNEL;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    TIM_ICInit(PANEL_VSYNC_TIM, &TIM_ICInitStructure);
    
    // NVIC is already set up for this timer in timestamper_init, so don't reinit
}

void vsync_enable(bool state)
{
    TIM_ClearITPendingBit(PANEL_VSYNC_TIM, PANEL_VSYNC_FLAG);
    TIM_ITConfig(PANEL_VSYNC_TIM, PANEL_VSYNC_FLAG, state ? ENABLE : DISABLE);
}

bool vsync_get(vsync_p vsync)
{
    if (vsync->frame != g_last_frame) {
        // Grab the most recent exposure from the ring buffer
        memcpy(vsync, &vsync_buffer[SLOT_FOR_FRAME(g_last_frame)], sizeof(vsync_t));
        return 1;
    }
    
    return 0;
}

bool vsync_get_frame(vsync_p vsync, uint16_t frame)
{
    // Grab a specific exposure from the ring buffer
    memcpy(vsync, &vsync_buffer[SLOT_FOR_FRAME(frame)], sizeof(vsync_t));
    // Make sure it actually matches
    return (vsync->frame == frame);
}

void vsync_update(bool rolled)
{    
    // Grab the buffer to store vsync information hopefully atomically
    vsync_p vsync = &vsync_buffer[SLOT_FOR_FRAME(g_last_frame + 1)];
    vsync->frame = g_last_frame + 1;
    
    // Get the high 16 bits from the 32 bit timer
    uint32_t now = timestamper_get_time();
    // Get the low 16 bits directly from the capture
    vsync->timestamp = (now & 0xFFFF0000) | TIM_GetCapture2(PANEL_VSYNC_TIM);
    
    // If the capture value is greater than the current timer value
    // and we know that the timer overflow counter was incremented,
    // it means the capture happened before the roll and the overflow increment
    // needs to be removed.
    if (rolled && ((now & 0xFFFF) < (vsync->timestamp & 0xFFFF))) {
        vsync->timestamp -= 0x10000;
    }
    
    // The top left pixel color gets read from the main loop to avoid
    // blocking other stuff during a high priority interrupt
    g_last_frame++;
}

uint32_t vsync_get_average(void)
{
    // Return the average microsecond interval
    return g_average_interval;
}

void vsync_reset_average(void)
{
    // Reset the average on loss of sync
    g_average_count = 0;
    g_average_interval = 0;
}

void vsync_update_average(void)
{
    // If this is the second frame or later
    if (g_average_count) {
        uint32_t frame = g_last_frame;
        // Grab the time elapsed between the new frame and the previous
        uint32_t interval = vsync_buffer[SLOT_FOR_FRAME(frame)].timestamp - 
                            vsync_buffer[SLOT_FOR_FRAME(frame - 1)].timestamp;
        
        // Update the running average.  On the first interval, this
        // just sets the average to that interval
        g_average_interval = (interval + g_average_interval * (g_average_count - 1)) / g_average_count;
    }
    
    // Cap the count so the averaging happens over a reasonable time frame
    if (g_average_count < 128)
        g_average_count++;
}

uint16_t vsync_get_refresh(void)
{
    // Calculate the nearest refresh rate once we've had enough frames to
    // get a good average
    if (g_average_count > 8)
        return lroundf(1000000.0F/(float)g_average_interval);
    else
        return 0;
}
