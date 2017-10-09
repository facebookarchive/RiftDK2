/************************************************************************************
Filename    :   button.c
Content     :   Button input abstraction
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "button.h"
#include "gpio.h"
#include "systick.h"
#include "hw_config.h"

#define BUTTON_BOUNCE           10

typedef struct button_struct {
    GPIO_TypeDef *port;
    uint32_t pin;
    uint32_t last_raw_change;
    bool last_raw_state;
    bool state;
} button_t, *button_p;

button_t g_power = {0};

void button_init(void)
{
    // Configure the button as a pulled up input
    gpio_config(POWER_BTN_PORT, POWER_BTN_PIN, GPIO_IN_PU);
    g_power.port = POWER_BTN_PORT;
    g_power.pin = POWER_BTN_PIN;
    g_power.last_raw_change = systick_get_tick_count();
    g_power.last_raw_state = g_power.state = GPIO_ReadInputDataBit(g_power.port, g_power.pin);
}

// Return 1 when the state changes
bool button_update(void)
{
    // Get the new raw state of the button
    bool raw_state = GPIO_ReadInputDataBit(g_power.port, g_power.pin);

    // If it changed
    if (raw_state != g_power.last_raw_state) {
        // Store the change time and new state
        g_power.last_raw_change = systick_get_tick_count();
        g_power.last_raw_state = raw_state;
        return 0;
    }

    // If it is changed from the last cleaned up value
    if (g_power.last_raw_state != g_power.state) {
        uint32_t now = systick_get_tick_count();
        // And the value has been unchanged for the debounce period
        if ((now - g_power.last_raw_change) > BUTTON_BOUNCE) {
            // Store the new state and return that the state has changed
            g_power.state = g_power.last_raw_state;
            return 1;
        }
    }

    return 0;
}

uint32_t button_last_change(void)
{
    return g_power.last_raw_change;
}

bool button_raw(void)
{
    // Power button is connected to ground, so active low
    return !g_power.last_raw_state;
}

// Return the actual cleaned up button state
bool button_state(void)
{
    // Power button is connected to ground, so active low
    return !g_power.state;
}
