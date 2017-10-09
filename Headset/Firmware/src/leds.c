/************************************************************************************

Filename    :   leds.c
Content     :   Driver for status LEDs.
Created     :
Authors     :   Lyle Bainbridge

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "gpio.h"
#include "hw_config.h"
#include "leds.h"


void leds_init(void)
{
    gpio_config(LED_BLUE_PORT, LED_BLUE_PIN, GPIO_OD);
    led_blue_off();
    led_amber_off();
    gpio_config(LED_AMBER_PORT, LED_AMBER_PIN, GPIO_OD);
    led_amber_off();
}

void led_blue_set(uint8_t set)
{
    if (set) {
        led_blue_on();
    }
    else {
        led_blue_off();
    }
}

void led_blue_on(void)
{
    GPIO_WriteBit(LED_BLUE_PORT, LED_BLUE_PIN, Bit_RESET);
}

void led_blue_off(void)
{
    GPIO_WriteBit(LED_BLUE_PORT, LED_BLUE_PIN, Bit_SET);
}

void led_amber_set(uint8_t set)
{
    if (set) {
        led_amber_on();
    }
    else {
        led_amber_off();
    }
}

void led_amber_on(void)
{
    GPIO_WriteBit(LED_AMBER_PORT, LED_AMBER_PIN, Bit_RESET);
}

void led_amber_off(void)
{
    GPIO_WriteBit(LED_AMBER_PORT, LED_AMBER_PIN, Bit_SET);
}

