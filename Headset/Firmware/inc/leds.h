/************************************************************************************

Filename    :   leds.h
Content     :   Driver for status LEDs.
Created     :
Authors     :   Lyle Bainbridge

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _LEDS_H_
#define _LEDS_H_


#include <stdint.h>

void leds_init(void);
void led_blue_set(uint8_t set);
void led_blue_on(void);
void led_blue_off(void);
void led_amber_set(uint8_t set);
void led_amber_on(void);
void led_amber_off(void);


#endif /* _LEDS_H_ */
