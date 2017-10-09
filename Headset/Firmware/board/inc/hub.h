/************************************************************************************

Filename    :   hub.h
Content     :   
Created     :
Authors     :   Lyle Bainbridge

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _HUB_H_
#define _HUB_H_

#include <stdbool.h>

void hub_init(bool override_power);

void hub_post_init(void);

void hub_accessory_power_fault_irq(void);

bool hub_get_self_powered(void);

void hub_check_ww(void);

void hub_accessory_led_update(bool suspended);

#endif /* _HUB_H_ */
