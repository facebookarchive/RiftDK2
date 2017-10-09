/************************************************************************************
Filename    :   button.h
Content     :   Button input abstraction
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _BUTTON_H_
#define _BUTTON_H_

#include <stdint.h>
#include <stdbool.h>

void button_init(void);

bool button_update(void);

uint32_t button_last_change(void);

bool button_raw(void);

bool button_state(void);

#endif /* _BUTTON_H_ */
