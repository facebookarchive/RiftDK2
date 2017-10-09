/************************************************************************************

Filename    :   systick.c
Content     :   Driver for the Cortex systick timer.
Created     :
Authors     :   Lyle Bainbridge

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _SYSTICK_H_
#define _SYSTICK_H_

#include <stdint.h>

void     systick_init(void);

void     systick_disable(void);

void     systick_reset_tick_count(void);

uint32_t systick_get_tick_count(void);

void     systick_update(void);

#endif /* _SYSTICK_H_ */
