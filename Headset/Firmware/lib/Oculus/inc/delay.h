/************************************************************************************

Filename    :   delay.h
Content     :   Millisecond delay function
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _DELAY_H_
#define _DELAY_H_

#include <stdint.h>

void delay_ms(uint32_t ms);
void delay_update(void);

#endif /* _DELAY_H_ */
