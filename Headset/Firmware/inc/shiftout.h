/************************************************************************************

Filename    :   shiftout.h
Content     :   Shift register interface
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _SHIFTOUT_H_
#define _SHIFTOUT_H_

#include <stdint.h>

void shiftout_init(void);

void shiftout_shift(const uint8_t *bytes, uint8_t num_bytes);

#endif /* _SHIFTOUT_H_ */
