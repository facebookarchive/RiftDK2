/************************************************************************************

Filename    :   display_info.h
Content     :   Headset display configuration storage
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _DISPLAY_INFO_H_
#define _DISPLAY_INFO_H_

#include <stdint.h>

void display_info_get(uint8_t *buf);

void display_info_set(uint8_t *buf);

#endif /* _DISPLAY_INFO_H_ */
