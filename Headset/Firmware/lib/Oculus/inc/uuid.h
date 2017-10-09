/************************************************************************************

Filename    :   uuid.h
Content     :   Headset serial number interface
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _UUID_H_
#define _UUID_H_

#include <stdint.h>
#include <stdbool.h>

#define UUID_LEN 20

void uuid_init(void);

void uuid_get(char *uuid_string, uint8_t len, bool use_unicode);

bool uuid_use_for_usb(void);

#endif /* _UUID_H_ */
