/************************************************************************************

Filename    :   usb.h
Content     :   STM32L USB interface related functions.
Created     :
Authors     :   Lyle Bainbridge

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _USB_H_
#define _USB_H_

#include <stdbool.h>

void usb_init(void);

void usb_deinit(void);

bool usb_ready(void);

#endif /* _USB_H_ */