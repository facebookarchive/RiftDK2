/************************************************************************************

Filename    :   manufacturing.h
Content     :   Manufacturing information storage
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _MANUFACTURING_H_
#define _MANUFACTURING_H_

#include <stdint.h>

void manufacturing_get(uint8_t *buf);

void manufacturing_set(uint8_t *buf);

#endif /* _MANUFACTURING_H_ */
