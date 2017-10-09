/************************************************************************************

Filename    :   gamma.h
Content     :   AMS568 gamma correction
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2014-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _GAMMA_H_
#define _GAMMA_H_

#include <stdint.h>

void gamma_generate(uint8_t *gamma, uint8_t *mtp);

#endif /* _GAMMA_H_ */
