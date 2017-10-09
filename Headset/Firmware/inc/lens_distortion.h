/************************************************************************************

Filename    :   lens_distortion.h
Content     :   Storage of lens distortion parameters
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2014-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _LENS_DISTORTION_H_
#define _LENS_DISTORTION_H_

#include <stdint.h>

void lens_distortion_get(uint8_t *buf);

void lens_distortion_set(uint8_t *buf);

#endif /* _LENS_DISTORTION_H_ */
