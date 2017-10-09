/************************************************************************************

Filename    :   sequence_patterns.h
Content     :   Default patterns for tracking LED modulation
Created     :
Authors     :   Nirav Patel, Dov Katz

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _SEQUENCE_PATTERNS_H_
#define _SEQUENCE_PATTERNS_H_

#include <stdint.h>

#define NUM_LEDS (40)
#define NUM_PATTERNS_DEFAULT (10)
#define NUM_PATTERNS_MAX (16)
#define STEADY_STATE_PATTERN (0xFF)

// DK2 LED sequences
static const uint8_t sequence_patterns[NUM_PATTERNS_DEFAULT][NUM_LEDS] = {
    {3,1,1,3,1,3,3,3,1,3,1,3,3,1,1,1,1,3,3,1,3,1,3,1,3,1,3,3,1,1,1,1,3,3,1,1,3,1,3,3},
    {1,3,3,1,1,3,3,3,1,3,3,1,1,3,1,1,1,1,3,1,1,3,1,1,3,3,1,3,1,1,3,3,1,3,1,3,3,1,1,3},
    {1,3,1,3,1,3,1,1,3,3,1,3,1,3,1,3,3,1,1,3,1,1,3,1,3,3,1,1,3,1,1,1,3,1,3,3,3,1,1,1},
    {1,1,3,3,3,3,1,3,3,1,1,1,3,3,1,1,1,1,1,3,3,3,1,1,3,1,3,1,3,1,1,1,1,3,3,3,1,3,1,1},
    {1,1,3,3,1,1,3,1,1,3,1,1,3,3,3,1,3,3,1,1,3,1,3,1,1,3,1,3,3,3,3,1,1,3,3,1,3,1,3,1},
    {1,1,1,1,3,3,3,1,1,1,3,3,3,3,1,3,1,1,1,1,1,3,3,1,1,1,3,3,3,3,1,1,1,1,1,3,3,1,1,3},
    {1,1,1,1,1,1,1,3,3,3,3,3,3,3,1,1,1,3,1,1,1,1,1,3,3,3,3,3,3,1,3,1,1,1,1,1,1,3,3,3},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,3,3,1,1,3,3,3,3,3,3,3,3,3,3,3,1,1,3,3,3,3,3,3,3,3,3},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,3,3,3,3,3,3,3,3,3,3,3,3,3,1,1,1,1,1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,3,3,3,3,3,3,3,3,3,3,3}
};

#endif /* _SEQUENCE_PATTERNS_H_ */
