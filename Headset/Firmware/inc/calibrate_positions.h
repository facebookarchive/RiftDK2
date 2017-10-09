/************************************************************************************

Filename    :   calibrate_positions.h
Content     :   Position calibration defaults
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2014-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _CALIBRATE_POSITIONS_H_
#define _CALIBRATE_POSITIONS_H_

#include "sequence_patterns.h"
#include <stdint.h>

enum {
    CALIBRATE_VERSION_NONE = 0,
    CALIBRATE_VERSION_DEFAULT = 1,
    CALIBRATE_VERSION_FACTORY = 2,
    CALIBRATE_VERSION_USER = 3,
};

enum {
    CALIBRATE_POSITION_LED = 0,
    CALIBRATE_POSITION_IMU = 1,
};

#define NUM_POSITIONS (NUM_LEDS + 1)

// Default positions for the LEDs and IMU on DK2
static const int32_t calibrate_default_positions[NUM_POSITIONS][3] = {
    {-27362,     -48634,     -22830},
    {-55762,     -46600,     -14507},
    {-37990,     -41637,      11711},
    {-56862,     -19886,      16428},
    {-78436,     -35918,      10008},
    {-87943,     -27452,     -14831},
    {-75427,     -45222,     -43164},
    {-88863,     -21454,     -48989},
    {-89397,        127,     -15706},
    {-88606,      25445,     -48946},
    {-74755,      46289,     -42967},
    {-87890,      27818,     -14583},
    {-83856,        141,      10811},
    {-78289,      36772,       9933},
    {-56600,      20089,      15496},
    {-56002,      47434,     -14789},
    {-28754,      49337,     -23028},
    {-28880,      43133,      10796},
    {-31247,        111,      17894},
    {-114  ,     -42040,      12566},
    {30    ,      -9698,      18075},
    {31265 ,         78,      17831},
    {28822 ,      43150,      10812},
    {28987 ,      49222,     -23113},
    {56053 ,      47272,     -14639},
    {56673 ,      20042,      15531},
    {78389 ,      36622,       9919},
    {83875 ,        -57,      10791},
    {87878 ,      27788,     -14648},
    {74475 ,      46103,     -42738},
    {88661 ,      25638,     -48780},
    {89273 ,        120,     -15734},
    {88913 ,     -21416,     -49069},
    {75576 ,     -45229,     -43318},
    {87725 ,     -27557,     -14987},
    {78438 ,     -35858,      10162},
    {56947 ,     -19846,      16527},
    {37921 ,     -41691,      11687},
    {55717 ,     -46678,     -14664},
    {27315 ,     -48583,     -22960},
    {67100 ,     -24100,       8545}
};

// Default normals for the LEDs and IMU on DK2
static const int16_t calibrate_default_normals[NUM_POSITIONS][3] = {
    {-54,       -1198,      36},
    {-85,       -1196,      42},
    {-21,       -592,       537},
    {0,         0,          800},
    {-485,      -461,       438},
    {-796,      -68,        27},
    {-157,      -682,       14},
    {-699,      -41,        3},
    {-1200,     0,          38},
    {-796,      72,         3},
    {-219,      769,        15},
    {-1195,     106,        39},
    {-658,      0,          454},
    {-485,      461,        439},
    {0,         13,         1100},
    {-85,       1196,       47},
    {-26,       699,        23},
    {-9,        641,        478},
    {0,         0,          1200},
    {0,         -592,       538},
    {0,         0,          1200},
    {0,         0,          1200},
    {8,         636,        486},
    {26,        699,        23},
    {85,        1196,       47},
    {0,         13,         1100},
    {485,       461,        439},
    {987,       0,          682},
    {1195,      106,        39},
    {219,       769,        15},
    {796,       72,         3},
    {1200,      0,          38},
    {699,       -41,        3},
    {157,       -682,       14},
    {796,       -68,        27},
    {728,       -461,       657},
    {0,         0,          800},
    {21,        -593,       537},
    {85,        -1196,      42},
    {54,        -1198,      36},
    {0,         0,          0}
};

#endif /* _CALIBRATE_POSITIONS_H_ */
