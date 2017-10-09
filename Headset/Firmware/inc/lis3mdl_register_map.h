/************************************************************************************

Filename    :   lis3mdl_register_map.h
Content     :   ST LIS3MDL Magnetometer register map
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _LIS3MDL_REGISTER_MAP_H_
#define _LIS3MDL_REGISTER_MAP_H_

#define WHO_AM_I    0x0F
#define CTRL_REG1   0x20
#define CTRL_REG2   0x21
#define CTRL_REG3   0x22
#define CTRL_REG4   0x23
#define CTRL_REG5   0x24
#define STATUS_REG  0x27
#define OUT_X_L     0x28
#define OUT_X_H     0x29
#define OUT_Y_L     0x2A
#define OUT_Y_H     0x2B
#define OUT_Z_L     0x2C
#define OUT_Z_H     0x2D
#define INT_SRC     0x31
#define INT_THS_L   0x32
#define INT_THS_H   0x33

// gain
#define GN_4    (0)
#define GN_8    (1 << 5)
#define GN_12   (2 << 5)
#define GN_16   (3 << 5)

#define GN_SCALE_4  (6842)
#define GN_SCALE_8  (3421)
#define GN_SCALE_12 (2281)
#define GN_SCALE_16 (1711)

#endif /* _LIS3MDL_REGISTER_MAP_H_ */
