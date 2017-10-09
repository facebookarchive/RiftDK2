/************************************************************************************

Filename    :   crc.h
Content     :   Cyclic redundancy check functions
Created     :
Authors     :   Lyle Bainbridge

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _CRC_H_
#define _CRC_H_

#include <stdint.h>


uint16_t crc16(const void* p_buffer, uint32_t length);


#endif /* _CRC_H_ */
