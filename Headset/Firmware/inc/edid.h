/************************************************************************************

Filename    :   edid.h
Content     :   EDID generation
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _EDID_H_
#define _EDID_H_

#include <stdint.h>

void edid_generate(uint8_t *buf, uint16_t year, uint8_t week, const char *serial);

void edid_generate_cea(uint8_t *buf);

#endif /* _EDID_H_ */
