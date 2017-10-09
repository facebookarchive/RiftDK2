/************************************************************************************

Filename    :   timestamper.h
Content     :   Microsecond timer for timestamping and triggering
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _TIMESTAMPER_H_
#define _TIMESTAMPER_H_

#include <stdint.h>
#include <stdbool.h>

void timestamper_init(void);

uint32_t timestamper_high_bits(void);

void timestamper_update(void);

uint32_t timestamper_get_time(void);

bool timestamper_is_after(uint32_t time);

void timestamper_delay_until(uint32_t time);

void timestamper_delay_us(uint32_t delay);

uint32_t timestamper_diff(void);

#endif /* _TIMESTAMPER_H_ */
