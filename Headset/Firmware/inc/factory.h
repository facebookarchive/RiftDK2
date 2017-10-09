/************************************************************************************

Filename    :   factory.h
Content     :   Tracker factory calibration interface
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _FACTORY_H_
#define _FACTORY_H_

#include <stdint.h>
#include <stdbool.h>
#include "mpu6500.h"

bool factory_check_enable(void);

void factory_set_command(uint8_t new_command);

void factory_update(mpu6500_data_p data);

#endif /* _FACTORY_H_ */
