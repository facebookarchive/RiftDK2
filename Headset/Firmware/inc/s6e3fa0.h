/************************************************************************************

Filename    :   s6e3fa0.h
Content     :   Samsung S6E3FA0 display driver configuration
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _S6E3FA0_H_
#define _S6E3FA0_H_

#include <stdbool.h>
#include <stdint.h>
#include "panel.h"

bool s6e3fa0_set_state(bool state, const panel_timing_p timing);

#endif /* _S6E3FA0_H_ */
