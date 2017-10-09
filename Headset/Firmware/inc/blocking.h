/************************************************************************************

Filename    :   blocking.h
Content     :   A workaround to send samples during blocking waits
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2014-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _BLOCKING_H_
#define _BLOCKING_H_

#include <stdint.h>

void blocking_update(uint32_t wait_us);

#endif /* _BLOCKING_H_ */
