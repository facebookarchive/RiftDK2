/************************************************************************************

Filename    :   stm32l.h
Content     :   Initialize the STM32L clocks and other related functionality.
Created     :
Authors     :   Lyle Bainbridge

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _STM32L_H_
#define _STM32L_H_

void stm32l_init(void);

void stm32l_start_hse(void);

#endif /* _STM32L_H_ */
