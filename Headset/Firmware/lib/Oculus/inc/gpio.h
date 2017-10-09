/************************************************************************************

Filename    :   gpio.h
Content     :   GPIO configuration and utility functions.
Created     :
Authors     :   Lyle Bainbridge

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _GPIO_H_
#define _GPIO_H_

#include <stdint.h>
#include <stm32l1xx.h>

#define GPIO_IN         (GPIO_Mode_IN)
#define GPIO_PP         (GPIO_Mode_OUT | (GPIO_OType_PP  << 2))
#define GPIO_OD         (GPIO_Mode_OUT | (GPIO_OType_OD  << 2))
#define GPIO_AF         (GPIO_Mode_AF)
#define GPIO_AN         (GPIO_Mode_AN)

#define GPIO_IN_PU      (GPIO_IN | (GPIO_PuPd_UP   << 4))
#define GPIO_IN_PD      (GPIO_IN | (GPIO_PuPd_DOWN << 4))
#define GPIO_OD_PU      (GPIO_OD | (GPIO_PuPd_UP   << 4))

// TODO: Are these even required?  Does AF always default to correct config?
#define GPIO_AF_PU      (GPIO_AF | (GPIO_PuPd_UP   << 4))
#define GPIO_AF_PD      (GPIO_AF | (GPIO_PuPd_DOWN << 4))
#define GPIO_AF_PP      (GPIO_AF | (GPIO_OType_PP  << 2))
#define GPIO_AF_OD      (GPIO_AF | (GPIO_OType_OD  << 2))
#define GPIO_AF_OD_PU   (GPIO_AF | (GPIO_OType_OD  << 2) | (GPIO_PuPd_UP << 4))

#define GPIO_400KHZ     0x0100
#define GPIO_2MHZ       0x0200
#define GPIO_10MHZ      0x0300
#define GPIO_40MHZ      0x0000


void gpio_config(GPIO_TypeDef* port, uint32_t pin, uint16_t config);
uint8_t gpio_get_state(GPIO_TypeDef* p_port, uint16_t pin);
void gpio_set_state(GPIO_TypeDef* p_port, uint16_t pin, uint8_t state);
inline void gpio_set(GPIO_TypeDef* p_port, uint16_t pin);
inline void gpio_reset(GPIO_TypeDef* p_port, uint16_t pin);

#endif /* _GPIO_H_ */
