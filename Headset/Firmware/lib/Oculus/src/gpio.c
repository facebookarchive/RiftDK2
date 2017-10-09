/************************************************************************************

Filename    :   gpio.c
Content     :   GPIO configuration and utility functions.
Created     :
Authors     :   Lyle Bainbridge

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "gpio.h"

#define MODE_CONFIG_BITS_MASK       0x0003
#define OTYPE_CONFIG_BITS_MASK      0x000c
#define PUPD_CONFIG_BITS_MASK       0x0030
#define SPEED_CONFIG_BITS_MASK      0x0300
#define OTYPE_CONFIG_BITS_POS       2
#define PUPD_CONFIG_BITS_POS        4


void gpio_config(GPIO_TypeDef* port, uint32_t pin, uint16_t config)
{
    GPIO_InitTypeDef gpio_init;

    gpio_init.GPIO_Pin   = pin;

    gpio_init.GPIO_Mode  = (GPIOMode_TypeDef)  (config & MODE_CONFIG_BITS_MASK);

    gpio_init.GPIO_OType = (GPIOOType_TypeDef)((config & OTYPE_CONFIG_BITS_MASK)
                                                      >> OTYPE_CONFIG_BITS_POS);

    gpio_init.GPIO_PuPd  = (GPIOPuPd_TypeDef) ((config & PUPD_CONFIG_BITS_MASK)
                                                      >> PUPD_CONFIG_BITS_POS);

    switch (config & 0x0300) {
        case GPIO_400KHZ:
            gpio_init.GPIO_Speed = GPIO_Speed_400KHz;
            break;

        case GPIO_2MHZ:
            gpio_init.GPIO_Speed = GPIO_Speed_2MHz;
            break;

        case GPIO_10MHZ:
            gpio_init.GPIO_Speed = GPIO_Speed_10MHz;
            break;

        case GPIO_40MHZ:
        default:
            gpio_init.GPIO_Speed = GPIO_Speed_40MHz;
            break;
    }

    if (port == GPIOA) {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    }
    else if (port == GPIOB) {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    }
    else if (port == GPIOC) {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
    }
    else if (port == GPIOD) {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
    }
    else if (port == GPIOE) {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);
    }
    else if (port == GPIOF) {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
    }
    else if (port == GPIOG) {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOG, ENABLE);
    }
    else {
        assert_param();
    }

    GPIO_Init(port, &gpio_init);
}

uint8_t gpio_get_state(GPIO_TypeDef* p_port, uint16_t pin)
{
    return ((p_port->IDR & pin) ? 1 : 0);
}

void gpio_set_state(GPIO_TypeDef* p_port, uint16_t pin, uint8_t state)
{
    GPIO_WriteBit(p_port, pin, (BitAction)state);
}

void gpio_set(GPIO_TypeDef* p_port, uint16_t pin)
{
    GPIO_WriteBit(p_port, pin, Bit_SET);
}

void gpio_reset(GPIO_TypeDef* p_port, uint16_t pin)
{
    GPIO_WriteBit(p_port, pin, Bit_RESET);
}
