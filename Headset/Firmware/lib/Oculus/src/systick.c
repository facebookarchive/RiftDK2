/************************************************************************************

Filename    :   systick.c
Content     :   Driver for the Cortex systick timer.
Created     :
Authors     :   Lyle Bainbridge

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include <stm32l1xx.h>
#include "delay.h"
#include "systick.h"

#define TICKS_PER_SECOND    1000L

static volatile uint32_t g_system_tick_count = 0;


void systick_init(void)
{
    RCC_ClocksTypeDef rcc_clocks;

    RCC_GetClocksFreq(&rcc_clocks);
    SysTick_Config(rcc_clocks.HCLK_Frequency / TICKS_PER_SECOND);
    // SysTick needs to be a higher priority than anything using delay
    // or CPAL
    NVIC_SetPriority(SysTick_IRQn, 1);
}

void systick_disable(void)
{
    SysTick_Config(0);
}

void systick_reset_tick_count(void)
{
    g_system_tick_count = 0;
}

uint32_t systick_get_tick_count(void)
{
    return g_system_tick_count;
}

void systick_update(void)
{
    g_system_tick_count++;

    delay_update();
}
