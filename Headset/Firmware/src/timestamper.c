/************************************************************************************

Filename    :   timestamper.c
Content     :   Microsecond timer for timestamping and triggering
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "timestamper.h"
#include "stm32l1xx.h"

#define TIMESTAMPER_PRESCALER ((SystemCoreClock / 1000000) - 1)

static volatile uint32_t updates = 0;
static uint32_t start_of_diff = 0;

void timestamper_init(void)
{
    // Set up TIM2 as a microsecond timer
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = TIMESTAMPER_PRESCALER;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    // Set up the interrupt on reload so we can increment and turn a 16-bit timer
    // into a 32-bit one
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    // The timestamper priority needs to be as high or higher than anything
    // using the timestamp to get a correct 32 bit timestamp
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    
    TIM_Cmd(TIM2, ENABLE);
}

uint32_t timestamper_high_bits(void)
{
    return updates;
}

void timestamper_update(void)
{
    updates += 0x10000;
}

uint32_t timestamper_get_time(void)
{
    // Read the high 16 bits both before and after getting the low 16-bits.
    // We need data memory barrier instructions, as SRAM and the timer
    // peripheral can in theory have their accesses re-ordered for
    // efficiency
    uint32_t first_update = updates;
    __DMB();
    uint32_t counter = TIM_GetCounter(TIM2);
    __DMB();
    uint32_t second_update = updates;
    
    // If the first and second reads of the high bits don't match,
    // it means we wrapped.  Use the first or second read depending on if
    // the low 16 bits were about to wrap or have already wrapped
    if ((first_update != second_update) && (counter >> 15))
        return first_update | counter;
    
    return second_update | counter;
}

bool timestamper_is_after(uint32_t time)
{
    uint32_t now = timestamper_get_time();
    return (now > time) && ((now - time) < 0x7FFFFFF);
}

// Only designed to delay up to a half hour
void timestamper_delay_until(uint32_t time)
{
    while (!timestamper_is_after(time));
}

void timestamper_delay_us(uint32_t delay)
{
    uint32_t now = timestamper_get_time();
    timestamper_delay_until(now + delay);
}

// A convenience function for benchmarking
uint32_t timestamper_diff(void)
{
    uint32_t now = timestamper_get_time();
    uint32_t diff = now - start_of_diff;
    start_of_diff = now;
    
    return diff;
}
