/************************************************************************************

Filename    :   usb.c
Content     :   STM32L USB interface related functions.
Created     :
Authors     :   Lyle Bainbridge

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "hw_config.h"
#include "gpio.h"
#include "usb_lib.h"
#include "usb_pwr.h"
#include "usb.h"

void usb_init(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // NOTE: Do not configure the USB GPIO alternate function, this does not
    // work.  Enabling the USB clock is all that is needed for USB.  Pin config
    // is done automatically.
    
    // Configure the EXTI line 18 connected internally to the USB IP
    EXTI_ClearITPendingBit(EXTI_Line18);
    EXTI_InitStructure.EXTI_Line = EXTI_Line18; // USB resume from suspend mode
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // Enable the USB interrupt
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Enable the USB Wake-up interrupt
    NVIC_InitStructure.NVIC_IRQChannel = USB_FS_WKUP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //  Enable the USB clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);

    USB_Init();
}

void usb_deinit(void)
{
    // Disconnect from USB
    PowerOff();
}

bool usb_ready(void)
{
    return bDeviceState == CONFIGURED;
}
