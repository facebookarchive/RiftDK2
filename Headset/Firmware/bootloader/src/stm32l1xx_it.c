/************************************************************************************

Filename    :   stm32l1xx_it.c
Content     :   Interrupt handling
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include <stdint.h>
#include <stm32l1xx.h>
#include "stm32l1xx_it.h"
#include "usb_istr.h"
#include "usb_lib.h"
#include "usb_pwr.h"
#include "systick.h"
#include "bootloader_state.h"
#include "cpal.h"

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    while (1) {
    }
}

void MemManage_Handler(void)
{
    while (1) {
    }
}

void BusFault_Handler(void)
{
    while (1) {
    }
}

void UsageFault_Handler(void)
{
    while (1) {
    }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
    systick_update();
    CPAL_I2C_TIMEOUT_Manager();
}

void USB_LP_IRQHandler(void)
{
    USB_Istr();
}


void USB_FS_WKUP_IRQHandler(void)
{
    EXTI_ClearITPendingBit(EXTI_Line18);
}
