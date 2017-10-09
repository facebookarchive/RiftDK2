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
#include "hw_config.h"
#include "sequence.h"
#include "timestamper.h"
#include "vsync.h"
#include "lis3mdl.h"
#include "mpu6500.h"
#include "hub.h"
#include "cpal_conf.h"
#include "systick.h"

extern bool v_lock_changed;

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

void EXTI9_5_IRQHandler(void)
{
    // Interrupt handler for sync input
    if (EXTI_GetITStatus(CAM_SYNC_IN_IRQ_LINE) != RESET) {
        EXTI_ClearITPendingBit(CAM_SYNC_IN_IRQ_LINE);
        sequence_trigger(1);
        return;
    }
    
    // Interrupt handler for wall wart line sync input
    if (EXTI_GetITStatus(WW_DETECT_IRQ_LINE) != RESET) {
        EXTI_ClearITPendingBit(WW_DETECT_IRQ_LINE);
        sequence_trigger(1);
        return;
    }
    
    // Interrupt for mag data ready
    if (EXTI_GetITStatus(MAG_DRDY_LINE) != RESET) {
        EXTI_ClearITPendingBit(MAG_DRDY_LINE);
        lis3mdl_ready();
        return;
    }
}

// Interrupt handler for HDMI bridge status changes
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(BRIDGE_IRQ_LINE) != RESET) {
        EXTI_ClearITPendingBit(BRIDGE_IRQ_LINE);
        // The only interrupt currently configured is hdmi rx locked or lost
        v_lock_changed = 1;
        return;
    }

    if (EXTI_GetITStatus(ACC_PWR_FAULT_IRQ_LINE) != RESET) {
        EXTI_ClearITPendingBit(ACC_PWR_FAULT_IRQ_LINE);
        hub_accessory_power_fault_irq();
        return;
    }
}

void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(SYNC_OUTPUT_TIM, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(SYNC_OUTPUT_TIM, TIM_IT_Update);
        sequence_trigger(0);
    }
}

void TIM2_IRQHandler(void)
{
    // TIM2 is a 16-bit microsecond timer, which we use for high precision
    // timestamping of the gyro and display.  We convert it to a 32-bit
    // timer by incrementing a counter on overflow.  There is a race between
    // the interrupt for the increment and the interrupts for the peripherals
    // that also use TIM2.
    //
    // There are three edge cases we need to handle (for both gyro and vsync):
    // 1. TIM2 rolls followed by gyro interrupt capture (around 0000).  In this
    //    case, TIM2_IRQHandler is called, timestamper_update happens,
    //    incrementing the overflow counter, and because a gyro interrupt
    //    is pending, the gyro_rolled flag is marked.  However, in
    //    mpu6500_update, since the capture value is below the current TIM2
    //    counter, the incremented overflow counter is correct.
    //
    // 2. Gyro interrupt capture (around FFFF) followed by TIM2 rolling.
    //    TIM2_IRQHandler is called, timestamper_update increments the overflow
    //    counter, and the gyro_rolled flag is marked.  In mpu6500_update,
    //    because the capture is higher than the current TIM2 counter and
    //    rolled is true, the overflow counter is decremented to be correct.
    //
    // 3. Gyro interrupt capture (around FFFF) followed by TIM2_IRQHandler.
    //    At some point before reading the overflow counter, an overflow occurs.
    //    Since we are already in TIM2_IRQHandler, the counter isn't incremented
    //    yet.  When we get into mpu6500_update, since rolled isn't set to true,
    //    we correctly use the old overflow counter.
    //
    // Bad things can potentially still happen here if we get hit with a
    // higher priority interrupt that takes longer than about 50 ms.
    bool rolled_over = 0;
    
    // Handle the 16-bit timer rolling over, to make a 32-bit timer
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        timestamper_update();
        // Mark that the overflow counter incremented for any other pending
        // flags
        rolled_over = 1;
    }
    
    // Capture panel vsyncs
    if (TIM_GetITStatus(PANEL_VSYNC_TIM, PANEL_VSYNC_FLAG) != RESET) {
        static bool vsync_rolled = 0;
        // We only want to take action once per TIM2_IRQHandler call.  If the
        // flag is set, it means we're going to get called again.
        if (!rolled_over) {
            TIM_ClearITPendingBit(PANEL_VSYNC_TIM, PANEL_VSYNC_FLAG);
            vsync_update(vsync_rolled);
            vsync_rolled = 0;
            return;
        } else {
            // Mark that the pending interrupt had an the overflow counter
            // incremented
            vsync_rolled = 1;
        }
    }
    
#ifdef GYRO_IRQ_TIM
    // Capture gyro interrupts
    if (TIM_GetITStatus(GYRO_IRQ_TIM, GYRO_IRQ_FLAG) != RESET) {
        static bool gyro_rolled = 0;
        if (!rolled_over) {
            TIM_ClearITPendingBit(GYRO_IRQ_TIM, GYRO_IRQ_FLAG);
            mpu6500_ready(gyro_rolled);
            gyro_rolled = 0;
            return;
        } else {
            gyro_rolled = 1;
        }
    }
#endif /* GYRO_IRQ_TIM */
}

void EXPOSURE_TIM_IRQHandler(void)
{
    // End the tracking led exposure and sync outputs
    if (TIM_GetITStatus(EXPOSURE_TIM, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(EXPOSURE_TIM, TIM_IT_Update);
        sequence_end_exposure();
    }
}

void USB_LP_IRQHandler(void)
{
    USB_Istr();
}


void USB_FS_WKUP_IRQHandler(void)
{
    EXTI_ClearITPendingBit(EXTI_Line18);
}
