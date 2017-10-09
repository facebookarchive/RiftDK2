/************************************************************************************

Filename    :   stm32l.c
Content     :   Initialize the STM32L clocks and other related functionality.
Created     :
Authors     :   Lyle Bainbridge

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "gpio.h"
#include "stm32l.h"
#include "hw_config.h"

uint32_t SystemCoreClock = 32000000;

void stm32l_init(void)
{
    // Use 2 bits of priority and 2 bits of subpriority
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // Enable the SYSCFG clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    // Enable the PWR management clocks
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

    // Set Vcore to 1.8V to get to 16-32 MHz
    PWR_VoltageScalingConfig(PWR_VoltageScaling_Range1);

    // Wait for the voltage regulator to be ready
    while(PWR_GetFlagStatus(PWR_FLAG_VOS) != RESET);

    // Start the HSE
    stm32l_start_hse();

    // Disable the MSI to save power
    RCC_MSICmd(DISABLE);
    
    // Set the Vector Table base location to make room for the bootloader
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, APPLICATION_OFFSET);
}

void stm32l_start_hse(void)
{
    ErrorStatus errorStatus;

    RCC_DeInit();

    // Enable HSE
    RCC_HSEConfig(RCC_HSE_ON);

    // Wait till HSE is ready */
    errorStatus = RCC_WaitForHSEStartUp();

    if (errorStatus == SUCCESS) {
        // Enable flash 64-bit access
        FLASH_ReadAccess64Cmd(ENABLE);

        // Enable flash prefetch buffer
        FLASH_PrefetchBufferCmd(ENABLE);

        // Set flash latency 1 wait state
        FLASH_SetLatency(FLASH_Latency_1);

        // HCLK = SYSCLK
        RCC_HCLKConfig(RCC_SYSCLK_Div1);

        // PCLK2 = HCLK
        RCC_PCLK2Config(RCC_HCLK_Div1);

        // PCLK1 = HCLK
        RCC_PCLK1Config(RCC_HCLK_Div1);

        // USB needs 24 MHz or 32 MHz, so just go to 32 MHz for now
        // PLLCLK = 12MHz * 8 / 3 = 32MHz
        RCC_PLLConfig(RCC_PLLSource_HSE, RCC_PLLDiv_3, RCC_PLLMul_8);

        // Enable PLL
        RCC_PLLCmd(ENABLE);

        // Wait until PLL is ready
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {
        }

        // Select PLL as system clock source
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

        // Wait until PLL is used as system clock source
        while(RCC_GetSYSCLKSource() != 0x0C) {
        }
    }
}


#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  while (1)
  {
  }
}
#endif
