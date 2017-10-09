/************************************************************************************

Filename    :   debug.c
Content     :   USART printf code
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "debug.h"
#include <stdio.h>
#include "gpio.h"
#include "hw_config.h"


void debug_init(void)
{
    USART_InitTypeDef USART_InitStructure;

    // The GPIO RCC needs to be enabled before the AF config
    RCC_AHBPeriphClockCmd(USART_GPIO_RCC, ENABLE);

    RCC_APB2PeriphClockCmd(USART_RCC, ENABLE);

    GPIO_PinAFConfig(USART_PORT, USART_TX_SOURCE, USART_AF);
    GPIO_PinAFConfig(USART_PORT, USART_RX_SOURCE, USART_AF);

    gpio_config(USART_PORT, USART_TX_PIN, GPIO_AF_PP);
    gpio_config(USART_PORT, USART_RX_PIN, GPIO_AF_PP);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART_NUM, &USART_InitStructure);
    USART_Cmd(USART_NUM, ENABLE);
}

int getchar(void)
{
    if (USART_GetFlagStatus(USART_NUM, USART_FLAG_RXNE) == SET)
        return USART_ReceiveData(USART_NUM);

    return EOF;
}

int putchar(int ch)
{
    while (USART_GetFlagStatus(USART_NUM, USART_FLAG_TXE) == RESET);
    USART_SendData(USART_NUM, (uint8_t)ch);

    return ch;
}
