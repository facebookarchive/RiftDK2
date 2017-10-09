/************************************************************************************

Filename    :   hub.c
Content     :   
Created     :
Authors     :   Lyle Bainbridge

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include <string.h>
#include "gpio.h"
#include "hw_config.h"
#include "delay.h"
#include "cpal_i2c.h"
#include "m24.h"
#include "uuid.h"
#include "hub.h"
#include "systick.h"

static void hub_handle_ww(void);
static void hub_set_accessory_pwr_fault_irq(FunctionalState state);

static bool g_ww_detect_state = 0;
static bool g_override_power = 0;
static bool g_acc_pwr_fault = 0;
static m24_t g_hub_m24 = {{0}};

#define OCULUS_VID_LSB      0x33
#define OCULUS_VID_MSB      0x28
#define OCULUS_PID_LSB      0x21
#define OCULUS_PID_MSB      0x20
#define HUB_EEPROM_SIZE     128
#define HUB_SERIAL_SIZE     15

uint8_t g_hub_cfg[HUB_EEPROM_SIZE] =
{
    OCULUS_VID_LSB, // 0x00 VID_LSB
    OCULUS_VID_MSB, // 0x01 VID_MSB
    OCULUS_PID_LSB, // 0x02 PID_LSB
    OCULUS_PID_MSB, // 0x03 PID_MSB
    0x00,           // 0x04 ChkSum
    0xfe,           // 0x05 Reserved (0xfe)
    0x02,           // 0x06 Removable ports, default to port 1 fixed
    0x01,           // 0x07 Number of ports, default to 1
    0xfa,           // 0x08 Maximum power, default to 500mA (2mA x 0xfa)
    // Reserved, 0x09 to 0x0f (note that 0x0b must be 0xfe)
    0xff, 0xff, 0xfe, 0xff, 0xff, 0xff, 0xff,
    // Vendor string, 0x10 to 0x3f
    0x0f,           // 0x10 Vendor string length
    'O','c','u','l','u','s',' ','V','R', ',', ' ', 'I', 'n', 'c', '.',
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    // Product string, 0x40 to 0x6f
    0x08,           // 0x40 Product string length
    'R', 'i', 'f', 't', ' ', 'D', 'K', '2',
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    // Serial number string, 0x70 to 0x7f
    0x00,           // 0x70 Serial number string length
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

void hub_init(bool override_power)
{
    g_override_power = override_power;
    
    // Configure hub reset output
    gpio_config(HUB_RESET_PORT, HUB_RESET_PIN, GPIO_PP);
    // Hold the hub in reset until initialization is complete
    GPIO_WriteBit(HUB_RESET_PORT, HUB_RESET_PIN, Bit_RESET);

    // Configure hub suspend input
    gpio_config(USB_SUSPEND_PORT, USB_SUSPEND_PIN, GPIO_IN);

    // Start with accessory power off until we know the wall wart is present
    gpio_config(ACC_PWR_EN_PORT, ACC_PWR_EN_PIN, GPIO_OD);
    GPIO_WriteBit(ACC_PWR_EN_PORT, ACC_PWR_EN_PIN, Bit_SET);

    // Start as a bus powered device until we know the wall wart is present
    gpio_config(HUB_SELFPWR_PORT, HUB_SELFPWR_PIN, GPIO_PP);
    GPIO_WriteBit(HUB_SELFPWR_PORT, HUB_SELFPWR_PIN, Bit_RESET);

    // Start WW detect as an input, it is later reconfigured in the LED code
    gpio_config(WW_DETECT_PORT, WW_DETECT_PIN, GPIO_IN);
    // FIXME: #142 the ID pin appears to stay high for some amount of time after
    // power on.  Wait a little while before reading the detect pin
    delay_ms(50);
    g_ww_detect_state = GPIO_ReadInputDataBit(WW_DETECT_PORT, WW_DETECT_PIN);

    g_hub_m24.address = HUB_I2C_SLAVE_ADDR;
    g_hub_m24.cpal = HUB_I2C_CPAL;
    g_hub_m24.device = HUB_I2C_DEVICE;
    g_hub_m24.options = 0;
    
    // Bring high the pull-up for SCL for the EEPROM interface
    gpio_config(I2C_SCL_PU_EN_PORT, I2C_SCL_PU_EN_PIN, GPIO_PP);
    GPIO_WriteBit(I2C_SCL_PU_EN_PORT, I2C_SCL_PU_EN_PIN, Bit_SET);
    // Initialize the M24 I2C bus for the hub boot EEPROM
    m24_init(&g_hub_m24);

    // Configure the hub boot EEPROM I2C buffer enable 
    gpio_config(HUB_I2C_EN_PORT, HUB_I2C_EN_PIN, GPIO_PP);

    // Disable the hub boot EEPROM I2C buffer
    GPIO_WriteBit(HUB_I2C_EN_PORT, HUB_I2C_EN_PIN, Bit_RESET);

    // Accessory LED is off by default
    gpio_config(ACC_LED_EN_PORT, ACC_LED_EN_PIN, GPIO_OD);
    GPIO_WriteBit(ACC_LED_EN_PORT, ACC_LED_EN_PIN, Bit_SET);
    
    // Update hub configuration if wall wart is present
    if (g_ww_detect_state) {
        // Set up the number of ports to two because the accesory port will be enabled
        // Also allow the override to enable the extra internal port
        g_hub_cfg[7] = g_override_power ? 3 : 2;

        // When the wall wart is present the hub becomes self powered
        GPIO_WriteBit(HUB_SELFPWR_PORT, HUB_SELFPWR_PIN, Bit_SET);

        // Set maximum power to 2mA (2mA x 1) when self powered
        g_hub_cfg[8] = 1;
    } else if (g_override_power) {
        // Force the accessory and internal ports enabled
        g_hub_cfg[7] = 3;
        
        // Fake presense of the wall wart
        GPIO_WriteBit(HUB_SELFPWR_PORT, HUB_SELFPWR_PIN, Bit_SET);
        
        // Claim the one unit load allowed for self-powered devices
        g_hub_cfg[8] = 100 / 2;
    } else {
        // Use 400 mA for the hub, leaving 100 for the Rift
        g_hub_cfg[8] = 400 / 2;
    }

    // Get the UUID to use as the hub serial number
    char uuid[UUID_LEN];
    uuid_get(uuid, 20, 0);

    // Set the serial number as the last 15 characters of the UUID
    g_hub_cfg[0x70] = HUB_SERIAL_SIZE;
    memcpy(&g_hub_cfg[0x71], &uuid[UUID_LEN - HUB_SERIAL_SIZE], HUB_SERIAL_SIZE);

    // Set the hub config checksum
    g_hub_cfg[4] = g_hub_cfg[0] + g_hub_cfg[1] + g_hub_cfg[2] + g_hub_cfg[3] + 1;

    // Read the current hub configuration
    uint8_t current_hub_cfg[HUB_EEPROM_SIZE];
    m24_read(&g_hub_m24, 0, current_hub_cfg, HUB_EEPROM_SIZE);

    // If the hub EEPROM has has changed, write the bytes that changed
    for (uint8_t i = 0; i < HUB_EEPROM_SIZE; i++) {
        if (g_hub_cfg[i] != current_hub_cfg[i]) {
            m24_write(&g_hub_m24, i, &g_hub_cfg[i], 1);
        }
    }

    // Deinitialize the M24 I2C bus to avoid contention with the hub which
    // does non-standard I2C with a push-pull clock
    m24_deinit(&g_hub_m24);
    gpio_config(GPIOB, GPIO_Pin_6 | GPIO_Pin_7, GPIO_IN);
    gpio_config(I2C_SCL_PU_EN_PORT, I2C_SCL_PU_EN_PIN, GPIO_AN);

    // Enable the hub boot EEPROM I2C buffer so that the hub can access it
    GPIO_WriteBit(HUB_I2C_EN_PORT, HUB_I2C_EN_PIN, Bit_SET);

    // Hub clock is driven by MCO pin at 12MHz from HSE
    gpio_config(MCO_PORT, MCO_PIN, GPIO_AF);
    RCC_MCOConfig(RCC_MCOSource_HSE, RCC_MCODiv_1);

    // Bring the hub out of reset
    GPIO_WriteBit(HUB_RESET_PORT, HUB_RESET_PIN, Bit_SET);
}

void hub_post_init(void)
{
    // Disable the hub boot EEPROM I2C buffer
    GPIO_WriteBit(HUB_I2C_EN_PORT, HUB_I2C_EN_PIN, Bit_RESET);

    // Configure the accessory port power and LED depending on ww detect
    hub_handle_ww();

    // Configure the interrupt for accessory power fault
    hub_set_accessory_pwr_fault_irq(ENABLE);
}

bool hub_get_self_powered(void)
{
    return g_ww_detect_state;
}

void hub_check_ww(void)
{
    // As a temporary workaround for the ID pin not being detectable until
    // USB is plugged in, check for wall wart again after USB is ready
    if (GPIO_ReadInputDataBit(WW_DETECT_PORT, WW_DETECT_PIN) && !g_ww_detect_state) {
        // Just reset so we reconfigure completely
        NVIC_SystemReset();
    }
}

static bool hub_allow_accessory(void)
{
    return ((g_ww_detect_state || g_override_power) && !g_acc_pwr_fault);
}

void hub_handle_ww(void)
{
    // Check if there is a power fault on the accessory port switch
    g_acc_pwr_fault = 
            !GPIO_ReadInputDataBit(ACC_PWR_FAULT_PORT, ACC_PWR_FAULT_PIN);
    
    // If there is no fault and we either have a wall wart or are forcing
    // enable on the accessory port, enable the accessory port
    if (hub_allow_accessory()) {
        GPIO_WriteBit(ACC_PWR_EN_PORT, ACC_PWR_EN_PIN, Bit_RESET);
    } else {
        GPIO_WriteBit(ACC_PWR_EN_PORT, ACC_PWR_EN_PIN, Bit_SET);
    }
}

void hub_accessory_power_fault_irq(void)
{
    // Hub accessory power handling is done in ww detect handling
    hub_handle_ww();
}

void hub_set_accessory_pwr_fault_irq(FunctionalState state)
{
    EXTI_InitTypeDef  EXTI_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    SYSCFG_EXTILineConfig(ACC_PWR_FAULT_IRQ_SOURCE_PORT, ACC_PWR_FAULT_IRQ_SOURCE);

    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Line = ACC_PWR_FAULT_IRQ_LINE;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // LB TODO: Set up NVIC in a central location where we can establish the
    // priorities for all interrupts.  Some interrupts like this are shared.
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void hub_accessory_led_update(bool suspended)
{
    // The LED comes out of the device power budget, so turn it off if
    // we suspend the device, even if the accessory port is still enabled
    if (!suspended && hub_allow_accessory()) {
        GPIO_WriteBit(ACC_LED_EN_PORT, ACC_LED_EN_PIN, Bit_RESET);
    } else {
        GPIO_WriteBit(ACC_LED_EN_PORT, ACC_LED_EN_PIN, Bit_SET);
    }
}
