/************************************************************************************

Filename    :   main.c
Content     :   Main loop and supporting functions for the main board.
Created     :
Authors     :   Nirav Patel, Lyle Bainbridge

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include <stm32l1xx.h>
#include "hw_config.h"
#include "usb.h"
#include "hub.h"
#include "leds.h"
#include "button.h"
#include "uuid.h"
#include "stm32l.h"
#include "systick.h"
#include "delay.h"
#include "bootloader.h"
#include "config.h"

typedef void (*pFunction)(void);

__IO uint8_t exit_stop_mode = 1;

int main(void)
{
    // stm32l_init happens from the reset handler
    systick_init();
    leds_init();
    button_init();
    config_init();
    
    // Check the EEPROM for if the bootloader should be used
    bool bootload = bootloader_should_use();
    
    // Also allow firmware updating to be forced by holding the power button
    if (!bootload) {
        button_update();
        while (button_raw()) {
            // Indicate that the button is working by turning on the amber LED
            led_amber_on();
            button_update();
            // If the button is held for 4 seconds on power on, go into bootloader
            // mode
            if (systick_get_tick_count() - button_last_change() > 4000) {
                bootload = 1;
                break;
            }
        }
    }
    
    led_amber_set(bootload);
    
    if (!bootload) {
        // Check if there is an application burned in
        if (((*(__IO uint32_t *)APPLICATION_ADDRESS) & 0x2FFE0000) == 0x20000000) {
            pFunction jump_to_application;
            uint32_t jump_address;
            
            // If there is, jump to it
            jump_address = *(__IO uint32_t *)(APPLICATION_ADDRESS + 4);
            jump_to_application = (pFunction)jump_address;
            // Set up the stack pointer
            __set_MSP(*(__IO uint32_t *)APPLICATION_ADDRESS);
            jump_to_application();
        } else {
            // If no application exists, stay in the bootloader, so turn on the LED
            led_amber_set(1);
        }
    }

    // To keep USB happy, we need some time between disconnecting the application
    // USB device and connecting the bootloader device.
    while (systick_get_tick_count() < 200);
    
    // Update firmware mode, initialize what we need for USB to work
    bootloader_init();
    uuid_init();
    hub_init(0);
    usb_init();
    while (!usb_ready());
    hub_post_init();
    
    // DK2 shares a bus for the hub and config i2c, so reinit config
    config_init();
    
    // TODO: Don't allow feature reports until after config_init
    
    bool led_state = 0;
    uint32_t led_toggle_time = systick_get_tick_count();
    
    while(1) {
        __WFI();
        // Get the current time for housekeeping use
        uint32_t now = systick_get_tick_count();
        
        // Toggle the Amber LED with a half second period
        if (now - led_toggle_time >= 250) {
            led_toggle_time = now;
            led_state = !led_state;
            led_amber_set(led_state);
        }
        
        if (bootloader_reboot()) {
            // Disconnect USB and reboot
            usb_deinit();
        
            // Wait before switching to application firmware since it is a
            // different device appearing on the same USB port
            delay_ms(200);
            
            NVIC_SystemReset();
        }
    }
}
