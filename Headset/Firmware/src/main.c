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

#include "hw_config.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stm32l1xx.h>
#include "stm32l.h"
#include "delay.h"
#include "gpio.h"
#include "leds.h"
#include "hub.h"
#include "h2c.h"
#include "panel.h"
#include "debug.h"
#include "systick.h"
#include "button.h"
#include "uuid.h"
#include "sequence.h"
#include "timestamper.h"
#include "in_report.h"
#include "feature_report.h"
#include "usb.h"
#include "lis3mdl.h"
#include "mpu6500.h"
#include "vsync.h"
#include "autocalibrate.h"
#include "calibrate.h"
#include "factory.h"
#include "bootloader_state.h"
#include "config.h"
#include "blocking.h"

volatile bool v_lock_changed = 0;
static feature_config_t g_config = {0};
static feature_keep_alive_t g_keep_alive = {0};

static void brownout_setup(void)
{
    uint8_t bor = FLASH_OB_GetBOR();
    // Check if we have brownout reset at 1.94V, set it otherwise
    if (bor != OB_BOR_LEVEL2) {
        FLASH_OB_Unlock();
        FLASH_OB_BORConfig(OB_BOR_LEVEL2);
        FLASH_OB_Launch();
        FLASH_OB_Lock();
    }
}

static void power_on(void)
{
    // Start with the Amber LED on
    led_amber_on();
    hub_accessory_led_update(0);
    
    // Unsleep the sensors
    lis3mdl_sleep(0);
    mpu6500_sleep(0);

    sequence_init();
    
    panel_reset_state();
    h2c_power_on();
    h2c_config();

    // Initialize the MIPI Master
    panel_timing_t initial_timing;
    panel_current_timing(&initial_timing);
    h2c_mipi_init(&initial_timing);

    // Go into video mode, though the HDMI PHY will remain inactive until HPD
    h2c_video_mode();
}

static void power_off(void)
{
    led_amber_off();
    led_blue_off();
    hub_accessory_led_update(1);

    // Power off the various peripherals before powering off
    panel_enable(0);
    panel_power_off();
    h2c_power_off();
    mpu6500_sleep(1);
    lis3mdl_sleep(1);
    sequence_deinit();
    in_report_reset();
}

static void perform_reset(void)
{
    power_off();
    
    // Reset
    NVIC_SystemReset();
}

static void use_pll(bool use_pll)
{
    if (use_pll) {
        // Switch back to using the PLL
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        while (RCC_GetSYSCLKSource() != 0x0C);
        
        // Reset the systick and button again
        systick_init();
        button_init();
    } else {
        // Switch to directly running on HSE to drop current consumption
        // by a few milliamps
        RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE);
        while (RCC_GetSYSCLKSource() != 0x08);
        
        // Reset the systick to get the right millisecond tick
        systick_init();
        // Reset the button, since it depends on systick count
        button_init();
    }
}

static void off_loop(bool off_reason_usb)
{
    // Power everything off
    power_off();
    
    bool using_pll = 1;
    
    while (1) {
        // If USB is suspended, we need to get under 12.5 mA, which requires
        // shutting off more 
        if (usb_suspended() && using_pll) {
            use_pll(0);
            using_pll = 0;
        } else if (!usb_suspended() && !using_pll) {
            use_pll(1);
            using_pll = 1;
        }
        
        // Keep the CPU asleep while waiting for something to happen
        __WFI();
        
        // Check if the button has been pressed or we're getting woken from USB
        // and the reason we went to sleep was USB suspend
        if ((button_update() && button_state()) || (off_reason_usb && usb_ready())) {
            // Go back to full clock speed to use USB
            if (!using_pll) {
                use_pll(1);
                using_pll = 1;
            }
            
            // Bring USB back and attempt to wake up the PC
            if (!usb_ready()) {
                // If resume isn't allowed, remain in Suspend
                if (!usb_resume()) {
                    continue;
                }
            }
            
            // Only wake up everything if USB successfully came back on
            if (usb_ready()) {
                power_on();
                break;
            }
        }
    }
}

static bool handle_button(void)
{
    if (button_update() && button_state()) {
        // Wait for the button to be let go
        while (button_state()) {
            button_update();
        }
        
        return 1;
    }
    
    return 0;
}

static void handle_feature_report(void)
{
    feature_config_t config;
    feature_range_t range;
    feature_register_t reg;
    feature_dfu_t dfu;
    sequence_t sequence;
    panel_t panel;
    sequence_pattern_t pattern;
    
    if (feature_report_get_config(&config)) {
        if (config.override_power != g_config.override_power) {
            // Only set the config if it changed
            uint8_t overrides = config.override_power;
            config_write(&overrides, CFG_SIZE_OVERRIDES, CFG_ADDR_OVERRIDES);
        }
        
        // Use the new configuration
        memcpy(&g_config, &config, sizeof(feature_config_t));
        
        // Reset the IN Report, since the coordinate system may have changed
        // TODO: should this also reset the magnetometer fields?
        in_report_reset();        
    }
    
    if (feature_report_get_range(&range)) {
        mpu6500_set_ranges(range.accel_range, range.gyro_range);
        lis3mdl_set_range(range.mag_range);
    }
    
    if (feature_report_get_register(&reg)) {
        switch (reg.device) {
            case FEATURE_REGISTER_MPU6500:
                // dump partially stored packets only when gyro registers change
                in_report_reset();
                mpu6500_set_register(reg.reg, reg.payload);
                break;

            case FEATURE_REGISTER_LIS3MDL:
                lis3mdl_set_register(reg.reg, reg.payload);
                break;

            default:
                break;
        }
    }
    
    if (feature_report_get_dfu(&dfu)) {
        if (dfu.use_dfu) {
            bootloader_state_set(BOOTLOAD_MANUAL);
            perform_reset();
        }
    }
    
    if (feature_report_get_keep_alive(&g_keep_alive)) {
        // Update the in report configuration to switch between DK1 and DK2
        in_report_update_id(g_keep_alive.in_report);
    }
    
    if (feature_report_get_tracking(&sequence)) {
        sequence_set_state(&sequence);
    }
    
    if (feature_report_get_display(&panel)) {
        panel_set_state(&panel);
    }
    
    if (feature_report_get_pattern(&pattern)) {
        sequence_set_custom_pattern(&pattern);
    }
    
    in_report_update_command_id(feature_report_latest_command_id());
}

void blocking_update(uint32_t wait_us)
{
    uint32_t start = timestamper_get_time();
    
    mpu6500_data_t mpu = {0};
    exposure_t exposure = {0};
    
    // This is ugly, but we have a bunch of blocking waits that shouldn't
    // prevent inertial samples from being transmitted.  Note that to reduce
    // the scope of conflicts, only gyro/accel and LED sequence information
    // is updated
    while (!timestamper_is_after(start + wait_us)) {
        // Update the gyro/accelerometer part of the in report if there is a new sample
        if (mpu6500_read(&mpu, g_config.use_raw)) {
            // Don't store autocalibration if the samples are raw, to avoid bad table items
            autocalibrate_update(&mpu, g_config.autocalibration, !g_config.use_raw);
            
            if (g_config.use_calibration)
                calibrate_apply(&mpu);
            
            in_report_update_mpu(&mpu);
        }
        
        // Update the tracking part of the in report if there was a new exposure
        if (sequence_get_exposure(&exposure)) {
            in_report_update_tracking(&exposure);
        }
        
        if (!g_config.command_keep_alive || ((systick_get_tick_count() - feature_report_latest_command_time()) < g_keep_alive.keep_alive)) {
            if (in_report_ready(g_config.interval)) {
                in_report_send();
            }
        }
    }
}

int main(void)
{
    // stm32l_init happens from the reset handler
    brownout_setup();
    systick_init();
    timestamper_init();
    config_init();
    uuid_init();
    leds_init();
    
    // Initialize the HID Reports
    feature_report_init_config(&g_config);
    feature_report_init_keep_alive(&g_keep_alive);
    in_report_update_id(g_keep_alive.in_report);
    
    sequence_init();
    debug_init();
    button_init();
    lis3mdl_init();
    mpu6500_init();
    calibrate_init();
    h2c_init();
    panel_init();

    // Wait until the modules are initialized before bringing up USB    
    hub_init(g_config.override_power);
    usb_init();
    bool factory_calibrating = factory_check_enable();
    
    // Spit out the build date/time/compiler version for board debugging
    // over the serial interface if we aren't in the calibration cube
    if (!factory_calibrating) {
        printf("%s %s %u\r\n", __DATE__, __TIME__, __VER__);
    }
    
    // Wait for USB to be present before doing anything unless we're in factory mode
    while (!factory_calibrating && !usb_ready());
    // As a workaround for #145, check wall wart presence again after USB
    // is ready
    hub_check_ww();
    // Take the i2c bus back from the hub after USB is ready
    hub_post_init();
    
    // DK2 shares a bus for the hub and config i2c, so reinit config
    config_init();
    
    // TODO: Don't allow feature report reads until after config is reinited
    
    // Turn on the bridge, panel, and sensors
    power_on();
    
    lis3mdl_data_t mag = {0};
    mpu6500_data_t mpu = {0};
    exposure_t exposure = {0};
    vsync_t vsync = {0};
    uint32_t last_feature_report = 0;
    bool reset_sync = 0;
    bool keep_alive_active = 0;
    
    while (1) {
        // Keep the CPU asleep while waiting for something to happen
//        __WFI();
        
        // Get the current time for housekeeping use
        uint32_t now = systick_get_tick_count();

        // Handle any incoming feature reports
        uint32_t latest_report = feature_report_count();
        if (last_feature_report != latest_report) {
            handle_feature_report();
            last_feature_report = latest_report;
        }
        
        // Use the button for power on/off
        if (handle_button()) {
            off_loop(0);
        }
                
        // Handle powering down due to USB suspend, only if we aren't in
        // a test fixture where USB should not be present
        if (!factory_calibrating && usb_suspended()) {
            // We get stuck in a suspend loop on boards before Rev3.3
#ifdef DK2_REV3_3
            off_loop(1);
#endif /* DK2_REV3_3 */
        }
        
        // Handle getting or losing HDMI Rx
        if (v_lock_changed) {
            v_lock_changed = 0;
            h2c_hdmi_v_clear();
            // Reset the calculated refresh rate if we get or lose sync
            vsync_reset_average();
            reset_sync = 1;
            if (h2c_hdmi_v_status()) {
                // Only take action if the panel was powered off
                if (!panel_enabled()) {
                    // If the resolution changed, we need to reconfigure
                    // the HDMI receiver for the panel to work
                    uint16_t h_active = h2c_h_active();
                    uint16_t v_active = h2c_v_active();
                    if (panel_resolution_changed(h_active, v_active)) {
                        panel_timing_t timing;
                        // We don't have the actual refresh rate yet, so
                        // get the default
                        panel_get_timing(0, &timing);
                        // Find a timing that matches the new resolution
                        uint8_t index = panel_get_closest_timing(timing.refresh, h_active, v_active);
                        // Get the timing and set it to the HDMI bridge
                        panel_get_timing(index, &timing);
                        h2c_mipi_init(&timing);
                    }
                    
                    // Don't power on panel until HDMI is present
                    panel_power_on();
                    
                    // Repeat the panel enable until it succeeds, since it may
                    // fail if TMDS briefly disappears
                    while (h2c_hdmi_v_status()) {
                        // Switch from Amber to Blue when the screen goes active
                        led_blue_on();
                        led_amber_off();
                    
                        if (panel_enable(1)) {
                            break;
                        } else {
                            // If a MIPI command fails, the H2C and/or panel are
                            // in an bad and unknown state, so we need to
                            // reset the TMDS PHY, the MIPI block (this happens
                            // in the h2c_mipi_write), and the panel
                            h2c_hdmi_phy_suspend(1);
                            panel_enable(0);
                            h2c_mipi_set_tx(0);
                            h2c_hdmi_phy_suspend(0);
                            
                            led_blue_off();
                            led_amber_on();
                            
                            blocking_update(120000);          
                        }
                    }
                    
                    h2c_mipi_set_tx(1);
                }
            } else {
                // Sending MIPI commands while the HDMI PHY is active but there
                // is no input TMDS signal puts the H2C in a state where
                // all MIPI commands fail.  Powering down the HDMI PHY
                // before sending the panel off commands appears to function
                // as a workaround.
                h2c_hdmi_phy_suspend(1);
                panel_enable(0);
                panel_power_off();
                h2c_mipi_set_tx(0);
                // Turn the phy back to auto-on on cable detect to be prepared
                // for the HDMI signal coming back
                h2c_hdmi_phy_suspend(0);
                
                // Switch from Blue to Amber when the screen goes to sleep
                led_blue_off();
                led_amber_on();
            }
        }
        
        // Handle factory calibration commands
        if (factory_calibrating) {
            int command = getchar();
            if (command != EOF) {
                factory_set_command(command);
            }
        }
        
        // Update the gyro/accelerometer part of the in report if there is a new sample
        if (mpu6500_read(&mpu, g_config.use_raw)) {
            if (factory_calibrating) {
                // Update the factory calibrate state machine if we are in the calibration fixture
                factory_update(&mpu);
            } else {
                // Don't store autocalibration if the samples are raw, to avoid bad table items
                autocalibrate_update(&mpu, g_config.autocalibration, !g_config.use_raw);
            }
            if (g_config.use_calibration)
                calibrate_apply(&mpu);
            
            in_report_update_mpu(&mpu);
        }
        
        // Update the magnetometer part of the in report if there is a new sample
        if (lis3mdl_read(&mag, g_config.use_raw)) {            
            in_report_update_mag(&mag);
        }
        
        // Update the tracking part of the in report if there was a new exposure
        if (sequence_get_exposure(&exposure)) {
            in_report_update_tracking(&exposure);
        }
        
        // Update the display part of the in report if there was a new frame
        if (panel_get_vsync(&vsync)) {
            in_report_update_display(&vsync);
        }
        
        // Handle the refresh rate changing
        if (reset_sync) {
            uint16_t refresh = panel_get_refresh();
            if (refresh && panel_enabled()) {
                uint16_t h_active = h2c_h_active();
                uint16_t v_active = h2c_v_active();
                panel_update_timing(refresh, h_active, v_active);
                reset_sync = 0;
                
                // Set the right values on the bridge
                // when the timings change
                // TODO: this potentially also effects other stuff like
                // camera sync when tied to vsync and scaling settings
                panel_timing_t new_timing;
                panel_current_timing(&new_timing);
                h2c_mipi_init(&new_timing);
            }
        }
        
        if (!g_config.command_keep_alive || ((now - feature_report_latest_command_time()) < g_keep_alive.keep_alive)) {
            keep_alive_active = 1;
            sequence_set_timeout(0);
        } else if (keep_alive_active) {
            keep_alive_active = 0;
            // Power off the LED system if we timed out of keep alive to avoid
            // leaving the LEDs and camera running
            sequence_set_timeout(1);
        }
        
        // Only send a new in report if the previous one sent, there is new
        // data, and we are either not using keep alive or we are within
        // the keep alive interval.
        // TODO: Also support motion keep alive?
        if (in_report_ready(g_config.interval) && keep_alive_active) {
            in_report_send();
        }
    }
}
