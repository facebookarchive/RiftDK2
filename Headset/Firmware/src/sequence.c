/************************************************************************************

Filename    :   sequence.c
Content     :   LED sequence driving
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "sequence.h"
#include "sequence_patterns.h"
#include "shiftout.h"
#include "gpio.h"
#include "hw_config.h"
#include "stm32l1xx.h"
#include "timestamper.h"
#include "hub.h"
#include <string.h>
#include <math.h>

// Glossary, since the naming in here is confusing
// pattern - The specific configuration of all LEDs in a single frame.  All
//           LEDs are either off (0), at low brightness (1), or at high (3)
// sequence - The set of states that a specific LED goes through sequentially
//            from frame to frame.

#define NUM_BYTES (NUM_LEDS / 4)

#define CARRIER_FREQ (85000)
#define CARRIER_PERIOD (SystemCoreClock / CARRIER_FREQ)
#define CARRIER_PULSE (CARRIER_PERIOD / 2)
#define DEFAULT_EXPOSURE (120)
#define SEQUENCE_INTERVAL (16667)
// This is a power of two to make the modulo just a bitwise AND
#define EXPOSURE_BUFFER_SLOTS 4
#define SLOT_FOR_FRAME(x) ((x) & (EXPOSURE_BUFFER_SLOTS - 1))

// Use a ring buffer to store exposures to avoid memory access issues between
// the main thread and the interrupt that performs the exposure
static exposure_t exposure_buffer[EXPOSURE_BUFFER_SLOTS] = {{0}};

static uint8_t default_packed_patterns[NUM_PATTERNS_DEFAULT][NUM_BYTES];
static const uint8_t steady_state_pattern[NUM_BYTES] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t custom_packed_patterns[NUM_PATTERNS_MAX][NUM_BYTES] = {{0}};

static sequence_t g_sequence = {0};
static uint32_t g_duty_cycle = 0;
static uint32_t g_last_frame = 0;
static bool g_index_changed = 0;
static bool g_timed_out = 0;
static uint8_t g_num_custom_patterns = 0;
static uint8_t g_num_patterns = NUM_PATTERNS_DEFAULT;
static uint8_t g_custom_pattern_led_index = 0;
static TIM_OCInitTypeDef g_oe_ocinit;

static void sequence_generate_default_patterns(void)
{
    // Generate the packed bits for each LED pattern
    for (uint8_t i = 0; i < NUM_PATTERNS_DEFAULT; i++) {
        const uint8_t *pattern = sequence_patterns[i];
        uint8_t *b = default_packed_patterns[i];
        uint8_t b_index = 0;

        // Pack the array of LEDs into the bytes to shift out, MSB first
        for (uint8_t j = 0; j < NUM_LEDS; j++) {
            *b |= (pattern[NUM_LEDS - 1 - j] & 0x03) << (6 - b_index);
            b_index += 2;
            if (b_index == 8) {
                b++;
                b_index = 0;
            }
        }
    }
}

static void sequence_init_sync_input(void)
{   
    // Init sync input for cameras like the Point Grey that only work
    // well free running
    gpio_config(CAM_SYNC_IN_PORT, CAM_SYNC_IN_PIN, GPIO_IN_PD);
    SYSCFG_EXTILineConfig(CAM_SYNC_IN_IRQ_SOURCE_PORT, CAM_SYNC_IN_IRQ_SOURCE);
    // Also configure the wall wart detect coming from the cable as an input
    // with an interrupt on it
    SYSCFG_EXTILineConfig(WW_DETECT_IRQ_SOURCE_PORT, WW_DETECT_IRQ_SOURCE);
    
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    // This needs to be a lower priority than the timestamper overflow
    // for the timestamp to be correct
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static void sequence_config_sync_input(bool state)
{
    if (state) {        
        // Enable sync input for cameras like the Point Grey that only work
        // well free running
        EXTI_InitTypeDef EXTI_InitStructure;
        EXTI_InitStructure.EXTI_Line = CAM_SYNC_IN_IRQ_LINE;
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
        EXTI_Init(&EXTI_InitStructure);
        
        gpio_config(WW_DETECT_PORT, WW_DETECT_PIN, GPIO_IN);
        EXTI_InitStructure.EXTI_Line = WW_DETECT_IRQ_LINE;
        EXTI_Init(&EXTI_InitStructure);
    } else {
        // Shut off the EXTI and go floating on the input when using sync out
        // instead of in
        EXTI_InitTypeDef EXTI_InitStructure;
        EXTI_InitStructure.EXTI_Line = CAM_SYNC_IN_IRQ_LINE;
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
        EXTI_InitStructure.EXTI_LineCmd = DISABLE;
        EXTI_Init(&EXTI_InitStructure);
        
        EXTI_InitStructure.EXTI_Line = WW_DETECT_IRQ_LINE;
        EXTI_Init(&EXTI_InitStructure);
    }
}

void sequence_init_sync_output(void)
{   
    // Configure the 2.5mm sync output
    gpio_config(CAM_SYNC_OUT_PORT, CAM_SYNC_OUT_PIN, GPIO_PP);
    GPIO_ResetBits(CAM_SYNC_OUT_PORT, CAM_SYNC_OUT_PIN);
    
    // Set up TIM3 as the sync output timer
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
    TIM_Cmd(SYNC_OUTPUT_TIM, DISABLE);
    
    // Set up the timer to allow for the highest precision update interrupt
    // for the specified exposure interval
    float framerate = 1000000.0f / (float)g_sequence.interval;
    float max_framerate = (float)SystemCoreClock / (float)0x10000;
    // First determine what the core clock needs to be divided by for the
    // timer clock to fit the desired framerate
    float divider = ceilf(max_framerate / framerate);
    // Then determine the period that gets it closest to the desired framerate
    float period = lroundf(((float)SystemCoreClock / divider) / framerate);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = (uint32_t)period;
    TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)divider - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(SYNC_OUTPUT_TIM, &TIM_TimeBaseStructure);
    
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    // This needs to be lower priority than the timestamper interrupt, since
    // we get the timestamp from it
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    TIM_Cmd(SYNC_OUTPUT_TIM, ENABLE);
}

static void sequence_config_sync_output(bool state)
{
    if (state) {
        // The camera triggers on rising edge, so go push pull low
        // Wait until sync out is actually used to do this to allow the
        // hub to use it as a wall wart detect during power on
        gpio_config(WW_DETECT_PORT, WW_DETECT_PIN, GPIO_PP);
        GPIO_ResetBits(WW_DETECT_PORT, WW_DETECT_PIN);
        
        // Only enable the interrupt when sync output is actually used
        TIM_ClearITPendingBit(SYNC_OUTPUT_TIM, TIM_IT_Update);
        TIM_ITConfig(SYNC_OUTPUT_TIM, TIM_IT_Update, ENABLE);
    } else {
        TIM_ITConfig(SYNC_OUTPUT_TIM, TIM_IT_Update, DISABLE);
    }
}

void sequence_send(bool expose)
{
    // Only do anything if we've enabled the LED sequence and we haved timed
    // out of sending
    if (!g_sequence.enable || g_timed_out)
        return;
    
    // Cache the pattern so we ensure the one exposed is what is recorded
    // in the buffer
    uint8_t pattern = g_sequence.pattern;

    // Only shift out the bits if the pattern changed
    if (g_index_changed) {
        if (pattern == STEADY_STATE_PATTERN) {
            // Go to the all LEDs on pattern if that is set
            shiftout_shift(steady_state_pattern, NUM_BYTES);
        } else if (g_sequence.custom_pattern) {
            // Otherwise use new the custom pattern if that is set
            shiftout_shift(custom_packed_patterns[pattern], NUM_BYTES);
        } else {
            // Otherwise grab the new one out of the default pattern buffer
            shiftout_shift(default_packed_patterns[pattern], NUM_BYTES);
        }
    }
    
    if (expose) {
        // Grab the buffer to store exposure information hopefully atomically
        exposure_p exposure = &exposure_buffer[SLOT_FOR_FRAME(g_last_frame + 1)];
        exposure->frame = g_last_frame + 1;
        
        // Reconfigure the end of exposure timer
        TIM_SetCounter(EXPOSURE_TIM, 0);
        TIM_SetAutoreload(EXPOSURE_TIM, g_sequence.exposure_length);
        // Reconfigure the 85 KHz modulation pwm timer
        TIM_SetCounter(SR_OE_TIM, 0);
        TIM_OC2Init(SR_OE_TIM, &g_oe_ocinit);
        TIM_SetCompare2(SR_OE_TIM, g_sequence.use_carrier ? g_duty_cycle : 0);
        
        // Set the 2.5mm and ID pin sync outputs
        GPIO_SetBits(CAM_SYNC_OUT_PORT, CAM_SYNC_OUT_PIN);
        if (!g_sequence.sync_input) {
            GPIO_SetBits(WW_DETECT_PORT, WW_DETECT_PIN);
        }
        
        // Enable the timers
        TIM_Cmd(SR_OE_TIM, ENABLE);
        TIM_Cmd(EXPOSURE_TIM, ENABLE);
        
        exposure->timestamp = timestamper_get_time();
        exposure->pattern = pattern;
        g_last_frame++;
    }
    
    // Post-increment so that if a pattern is specifically set, it is used first
    if (g_sequence.autoincrement)
        sequence_next();
}

void sequence_init(void)
{
    // Load the default values
    g_duty_cycle = CARRIER_PULSE;
    g_sequence.exposure_length = DEFAULT_EXPOSURE;
    g_sequence.interval = SEQUENCE_INTERVAL;
    g_sequence.pattern = 0;
    g_sequence.pulse_duty = sequence_get_duty();
    g_sequence.autoincrement = 1;
    g_sequence.enable = 0;
    g_sequence.use_carrier = 1;
    // Since the drivers may have been powered down,
    // set the pattern to be loaded on the next sync
    g_index_changed = 1;
    g_timed_out = 0;
    
    shiftout_init();
    
    // Initialze the carrier signal output
    RCC_APB2PeriphClockCmd(SR_OE_TIM_RCC, ENABLE);
 
    // Run the output enable PWMed at the carrier frequency of the IR reciever
    gpio_config(SR_OE_PORT, SR_OE_PIN, GPIO_AF_PP);
    GPIO_PinAFConfig(SR_OE_PORT, SR_OE_SOURCE, SR_OE_AF);
    
    // Set up the frequency of the timer
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = CARRIER_PERIOD;
    TIM_TimeBaseInit(SR_OE_TIM, &TIM_TimeBaseStructure);
    
    // Set up the output compare for PWM
    g_oe_ocinit.TIM_OCMode = TIM_OCMode_PWM2;
    g_oe_ocinit.TIM_OutputState = TIM_OutputState_Enable;
    g_oe_ocinit.TIM_Pulse = 0;
    g_oe_ocinit.TIM_OCPolarity = TIM_OCPolarity_Low;
    // Use the right channel for the pin
    TIM_OC2Init(SR_OE_TIM, &g_oe_ocinit);
    TIM_OC2PreloadConfig(SR_OE_TIM, TIM_OCPreload_Enable);
    
    // Set up a timer for the exposure length
    RCC_APB1PeriphClockCmd(EXPOSURE_TIM_RCC, ENABLE);
    
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = g_sequence.exposure_length;
    // Count microseconds
    TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock / 1000000) - 1);
    TIM_TimeBaseInit(EXPOSURE_TIM, &TIM_TimeBaseStructure);
    // We want to trigger each exposure explicitly
    TIM_SelectOnePulseMode(EXPOSURE_TIM, TIM_OPMode_Single);
    
    // Set up the interrupt for the end of timer
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = EXPOSURE_TIM_IRQ;
    // This needs to be a high priority to avoid keeping the LEDs on for too long
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_ClearITPendingBit(EXPOSURE_TIM, TIM_IT_Update);
    TIM_ITConfig(EXPOSURE_TIM, TIM_IT_Update, ENABLE);

    // Initialize the sync output timer
    sequence_init_sync_output();
    // Initialize the sync input interrupt
    sequence_init_sync_input();
    
    sequence_generate_default_patterns();

    // Enable the 2.0V supply for the IR LEDs
    // This needs to happen before the shift registers can be used
    gpio_config(IR_LED_PWR_EN_PORT, IR_LED_PWR_EN_PIN, GPIO_PP);
    GPIO_SetBits(IR_LED_PWR_EN_PORT, IR_LED_PWR_EN_PIN);
    
    // Start as sync output by default.  Note that we start with Enable = 0,
    // so the LEDs won't start blasting
    sequence_set_sync_input(0);
}

void sequence_deinit(void)
{
    sequence_set_enable(0); 
    sequence_config_sync_output(0);
    sequence_config_sync_input(0);

    gpio_config(SR_PORT, SR_SPI_SCK_PIN, GPIO_PP);
    gpio_set_state(SR_PORT, SR_SPI_SCK_PIN, 0);

    gpio_config(SR_PORT, SR_SPI_SDI_PIN, GPIO_PP);
    gpio_set_state(SR_PORT, SR_SPI_SDI_PIN, 0);

    gpio_config(SR_RCLK_PORT, SR_RCLK_PIN, GPIO_PP);
    gpio_set_state(SR_RCLK_PORT, SR_RCLK_PIN, 0);

    gpio_config(SR_OE_PORT, SR_OE_PIN, GPIO_PP);
    gpio_set_state(SR_OE_PORT, SR_OE_PIN, 0);
}

void sequence_next(void)
{
    // Go to the next pattern, wrapping around
    g_sequence.pattern++;
    if (g_sequence.pattern >= g_num_patterns)
        g_sequence.pattern = 0;
    g_index_changed = 1;
}

void sequence_end_exposure(void)
{
    // Stop the led output enable
    TIM_ForcedOC2Config(SR_OE_TIM, TIM_ForcedAction_InActive);
    
    // Set the 2.5mm and ID sync outputs low
    GPIO_ResetBits(CAM_SYNC_OUT_PORT, CAM_SYNC_OUT_PIN);
    if (!g_sequence.sync_input) {
        // Go back low
        GPIO_ResetBits(WW_DETECT_PORT, WW_DETECT_PIN);
    }
}

void sequence_get_state(sequence_p sequence)
{
    memcpy(sequence, &g_sequence, sizeof(sequence_t));
}

void sequence_set_state(sequence_p sequence)
{
    // TODO: make this atomic from the perspective of the LEDs
    sequence_set_autoincrement(sequence->autoincrement);
    sequence_set_enable(sequence->enable);
    sequence_set_use_carrier(sequence->use_carrier);
    sequence_set_pattern(sequence->pattern);
    sequence_set_duty(sequence->pulse_duty);
    sequence_set_interval(sequence->interval);
    sequence_set_exposure_length(sequence->exposure_length);
    sequence_set_sync_input(sequence->sync_input);
    sequence_set_use_custom_pattern(sequence->custom_pattern);
}

void sequence_get_custom_pattern(sequence_pattern_p pattern)
{
    pattern->led_index = g_custom_pattern_led_index;
    pattern->num_leds = NUM_LEDS;
    pattern->sequence_length = g_num_patterns;
    
    // Clear the sequence to start with
    pattern->sequence = 0;
    
    // Find the right byte bin and offset for the pattern, MSB first
    uint8_t byte_num = NUM_BYTES - (pattern->led_index >> 2) - 1;
    uint8_t byte_offset = (pattern->led_index & 3) * 2;
    for (uint8_t i = 0; i < pattern->sequence_length; i++) {
        // Copy the right bits for the LED from each pattern, MSB first
        if (g_sequence.custom_pattern) {
            pattern->sequence |= (((custom_packed_patterns[i][byte_num]) >> byte_offset) & 3) << (i * 2);
        } else {
            pattern->sequence |= (((default_packed_patterns[i][byte_num]) >> byte_offset) & 3) << (i * 2);
        }
    }
    
    // Autoincrement on reads, wrapping at the final LED
    g_custom_pattern_led_index++;
    if (g_custom_pattern_led_index == NUM_LEDS)
        g_custom_pattern_led_index = 0;
}

void sequence_set_custom_pattern(sequence_pattern_p pattern)
{
    // Only take action if the led index and length were valid
    if ((pattern->led_index < NUM_LEDS) &&
        g_sequence.custom_pattern &&
        pattern->sequence_length &&
        (pattern->sequence_length <= NUM_PATTERNS_MAX)) {
        
        // Handle the sequence length changing
        if (pattern->sequence_length != g_num_custom_patterns) {
            if (pattern->sequence_length < g_num_custom_patterns) {
                // If we are now going to a smaller number of patterns,
                // clear out any that were stored past this to avoid
                // weird state if we later go longer again
                for (uint8_t i = pattern->sequence_length; i < g_num_custom_patterns; i++) {
                    memset(custom_packed_patterns[i], 0, NUM_BYTES);
                }
            }
                    
            g_num_custom_patterns = pattern->sequence_length;
            g_num_patterns = g_num_custom_patterns;
            // If the current pattern is out of bounds of the new number of
            // patterns, start again at 0
            sequence_set_pattern(g_sequence.pattern);
        }
        
        // Find the right byte bin and offset for the pattern, MSB first
        uint8_t byte_num = NUM_BYTES - (pattern->led_index >> 2) - 1;
        uint8_t byte_offset = (pattern->led_index & 3) * 2;
        for (uint8_t i = 0; i < g_num_custom_patterns; i++) {
            // Clear what was set before
            custom_packed_patterns[i][byte_num] &= ~(3 << byte_offset);
            // Write in the new bits for this pattern index
            custom_packed_patterns[i][byte_num] |= ((pattern->sequence >> (i * 2)) & 3) << byte_offset;
        }
        
        // Set the read index to the write for easier read back verification
        g_custom_pattern_led_index = pattern->led_index;
        
        // Mark that the pattern has changed
        g_index_changed = 1;
    }
}

void sequence_set_pattern(uint8_t pattern)
{
    // Make sure the pattern is a valid index
    if ((pattern >= g_num_patterns) && (pattern != STEADY_STATE_PATTERN))
        pattern = 0;
    g_sequence.pattern = pattern;
    g_index_changed = 1;
}

uint8_t sequence_get_duty(void)
{
    // Turn back into a 0-255 value
    return UINT8_MAX - lroundf((float)(g_duty_cycle * 255) / (float)CARRIER_PERIOD);
}

void sequence_set_duty(uint8_t duty)
{
    g_sequence.pulse_duty = duty;
    // Turn into a timer count for PWM
    g_duty_cycle = lroundf((float)((UINT8_MAX - duty) * CARRIER_PERIOD) / 255.0f);
}

void sequence_set_exposure_length(uint16_t length)
{
    // Make sure the length isn't above the frame length
    if (length > g_sequence.interval) {
        length = g_sequence.interval;
    } else if (length < 10) {
        // Extremely short exposures are not likely to work well
        length = 10;
    }
    
    g_sequence.exposure_length = length;
}

void sequence_set_interval(uint16_t interval)
{
    // Prevent intervals short enough to cause excessive interrupt load
    if (interval < 1000)
        interval = 1000;
    
    if (interval != g_sequence.interval) {
        g_sequence.interval = interval;
        
        // Reconfigure the sync timer for the new exposure framerate
        sequence_init_sync_output();
    }
}

void sequence_set_timeout(bool timed_out)
{
    g_timed_out = timed_out;
}

void sequence_set_enable(bool enable)
{    
    // Turn on/off the LED and LED driver power supplies
    if (enable) {
        // The pattern needs to be re-loaded into the now empty shift registers
        if (!g_sequence.enable)
            g_index_changed = 1;
        GPIO_SetBits(IR_LED_PWR_EN_PORT, IR_LED_PWR_EN_PIN);
        g_timed_out = 0;
    } else {
        GPIO_ResetBits(IR_LED_PWR_EN_PORT, IR_LED_PWR_EN_PIN);
    }
    
    g_sequence.enable = enable;
}

void sequence_set_autoincrement(bool autoincrement)
{
    g_sequence.autoincrement = autoincrement;
}

void sequence_set_use_carrier(bool use_carrier)
{
    g_sequence.use_carrier = use_carrier;
}

void sequence_set_sync_input(bool sync_input)
{
    g_sequence.sync_input = sync_input;
    // Sync input and output are mutually exclusive.  Disable one before 
    // enabling the other
    if (sync_input) {
        sequence_config_sync_output(0);
        sequence_config_sync_input(1);
    } else {
        sequence_config_sync_input(0);
        sequence_config_sync_output(1);
    }
}

void sequence_set_use_custom_pattern(bool custom_pattern)
{
    g_sequence.custom_pattern = custom_pattern;
    
    // Set the number of patterns based on if we use the default or a custom pattern set
    if (g_sequence.custom_pattern) {
        g_num_patterns = g_num_custom_patterns;
    } else {
        g_num_patterns = NUM_PATTERNS_DEFAULT;
    }
    
    // Reset the LEDs and make sure the pattern index is within bounds
    sequence_set_pattern(g_sequence.pattern);
}

bool sequence_get_exposure(exposure_p exposure)
{
    if (exposure->frame != g_last_frame) {
        // Grab the most recent exposure from the ring buffer
        memcpy(exposure, &exposure_buffer[SLOT_FOR_FRAME(g_last_frame)], sizeof(exposure_t));
        return 1;
    }
    
    return 0;
}

void sequence_trigger(bool input)
{
    // Make sure the trigger matches the current expected sync source
    if (input == g_sequence.sync_input) {
        sequence_send(1);
    }
}
