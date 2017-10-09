/************************************************************************************

Filename    :   sequence.h
Content     :   LED sequence driving
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _SEQUENCE_H_
#define _SEQUENCE_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct exposure_struct {
    uint32_t timestamp;
    uint16_t frame;
    uint8_t pattern;
} exposure_t, *exposure_p;

typedef struct sequence_struct {
    uint16_t exposure_length;
    uint16_t interval;
    uint8_t pattern;
    uint8_t pulse_duty;
    bool autoincrement;
    bool enable;
    bool use_carrier;
    bool sync_input;
    bool custom_pattern;
} sequence_t, *sequence_p;

typedef struct sequence_pattern_struct {
    uint8_t sequence_length;
    uint32_t sequence;
    uint16_t led_index;
    uint16_t num_leds;
} sequence_pattern_t, *sequence_pattern_p;

void sequence_init(void);

void sequence_deinit(void);

void sequence_send(bool expose);

void sequence_next(void);

void sequence_end_exposure(void);

void sequence_get_state(sequence_p sequence);

void sequence_set_state(sequence_p sequence);

void sequence_get_custom_pattern(sequence_pattern_p pattern);

void sequence_set_custom_pattern(sequence_pattern_p pattern);

void sequence_set_pattern(uint8_t pattern);

uint8_t sequence_get_duty(void);

void sequence_set_duty(uint8_t duty);

void sequence_set_exposure_length(uint16_t length);

uint16_t sequence_get_interval(void);

void sequence_set_interval(uint16_t interval);

void sequence_set_timeout(bool timed_out);

void sequence_set_enable(bool enable);

void sequence_set_autoincrement(bool autoincrement);

void sequence_set_use_carrier(bool use_carrier);

void sequence_set_sync_input(bool sync_input);

void sequence_set_use_custom_pattern(bool custom_pattern);

bool sequence_get_exposure(exposure_p exposure);

void sequence_trigger(bool input);

#endif /* _SEQUENCE_H_ */
