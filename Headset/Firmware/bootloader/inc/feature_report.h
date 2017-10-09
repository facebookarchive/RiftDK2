/************************************************************************************

Filename    :   feature_report.h
Content     :   DK2 Bootloader feature reports
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _FEATURE_REPORT_H_
#define _FEATURE_REPORT_H_

#include <stdint.h>
#include <stdbool.h>

// Callbacks for USB
uint16_t feature_report_length(uint8_t report_id, uint16_t length);
uint8_t *feature_report_get_buf(uint8_t report_id, uint16_t offset);
uint8_t *feature_report_set_buf(uint8_t report_id, uint16_t offset);

void feature_report_parse(void);

#endif /* _FEATURE_REPORT_H_ */
