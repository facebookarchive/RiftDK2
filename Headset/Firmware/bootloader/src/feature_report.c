/************************************************************************************

Filename    :   feature_report.c
Content     :   DK2 Bootloader feature reports
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "feature_report.h"
#include "bootloader.h"
#include <stdlib.h>
#include <string.h>

enum {
    // Bootloader reports
    FEATURE_CONFIG = 2,
    FEATURE_COMMAND = 3,
    FEATURE_MEMORY_REGION = 4,
    FEATURE_PAYLOAD = 5,
    FEATURE_MAX = 6
};


// Report ID and Command ID
#define FEATURE_HEADER_SIZE 3
#define FEATURE_MAX_SIZE 1030   // FEATURE_PAYLOAD

static const uint16_t report_sizes[FEATURE_MAX] = {
    0,   // Index 0, not valid for reports?
    0,   // Not used
    4,   // FEATURE_CONFIG
    6,   // FEATURE_COMMAND
    12,  // FEATURE_MEMORY_REGION
    1030 // FEATURE_PAYLOAD
};   

// For the largest buffer we need
uint8_t feature_buf[FEATURE_MAX_SIZE];

uint16_t feature_report_length(uint8_t report_id, uint16_t length)
{
    // Switch based on ReportID
    switch (report_id) {
        case FEATURE_CONFIG:
        case FEATURE_COMMAND:
        case FEATURE_MEMORY_REGION:
        case FEATURE_PAYLOAD:
            return report_sizes[report_id];
        default:
            break;
    }

    return 0;
}

uint8_t *feature_report_get_buf(uint8_t report_id, uint16_t offset)
{
    memory_region_s memory_region;
    feature_buf[0] = report_id;
    
    // Switch based on ReportID
    switch (report_id) {
        case FEATURE_CONFIG:
            bootloader_get_config((uint16_t *)(feature_buf + 1), feature_buf + 3);
            return feature_buf;

        case FEATURE_COMMAND:
            bootloader_get_command(feature_buf + 1, (uint32_t *)(feature_buf + 2));
            return feature_buf;

        case FEATURE_MEMORY_REGION:
            bootloader_get_memory_region(&memory_region);
            feature_buf[1] = memory_region.region_num;
            feature_buf[2] = memory_region.region_type;
            feature_buf[3] = memory_region.region_mask;
            *(uint32_t *)(feature_buf + 4) = memory_region.num_blocks;
            *(uint16_t *)(feature_buf + 8) = memory_region.block_size;
            *(uint16_t *)(feature_buf + 10) = memory_region.write_time;
            return feature_buf;

        case FEATURE_PAYLOAD:
            if (!offset) {
                // If this is the first call, fill the buffer
                bootloader_get_payload(feature_buf + 1, (uint32_t *)(feature_buf + 2),
                                       feature_buf + 6);
            }
            // otherwise just return the right offset into it
            return feature_buf + offset;
            
        default:
            break;
    }

    return NULL;
}

uint8_t *feature_report_set_buf(uint8_t report_id, uint16_t offset)
{
    // Switch based on ReportID
    switch (report_id) {
        case FEATURE_PAYLOAD:
            // Payload is over 64 bytes, so there are multiple sets
            return feature_buf + offset;
            
        case FEATURE_CONFIG:
        case FEATURE_COMMAND:
        case FEATURE_MEMORY_REGION:
            return feature_buf;
        default:
            break;
    }

    return NULL;
}

void feature_report_parse(void)
{
    switch (feature_buf[0]) {
        case FEATURE_CONFIG:
            // Setting this report doesn't do anything for DK2
            break;

        case FEATURE_COMMAND:
            bootloader_set_command(feature_buf[1], *(uint32_t *)(feature_buf + 2));
            break;

        case FEATURE_MEMORY_REGION:
            // Setting this report doesn't do anything for DK2
            break;

        case FEATURE_PAYLOAD:
            bootloader_set_payload(feature_buf[1], *(uint32_t *)(feature_buf + 2),
                                   feature_buf + 6);
            break;

        default:
            break;
    }
}
