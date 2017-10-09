/************************************************************************************

Filename    :   manufacturing.c
Content     :   Manufacturing information storage
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "manufacturing.h"
#include "config.h"
#include <string.h>

typedef struct manufacturing_struct {
    uint32_t stage_time;
    uint32_t result;
    uint16_t stage_location;
    uint8_t stage;
    uint8_t stage_version;
} manufacturing_t, *manufacturing_p;

#define NUM_STAGES (CFG_NUM_MANUFACTURING)

static uint8_t stage_index = 0;

static void manufacturing_parse_report(manufacturing_p stage, uint8_t *buf)
{
    stage->stage = buf[0];
    stage->stage_version = buf[1];
    stage->stage_location = *(uint16_t *)(buf + 2);
    stage->stage_time = *(uint32_t *)(buf + 4);
    stage->result = *(uint32_t *)(buf + 8);
}

static void manufacturing_pack_report(manufacturing_p stage, uint8_t *buf)
{
    buf[0] = stage->stage;
    buf[1] = stage->stage_version;
    *(uint16_t *)(buf + 2) = stage->stage_location;
    *(uint32_t *)(buf + 4) = stage->stage_time;
    *(uint32_t *)(buf + 8) = stage->result;
}

static void manufacturing_parse_eeprom(manufacturing_p stage, uint8_t *buf)
{
    stage->stage_version = buf[0];
    stage->stage_location = *(uint16_t *)(buf + 1);
    stage->stage_time = *(uint32_t *)(buf + 3);
    stage->result = *(uint32_t *)(buf + 7);
}

static void manufacturing_pack_eeprom(manufacturing_p stage, uint8_t *buf)
{
    buf[0] = stage->stage_version;
    *(uint16_t *)(buf + 1) = stage->stage_location;
    *(uint32_t *)(buf + 3) = stage->stage_time;
    *(uint32_t *)(buf + 7) = stage->result;
}

static void manufacturing_store(manufacturing_p stage)
{
    uint8_t buf[CFG_SIZE_MANUFACTURING];
    manufacturing_pack_eeprom(stage, buf);
    
    // The stage number is the offset into the array of stored manufacturing stages
    config_write(buf, CFG_SIZE_MANUFACTURING, CFG_ADDR_MANUFACTURING + CFG_OFFSET_MANUFACTURING(stage->stage));
}

static void manufacturing_load(uint8_t index, manufacturing_p stage)
{
    stage->stage = index;
    
    uint8_t buf[CFG_SIZE_MANUFACTURING];
    // If the stage isn't found, zero the fields
    if (!config_read(buf, CFG_SIZE_MANUFACTURING, CFG_ADDR_MANUFACTURING + CFG_OFFSET_MANUFACTURING(stage->stage)))
        memset(buf, 0, CFG_SIZE_MANUFACTURING);
    
    manufacturing_parse_eeprom(stage, buf);
}

void manufacturing_get(uint8_t *buf)
{
    buf[0] = NUM_STAGES;
    manufacturing_t stage = {0};
    manufacturing_load(stage_index, &stage);
    manufacturing_pack_report(&stage, buf + 1);
    
    // Autoincrement on reads
    stage_index++;
    if (stage_index >= NUM_STAGES)
        stage_index = 0;
}

void manufacturing_set(uint8_t *buf)
{
    manufacturing_t stage = {0};
    manufacturing_parse_report(&stage, buf + 1);
 
    // Make sure this is a valid stage number
    if (stage.stage < NUM_STAGES) {
        manufacturing_store(&stage);
        
        // Set the index on write for quicker read back verification
        stage_index = stage.stage;
    }
}
