/************************************************************************************

Filename    :   config.h
Content     :   Persistant application configuration handling.
Created     :
Authors     :   Lyle Bainbridge

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _CONFIG_H_
#define _CONFIG_H_

#if USE_INTERNAL_EEPROM
#include "eeprom.h"
#define EEPROM_START_ADDR INTERNAL_EEPROM_START_ADDR
#define EEPROM_PAGE_SIZE  INTERNAL_EEPROM_PAGE_SIZE
#else /* USE_INTERNAL_EEPROM */
#include "m24.h"
#define EEPROM_START_ADDR (0x0)
#define EEPROM_PAGE_SIZE  M24_PAGE_SIZE
#endif /* USE_INTERNAL_EEPROM */

// Page align an address for faster reads and writes
#define CFG_ALIGN(a) ((a + (EEPROM_PAGE_SIZE - 1)) & ~(EEPROM_PAGE_SIZE - 1))

// Each item stored in EEPROM has a 16 bit crc
#define CFG_SIZE_HEADER sizeof(uint16_t)

#define CFG_SIZE_CALIBRATE (100)
#define CFG_SIZE_BOOTLOAD (sizeof(uint8_t))
#define CFG_SIZE_SERIAL_SET (sizeof(uint8_t))
#define CFG_SIZE_DISPLAY_INFO (53)
#define CFG_SIZE_CALIBRATE_MAG (sizeof(uint32_t) * 4 * 3 + sizeof(uint16_t))

// There are multiple sets of position calibration
#define CFG_SIZE_CALIBRATE_POS (sizeof(int32_t) * 3 + sizeof(int16_t) * 4 + sizeof(uint8_t))
#define CFG_NUM_CALIBRATE_POS (41)
#define CFG_OFFSET_CALIBRATE_POS(i) ((CFG_SIZE_CALIBRATE_POS + CFG_SIZE_HEADER) * i)

// There are multiple sets of temperature data
#define CFG_SIZE_TEMPERATURE (8 + sizeof(uint32_t) + sizeof(uint16_t) + sizeof(int8_t))
// Page align the temperature data since it'll be written live
#define CFG_SIZE_TEMPERATURE_PAGE (32)
#define CFG_NUM_TEMPERATURE_SAMPLES (5)
#define CFG_NUM_TEMPERATURE_BINS (7)
#define CFG_OFFSET_TEMPERATURE(b, s) (CFG_SIZE_TEMPERATURE_PAGE * (CFG_NUM_TEMPERATURE_SAMPLES * b + s))

// There are multiple sets of manufacturing data
#define CFG_SIZE_MANUFACTURING (sizeof(uint32_t) * 2 + sizeof(uint16_t) + sizeof(uint8_t))
#define CFG_NUM_MANUFACTURING (10)
#define CFG_OFFSET_MANUFACTURING(i) ((CFG_SIZE_MANUFACTURING + CFG_SIZE_HEADER) * i)

#define CFG_SIZE_LENSDISTORTION (59)
#define CFG_NUM_LENSDISTORTION (2)
#define CFG_OFFSET_LENSDISTORTION(i) ((CFG_SIZE_LENSDISTORTION + CFG_SIZE_HEADER) * i)

#define CFG_SIZE_OVERRIDES (sizeof(uint8_t))

#define CFG_SIZE_SERIAL (12)

// Configuration data memory map in EEPROM 
// Note: Next configuration block address, is previous block address plus
// the size of the data stored in that block, plus the size of the crc16
enum {
    CFG_ADDR_CALIBRATE = EEPROM_START_ADDR,
    CFG_ADDR_BOOTLOAD = CFG_ADDR_CALIBRATE + CFG_SIZE_CALIBRATE + CFG_SIZE_HEADER,
    CFG_ADDR_SERIAL_SET = CFG_ADDR_BOOTLOAD + CFG_SIZE_BOOTLOAD + CFG_SIZE_HEADER,
    CFG_ADDR_DISPLAY_INFO = CFG_ADDR_SERIAL_SET + CFG_SIZE_SERIAL_SET + CFG_SIZE_HEADER,
    CFG_ADDR_CALIBRATE_MAG = CFG_ADDR_DISPLAY_INFO + CFG_SIZE_DISPLAY_INFO + CFG_SIZE_HEADER,
    CFG_ADDR_CALIBRATE_POS = CFG_ADDR_CALIBRATE_MAG + CFG_SIZE_CALIBRATE_MAG + CFG_SIZE_HEADER,
    CFG_ADDR_TEMPERATURE = CFG_ALIGN(CFG_ADDR_CALIBRATE_POS + (CFG_SIZE_CALIBRATE_POS + CFG_SIZE_HEADER) * CFG_NUM_CALIBRATE_POS),
    CFG_ADDR_MANUFACTURING = CFG_ALIGN(CFG_ADDR_TEMPERATURE + CFG_SIZE_TEMPERATURE_PAGE * CFG_NUM_TEMPERATURE_SAMPLES * CFG_NUM_TEMPERATURE_BINS),
    CFG_ADDR_LENSDISTORTION = CFG_ALIGN(CFG_ADDR_MANUFACTURING + (CFG_SIZE_MANUFACTURING + CFG_SIZE_HEADER) * CFG_NUM_MANUFACTURING),
    CFG_ADDR_OVERRIDES = CFG_ALIGN(CFG_ADDR_LENSDISTORTION + (CFG_SIZE_LENSDISTORTION + CFG_SIZE_HEADER) * CFG_NUM_LENSDISTORTION),
    CFR_ADDR_SERIAL = CFG_ALIGN(CFG_ADDR_OVERRIDES + CFG_SIZE_OVERRIDES + CFG_SIZE_HEADER)
};

uint8_t config_init(void);

uint8_t config_write(uint8_t* p_data, uint16_t len, uint32_t address);

uint8_t config_write_raw(uint8_t* p_data, uint16_t len, uint32_t address);

uint8_t config_read(uint8_t* p_data, uint16_t len, uint32_t address);

uint8_t config_read_raw(uint8_t* p_data, uint16_t len, uint32_t address);

#endif /* _CONFIG_H_ */
