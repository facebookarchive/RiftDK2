/************************************************************************************
Filename    :   m24.h
Content     :   Interface to M24 series EEPROM
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _M24_H_
#define _M24_H_

#include <stdbool.h>
#include <stdint.h>
#include "cpal.h"

#define M24_PAGE_SIZE (32)

typedef struct m24_struct {
    CPAL_TransferTypeDef rx;
    CPAL_TransferTypeDef tx;
    uint8_t buf[M24_PAGE_SIZE];
    CPAL_InitTypeDef *cpal;
    CPAL_DevTypeDef device;
    uint32_t options;
    uint8_t address;
} m24_t, *m24_p;

bool m24_init(m24_p m24);

void m24_deinit(m24_p m24);

bool m24_write(m24_p m24, uint32_t address, const uint8_t *data, uint16_t length);

bool m24_read(m24_p m24, uint32_t address, uint8_t *data, uint16_t length);

#endif /* _M24_H_ */
