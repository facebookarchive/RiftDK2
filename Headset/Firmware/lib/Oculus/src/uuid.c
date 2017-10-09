/************************************************************************************

Filename    :   uuid.c
Content     :   Headset serial number interface
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "uuid.h"
#include "config.h"
#include <string.h>

// Include space for a null character
static char g_uuid[UUID_LEN+1] = {0};
static bool g_usb_uuid = 0;
// Use Crockford's Base32 scheme: http://www.crockford.com/wrmg/base32.html
static const char *crockford = "0123456789ABCDEFGHJKMNPQRSTVWXYZ";

#ifdef STM32L1XX_MD
#define UUID_ADDR_0 ((uint32_t *)0x1FF80050)
#define UUID_ADDR_1 ((uint32_t *)0x1FF80054)
#define UUID_ADDR_2 ((uint32_t *)0x1FF80064)
#else
#error "Set the memory addresses for the 96-bit UUID"
#endif /* STM32L1XX */

static void uuid_to_base32(uint8_t *bytes, uint8_t num_bytes, char *buf, uint8_t buf_len)
{
    int8_t num_bits = 0;
    uint16_t bit_buffer = 0;
    uint8_t byte_num = 0;

    for (uint8_t i = 0; i < buf_len; i++) {
        // Load in another byte if we don't have enough bits for the next character
        if (num_bits < 5) {
            // Stop if we run out of bytes to encode
            if (byte_num < num_bytes) {
                bit_buffer |= bytes[byte_num] << num_bits;
                num_bits += 8;
                byte_num++;
            }
        }

        // Select 5 bits and get the Crockford Base32 character for it
        buf[i] = crockford[bit_buffer & 0x1F];

        // Shift out the bits we used
        bit_buffer >>= 5;
        num_bits -= 5;
    }
}

void uuid_init(void)
{
    uint32_t uuid_bytes[3];
    // Copy the 96-bit UUID burnt into the microcontroller
    uuid_bytes[0] = *UUID_ADDR_0;
    uuid_bytes[1] = *UUID_ADDR_1;
    uuid_bytes[2] = *UUID_ADDR_2;
    uuid_to_base32((uint8_t *)uuid_bytes, sizeof(uint32_t)*3, g_uuid, UUID_LEN);
    
    // Also check if USB should be using the UUID for its serial
    uint8_t usb_uuid = 0;
    if (config_read(&usb_uuid, CFG_SIZE_SERIAL_SET, CFG_ADDR_SERIAL_SET) && usb_uuid) {
        g_usb_uuid = 1;
    }
}

void uuid_get(char *uuid_string, uint8_t len, bool use_unicode)
{
    if (use_unicode) {
        // Make sure the buffer is long enough
        if (len < UUID_LEN*2)
            return;

        for (uint8_t i = 0; i < UUID_LEN; i++) {
            uuid_string[i*2] = g_uuid[i];
            uuid_string[i*2+1] = 0;
        }
    } else {
        // Make sure the buffer is long enough
        if (len < UUID_LEN)
            return;

        memcpy(uuid_string, g_uuid, UUID_LEN);
    }
}

bool uuid_use_for_usb(void)
{
    return g_usb_uuid;
}
