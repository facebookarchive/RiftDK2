/************************************************************************************

Filename    :   edid.c
Content     :   EDID generation
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include <string.h>
#include <math.h>
#include "panel.h"
#include "edid.h"

// Aspect ratios for the standard timings block
enum {
    RATIO_16_10 = 0,
    RATIO_4_3 = 1,
    RATIO_5_4 = 2,
    RATIO_16_9 = 3
};

typedef struct detailed_timing {
    uint16_t clock; // in 10 kHz
    uint16_t h_active; // in pixels
    uint16_t h_blank;
    uint16_t v_active; // in lines
    uint16_t v_blank;
    uint16_t h_sync_offset; // in pixels
    uint16_t h_sync_pulse;
    uint8_t v_sync_offset; // in lines
    uint8_t v_sync_pulse;
    uint16_t h_size; // in mm
    uint16_t v_size;
    uint8_t h_border; // in pixels, each side
    uint8_t v_border; // in lines, each side
    uint8_t features;
} detailed_timing_s, *detailed_timing_p;

static void edid_vendor_id(uint8_t *buf, const char *id)
{
    buf[0] = ((id[0] & 0x1F) << 2) | ((id[1] & 0x1F) >> 3);
    buf[1] = ((id[1] & 0x1F) << 5) | (id[2] & 0x1F);
}

static void edid_header(uint8_t *buf, uint16_t year, uint8_t week)
{
    // Magic header for EDID
    static const uint8_t block_zero_header[8] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0x00};
    static const uint8_t product_code[2] = {0x03, 0x00};
    // Zero serial number means that there is a string descriptor with it
    static const uint8_t serial_number[4] = {0x00, 0x00, 0x00, 0x00};

    memcpy(buf, block_zero_header, sizeof(block_zero_header));
    edid_vendor_id(buf + 8, "OVR");
    memcpy(buf + 10, product_code, sizeof(product_code));
    memcpy(buf + 12, serial_number, sizeof(serial_number));
    buf[16] = week; // Week
    buf[17] = year - 1990;
    buf[18] = 1; // EDID Version
    buf[19] = 3; // EDID Revision
}

static void edid_basic_parameters(uint8_t *buf)
{
    buf[20] = 1 << 7; // Digital display
    buf[21] = 0; // Width and height are 0, call it a projector
    buf[22] = 0;
    buf[23] = 220 - 100; // Gamma (2.2*100)
    buf[24] = 0xE2; // Support DPMS, RGB 4:4:4, preferred timing is DTD
}

static void edid_color_coordinates(uint8_t *buf, const float *red, const float *green, const float *blue, const float *white)
{
    // EDID uses a weird fractional 10 bit storage for chromacity coordinates
    uint16_t r_x = lroundf(red[0] * 1024.0f);
    uint16_t r_y = lroundf(red[1] * 1024.0f);
    uint16_t g_x = lroundf(green[0] * 1024.0f);
    uint16_t g_y = lroundf(green[1] * 1024.0f);
    uint16_t b_x = lroundf(blue[0] * 1024.0f);
    uint16_t b_y = lroundf(blue[1] * 1024.0f);
    uint16_t w_x = lroundf(white[0] * 1024.0f);
    uint16_t w_y = lroundf(white[1] * 1024.0f);
    
    uint8_t color_coordinates[10];
    color_coordinates[0] = ((r_x & 0x3) << 6) | ((r_y & 0x3) << 4) | \
                           ((g_x & 0x3) << 2) | (g_y & 0x3);
    color_coordinates[1] = ((b_x & 0x3) << 6) | ((b_y & 0x3) << 4) | \
                           ((w_x & 0x3) << 2) | (w_y & 0x3);
    color_coordinates[2] = r_x >> 2;
    color_coordinates[3] = r_y >> 2;
    color_coordinates[4] = g_x >> 2;
    color_coordinates[5] = g_y >> 2;
    color_coordinates[6] = b_x >> 2;
    color_coordinates[7] = b_y >> 2;
    color_coordinates[8] = w_x >> 2;
    color_coordinates[9] = w_y >> 2;

    memcpy(buf+25, color_coordinates, sizeof(color_coordinates));
}

static void edid_timing_bitmap(uint8_t *buf)
{
    buf[35] = 0; //(1 << 5) | 1; // support 640x480 and 800x600 if nothing else
    buf[36] = 0;
    buf[37] = 0; // TODO: manufacturer specific modes?
}

// We don't use any standard modes as long as we are portait mode
/*
static void edid_standard_mode(uint8_t *buf, uint16_t x, uint8_t ratio, uint8_t freq)
{
    buf[0] = x/8-31;
    buf[1] = (ratio << 6) | (freq - 60);
}
*/

static void edid_null_mode(uint8_t *buf)
{
    buf[0] = 0x01;
    buf[1] = 0x01;
}

static void edid_standard_modes(uint8_t *buf)
{
    // We don't use any standard modes as long as we are portait mode
    edid_null_mode(buf + 38);
    edid_null_mode(buf + 40);
    edid_null_mode(buf + 42);
    edid_null_mode(buf + 44);
    edid_null_mode(buf + 46);
    edid_null_mode(buf + 48);
    edid_null_mode(buf + 50);
    edid_null_mode(buf + 52);
}

static void edid_detailed_timing_descriptor(uint8_t *buf, detailed_timing_p t)
{
    buf[0] = t->clock;
    buf[1] = t->clock >> 8;
    buf[2] = t->h_active;
    buf[3] = t->h_blank;
    buf[4] = ((t->h_active & 0x0F00) >> 4) | ((t->h_blank & 0x0F00) >> 8);
    buf[5] = t->v_active;
    buf[6] = t->v_blank;
    buf[7] = ((t->v_active & 0x0F00) >> 4) | ((t->v_blank & 0x0F00) >> 8);
    buf[8] = t->h_sync_offset;
    buf[9] = t->h_sync_pulse;
    buf[10] = (t->v_sync_offset << 4) | (t->v_sync_pulse & 0x0F);
    buf[11] = ((t->h_sync_offset & 0x0300) >> 2) | ((t->h_sync_pulse & 0x0300) >> 4) |
        ((t->v_sync_offset & 0x30) >> 2) | ((t->v_sync_pulse & 0x30) >> 4);
    buf[12] = t->h_size;
    buf[13] = t->v_size;
    buf[14] = ((t->h_size & 0x0F00) >> 4) | ((t->v_size & 0x0F00) >> 8);
    buf[15] = t->h_border;
    buf[16] = t->v_border;
    buf[17] = t->features;
}

enum {
    STRING_SERIAL = 0xFF,
    STRING_TEXT = 0xFE,
    STRING_NAME = 0xFC
};

static void edid_string_descriptor(uint8_t *buf, uint8_t string_type, const char *string)
{
    buf[0] = 0;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = string_type;
    buf[4] = 0;

    int len = strlen(string);
    if (len > 13) len = 13;
    int i = 0;
    for (i = 0; i < len; i++)
        buf[5+i] = string[i];

    // Text needs to end in a line feed and then get padded with spaces
    if (i < 13) {
        buf[5+i] = 0x0A; // LF
        i++;
        for (; i < 13; i++)
            buf[5+i] = 0x20; // SP
    }
}

static void edid_range_limits(uint8_t *buf, uint8_t v_rate_min, uint8_t v_rate_max,
                                  uint8_t h_rate_min, uint8_t h_rate_max, uint8_t pixel_rate_max)
{
    buf[0] = 0;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0xFD;
    buf[4] = 0;
    buf[5] = v_rate_min;
    buf[6] = v_rate_max;
    buf[7] = h_rate_min;
    buf[8] = h_rate_max;
    buf[9] = pixel_rate_max;
    buf[10] = 00;
    // Since we don't have extended GTF, pad with 0A 20...
    buf[11] = 0x0A;
    int i;
    for (i = 0; i < 6; i++)
        buf[12+i] = 0x20;
}

static uint8_t generate_checksum(uint8_t *buf)
{
    uint8_t sum = 0;
    int i;
    for (i = 0; i < 127; i++)
        sum += buf[i];

    return 256 - sum;
}

void edid_generate(uint8_t *buf, uint16_t year, uint8_t week, const char *serial)
{
    // These blocks all have fixed positions in the buffer
    edid_header(buf, year, week);
    edid_basic_parameters(buf);
    // CIE1931 base coordinates from Soul
    static const float r[] = {0.665f, 0.334f};
    static const float g[] = {0.250f, 0.711f};
    static const float b[] = {0.139f, 0.050f};
    // TODO: The white coordinate is from sRGB, what is the real one?
    static const float w[] = {0.3127f, 0.3290f};
    edid_color_coordinates(buf, r, g, b, w);
    edid_timing_bitmap(buf);
    edid_standard_modes(buf);

    panel_timing_t panel;
    uint8_t h, v;
    panel_get_timing(0, &panel);
    panel_get_size(&h, &v);

    detailed_timing_s t;
    // The pixel clock in 10 KHz units
    t.clock = panel.h_total * panel.v_total * panel.refresh / 10000;
    t.h_active = panel.h_active;
    t.h_blank = panel.h_front + panel.h_pulse + panel.h_back;
    t.v_active = panel.v_active;
    t.v_blank = panel.v_front + panel.v_pulse + panel.v_back;
    t.h_sync_offset = panel.h_front;
    t.h_sync_pulse = panel.h_pulse;
    t.v_sync_offset = panel.v_front;
    t.v_sync_pulse = panel.v_pulse;
    // TODO: Can we fix portait by swapping these
    t.h_size = h;
    t.v_size = v;
    t.h_border = 0;
    t.v_border = 0;
    t.features = (1 << 4) | (1 << 3) | (1 << 1); // +hsync -vsync

    // The default detailed timing block
    edid_detailed_timing_descriptor(buf + 54, &t);
    // Monitor name block is required
    edid_string_descriptor(buf + 72, STRING_NAME, "Rift DK2");
    edid_string_descriptor(buf + 90, STRING_SERIAL, serial);
    // Monitor timings, this block is required
    edid_range_limits(buf + 108, 56, 77, 30, 150, 170 / 10);

    buf[126] = 1; // One CEA block

    buf[127] = generate_checksum(buf);
}

static void cea_header(uint8_t *buf, uint8_t dtd_loc, uint8_t dtd_num)
{
    buf[0] = 0x02; // CEA
    buf[1] = 0x03; // Rev 3
    buf[2] = dtd_loc; // Location of detailed timing descriptors
    buf[3] = dtd_num; // Number of detailed timing descriptors, also additional
                      // features which we do not support
}

/*
static void cea_hdmi(uint8_t *buf)
{
    buf[0] = (3 << 5) | 10; // HDMI, 10 bytes
    buf[1] = 0x03; // IEEE ID for HDMI
    buf[2] = 0x0C;
    buf[3] = 0x00;
    buf[4] = 0x10; // CEC 1.0.0.0
    buf[5] = 0x00;
    buf[6] = (1 << 6) | (1 << 5) | (1 << 4) | (1 << 3); // depths
    buf[7] = 1150/5; // 1150 MHz / 5
    buf[8] = 1 << 7; // latency specified
    buf[9] = 10/2; // 10 ms TODO: whats the real latency
    buf[10] = 0/2; // 0 ms since no audio
}

static void cea_standard_timings(uint8_t *buf)
{
    buf[0] = (2 << 5) | 4; // Video, 4 bytes
    buf[1] = (1 << 7) | 4; // 720p60, call it native
    buf[2] = 16; // 1080p60
    buf[3] = 3; // 480p 16:9
    buf[4] = 1; // 480p 4:3
}
*/

void edid_generate_cea(uint8_t *buf)
{
    uint8_t num_timings = panel_num_timings();

    // If we have more than just the default one, add them
    if (num_timings > 1) {
        // A maximum of 6 more timings beyond the default one are supported
        if (num_timings > 7)
            num_timings = 7;

        cea_header(buf, 4, num_timings - 1);

        detailed_timing_s t;
        panel_timing_t panel;
        uint8_t h, v;
        panel_get_size(&h, &v);

        // iterate through the list and add each supported timing
        for (uint8_t i = 1; i < num_timings; i++) {
            panel_get_timing(i, &panel);

            t.clock = panel.h_total * panel.v_total * panel.refresh / 10000;
            t.h_active = panel.h_active;
            t.h_blank = panel.h_front + panel.h_pulse + panel.h_back;
            t.v_active = panel.v_active;
            t.v_blank = panel.v_front + panel.v_pulse + panel.v_back;
            t.h_sync_offset = panel.h_front;
            t.h_sync_pulse = panel.h_pulse;
            t.v_sync_offset = panel.v_front;
            t.v_sync_pulse = panel.v_pulse;
            // TODO: Can we fix portait by swapping these
            t.h_size = h;
            t.v_size = v;
            t.h_border = 0;
            t.v_border = 0;
            t.features = (1 << 4) | (1 << 3) | (1 << 1); // +hsync -vsync

            edid_detailed_timing_descriptor(buf + 4 + (i - 1) * 18, &t);
        }

        buf[127] = generate_checksum(buf);
    }
}
