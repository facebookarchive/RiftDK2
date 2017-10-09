/************************************************************************************
Filename    :   h2c.h
Content     :   Control of the Toshiba H2C+
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _H2C_H_
#define _H2C_H_

#include <stdbool.h>
#include <stdint.h>
#include "panel.h"

enum {
    MIPI_MODE_NONE = 0,
    MIPI_MODE_GENERIC = 1,
    MIPI_MODE_DCS = 2,
    MIPI_MODE_VIDEO = 3,
    MIPI_MODE_GENERIC_READ = 4,
    MIPI_MODE_DCS_READ = 5,
};

enum {
    MIPI_DATA_VSTART    = 0x01,
    MIPI_DATA_VEND      = 0x11,
    MIPI_DATA_HSTART    = 0x21,
    MIPI_DATA_HEND      = 0x31,
    MIPI_DATA_EOT       = 0x08,
    MIPI_DATA_CMOFF     = 0x02,
    MIPI_DATA_CMON      = 0x12,
    MIPI_DATA_SHUTDOWN  = 0x22,
    MIPI_DATA_TURNON    = 0x32,
    MIPI_DATA_GENSW0    = 0x03,
    MIPI_DATA_GENSW1    = 0x13,
    MIPI_DATA_GENSW2    = 0x23,
    MIPI_DATA_GENR0     = 0x04,
    MIPI_DATA_GENR1     = 0x14,
    MIPI_DATA_GENR2     = 0x24,
    MIPI_DATA_DCSSW0    = 0x05,
    MIPI_DATA_DCSSW1    = 0x15,
    MIPI_DATA_DCSR      = 0x06,
    MIPI_DATA_SMPR      = 0x37,
    MIPI_DATA_NULL      = 0x09,
    MIPI_DATA_BLANK     = 0x19,
    MIPI_DATA_GENLW     = 0x29,
    MIPI_DATA_DCSLW     = 0x39,
    MIPI_DATA_RGB565    = 0x0E,
    MIPI_DATA_RGB666    = 0x1E,
    MIPI_DATA_RGB666L   = 0x2E,
    MIPI_DATA_RGB888    = 0x3E,
};

void h2c_init(void);

void h2c_power_on(void);

void h2c_config(void);

void h2c_power_off(void);

bool h2c_version(uint32_t *version);

bool h2c_cable_status(void);

bool h2c_hpa_status(void);

bool h2c_tmds_clock_status(void);

bool h2c_tmds_pll_status(void);

void h2c_hdmi_reset(void);

void h2c_hdmi_phy_suspend(bool suspend);

bool h2c_hdmi_de_status(void);

bool h2c_hdmi_v_status(void);

void h2c_hdmi_v_clear(void);

uint16_t h2c_h_active(void);

uint16_t h2c_v_active(void);

void h2c_mipi_init(const panel_timing_p panel);

void h2c_mipi_reset(void);

void h2c_video_mode(void);

void h2c_mipi_set_tx(bool state);

bool h2c_mipi_read(uint8_t mode, uint8_t const *command, uint8_t command_len, uint8_t *buf, uint8_t *buf_len, bool blocking);

void h2c_mipi_flush_read(void);

bool h2c_mipi_complete_read(uint8_t mode, uint8_t command_len, uint8_t *buf, uint8_t *buf_len);

bool h2c_mipi_write(uint8_t mode, const uint8_t *buf, uint8_t len);

uint16_t h2c_mipi_ack_err(void);

uint16_t h2c_mipi_rx_err(void);

uint16_t h2c_mipi_dsi_err(void);

uint16_t h2c_mipi_dsi_status(void);

#endif /* _H2C_H_ */
