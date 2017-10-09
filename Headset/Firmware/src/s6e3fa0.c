/************************************************************************************

Filename    :   s6e3fa0.c
Content     :   Samsung S6E3FA0 display driver configuration
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "s6e3fa0.h"
#include "h2c.h"
#include "blocking.h"
#include "panel.h"
#include "ams568.h"

bool s6e3fa0_set_state(bool state, const panel_timing_p timing)
{
    if (state) {
        if (!ams568_init(timing)) {
            return 0;
        }
        
        // Turn on display
        static const uint8_t display_on[] = {0x29};
        if (!h2c_mipi_write(MIPI_MODE_DCS, display_on, sizeof(display_on))) {
            return 0;
        }
        
        return 1;
    } else {
        // Turn off display
        static const uint8_t display_off[] = {0x28};
        if (!h2c_mipi_write(MIPI_MODE_DCS, display_off, sizeof(display_off))) {
            return 0;
        }

        blocking_update(34000);

        // Start sleeping.  This is expected to fail if TMDS disappeared, 
        // and the panel is getting shut off anyway, so don't 
        // bother checking for failure
        static const uint8_t sleep_on[] = {0x10};
        h2c_mipi_write(MIPI_MODE_DCS, sleep_on, sizeof(sleep_on));
        return 1;
    }
}
