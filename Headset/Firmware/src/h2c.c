/************************************************************************************

Filename    :   h2c.c
Content     :   Control of the Toshiba H2C+
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#include "h2c.h"
#include "edid.h"
#include "blocking.h"
#include "panel.h"
#include "cpal_i2c.h"
#include "h2c_register_map.h"
#include <string.h>
#include <math.h>
#include "hw_config.h"
#include "gpio.h"
#include "uuid.h"
#include "timestamper.h"

typedef struct h2c_struct {
    CPAL_TransferTypeDef rx;
    CPAL_TransferTypeDef tx;
    uint8_t rx_buf[64];
    uint8_t tx_buf[64];
    uint8_t command_buf[64];
    uint8_t command_len;
    uint8_t command_type;
    uint8_t mipi_mode;
    uint8_t max_return;
} h2c_t, *h2c_p;

h2c_t g_h2c = {{0}};

#define MAX_CACHED_COMMAND 64
#define USE_MANUAL_HPDO 0
#define H2C_ADDR 0x1E
#define H2C_DEBUG 0
#define USE_PULSE 1
#define USE_VIP 0

// PLL Clock Speed
// float(refclk)*((fbd+1.0)/(prd+1.0))*(1.0/(2**frs))
#if USE_VIP
// 913 MHz
#define FBD 168
#define PRD 4
#else /* USE_VIP */
// 1.053 GHz
#define FBD 194
#define PRD 4
#endif /* USE_VIP */
static const float dsiclk = 27.0F * (float)(FBD + 1) / (float)(PRD + 1) / 2.0F;

static bool h2c_read(uint8_t addr, uint16_t reg, uint8_t *data, uint8_t len);
static bool h2c_wait_write(bool blocking);

static void h2c_config_i2c(void)
{
    // Set up the buffers
    g_h2c.rx.pbBuffer = g_h2c.rx_buf;
    g_h2c.tx.pbBuffer = g_h2c.tx_buf;

    // Initialize the I2C interface for the H2C
    // H2C uses 16 bit MSB first register addressing
    CPAL_I2C_DeInit(&I2C2_DevStructure);
    CPAL_I2C_StructInit(&I2C2_DevStructure);
    I2C2_DevStructure.CPAL_Dev = CPAL_I2C2;
    I2C2_DevStructure.CPAL_Mode = CPAL_MODE_MASTER;
    I2C2_DevStructure.CPAL_ProgModel = CPAL_PROGMODEL_INTERRUPT;
    I2C2_DevStructure.pCPAL_TransferRx = &g_h2c.rx;
    I2C2_DevStructure.pCPAL_TransferTx = &g_h2c.tx;
    I2C2_DevStructure.wCPAL_Options |= CPAL_OPT_16BIT_REG;
    // Run i2c at 400 KHz mostly to get the MIPI reads faster
    I2C2_DevStructure.pCPAL_I2C_Struct->I2C_ClockSpeed = 400000;
    CPAL_I2C_Init(&I2C2_DevStructure);
}

static bool h2c_cpal_wait_ready(void)
{
    while (I2C2_DevStructure.CPAL_State != CPAL_STATE_READY) {
        if (I2C2_DevStructure.CPAL_State == CPAL_STATE_ERROR) {
            // Reinitialize i2c if there was an error
            h2c_config_i2c();
            return 0;
        }
    }
    
    // Once we know the cpal interface is free, load our structs in
    I2C2_DevStructure.pCPAL_TransferRx = &g_h2c.rx;
    I2C2_DevStructure.pCPAL_TransferTx = &g_h2c.tx;
    
    return 1;
}

#if H2C_DEBUG
static bool h2c_verify_write(uint8_t addr, uint16_t reg, const uint8_t *data, uint8_t len)
{
    // Some registers are W/O, so don't verify writes on them
    if ((reg == H2C_CSI_DSI_CONFW) ||
        (reg == H2C_CSI_DSI_START) ||
        (reg == H2C_STARTCNTRL) ||
        (reg == H2C_DSICTL)) {
            return 1;
    }

    uint8_t ver[64] = {0x00};
    h2c_read(addr, reg, ver, len);

    if (memcmp(data, ver, len)) {
        printf("mismatch on addr 0x%04X\r\n", reg);
        for (uint8_t i = 0; i < len; i++) {
            printf("0x%02X ", data[i]);
        }
        printf("\r\n");
        for (uint8_t i = 0; i < len; i++) {
            printf("0x%02X ", ver[i]);
        }
        printf("\r\n");
        return 0;
    }

    return 1;
}
#endif /* H2C_DEBUG */

static bool h2c_write_byte(uint8_t addr, uint16_t reg, uint8_t data)
{
    if (!h2c_cpal_wait_ready()) {
        return 0;
    }

    g_h2c.tx.wNumData = 1;
    g_h2c.tx.wAddr1 = addr;
    g_h2c.tx.wAddr2 = reg;
    g_h2c.tx_buf[0] = data;
    g_h2c.tx.pbBuffer = g_h2c.tx_buf;

    if (CPAL_I2C_Write(&I2C2_DevStructure) != CPAL_PASS)
        return 0;

#if H2C_DEBUG
    if (addr == H2C_ADDR)
        h2c_verify_write(addr, reg, &data, 1);
#endif /* H2C_DEBUG */

    return 1;
}

static bool h2c_write(uint8_t addr, uint16_t reg, const uint8_t *data, uint8_t len)
{
    if (!h2c_cpal_wait_ready()) {
        return 0;
    }

    g_h2c.tx.wNumData = len;
    g_h2c.tx.wAddr1 = addr;
    g_h2c.tx.wAddr2 = reg;
    memcpy(g_h2c.tx_buf, data, len);
    g_h2c.tx.pbBuffer = g_h2c.tx_buf;

    if (CPAL_I2C_Write(&I2C2_DevStructure) != CPAL_PASS)
        return 0;

#if H2C_DEBUG
    if (addr == H2C_ADDR)
        h2c_verify_write(addr, reg, data, len);
#endif /* H2C_DEBUG */

    return 1;
}

static bool h2c_read(uint8_t addr, uint16_t reg, uint8_t *data, uint8_t len)
{
    if (!h2c_cpal_wait_ready()) {
        return 0;
    }

    g_h2c.rx.wNumData = len;
    g_h2c.rx.wAddr1 = addr;
    g_h2c.rx.wAddr2 = reg;
    g_h2c.rx.pbBuffer = g_h2c.rx_buf;

    if (CPAL_I2C_Read(&I2C2_DevStructure) != CPAL_PASS)
        return 0;

    if (!h2c_cpal_wait_ready()) {
        return 0;
    }

    memcpy(data, g_h2c.rx_buf, len);

    return 1;
}

#if H2C_DEBUG
static void h2c_mipi_debug(uint8_t id)
{
    uint8_t conf[] = {0x00, 0x00};
    h2c_read(H2C_ADDR, H2C_CSI_DSI_CONTROL, conf, sizeof(conf));
    uint16_t control = conf[0] | (conf[1] << 8);

    uint16_t status = h2c_mipi_dsi_status();
    uint16_t ack = h2c_mipi_ack_err();
    uint16_t rx = h2c_mipi_rx_err();
    uint16_t dsi = h2c_mipi_dsi_err();

    printf("%d H2C MIPI control 0x%X status 0x%X, ackerr 0x%X, rxerr 0x%X, dsierr 0x%X\r\n", id, control, status, ack, rx, dsi);
}

static void h2c_hdmi_debug(uint8_t id)
{
    uint8_t r = 0;
    h2c_read(H2C_ADDR, H2C_SYS_STATUS, &r, sizeof(r));
    printf(" %d H2C HDMI sys 0x%X ", id, r);
    h2c_read(H2C_ADDR, H2C_VI_STATUS, &r, sizeof(r));
    printf("vi0 0x%X ", r);
    h2c_read(H2C_ADDR, H2C_VI_STATUS1, &r, sizeof(r));
    printf("vi1 0x%X ", r);
    h2c_read(H2C_ADDR, H2C_VI_STATUS2, &r, sizeof(r));
    printf("vi2 0x%X ", r);
    h2c_read(H2C_ADDR, H2C_CLK_STATUS, &r, sizeof(r));
    printf("clk 0x%X ", r);
    h2c_read(H2C_ADDR, H2C_PHYERR_STATUS, &r, sizeof(r));
    printf("phy 0x%X ", r);
    h2c_read(H2C_ADDR, H2C_VI_STATUS3, &r, sizeof(r));
    printf("vi3 0x%X", r);
}
#endif /* H2C_DEBUG */

// Configuration writes for DSI require writing to a different register than
// we read from.  These write appear to be failing occasionally, so we need
// to verify that they succeed
static bool h2c_confw_verify(const uint8_t *command)
{
    // Find the target register
    uint16_t addr = 0;
    switch (command[3] & 0x1F) {
        case 0x03:
            addr = H2C_CSI_DSI_CONTROL;
            break;
        case 0x12:
            addr = H2C_DSI_RXERR_HALT;
            break;
        case 0x15:
            addr = H2C_CSI_DSI_ERR_HALT;
            break;
        default:
            return 0;
    }

    // Check if this is a set or clear on the bits
    bool setting = (command[3] & 0xA0) == 0xA0;

    // Read the target register
    uint8_t confw_ver[2] = {0x00};
    h2c_read(H2C_ADDR, addr, confw_ver, sizeof(confw_ver));
#if H2C_DEBUG
    printf("confw verify %s {0x%02X, 0x%02X}\r\n", setting ? "set" : "clear", confw_ver[0], confw_ver[1]);
#endif /* H2C_DEBUG */

    // Check if the bits were set or cleared successfully
    for (uint8_t i = 0; i < 3; i++) {
        if (setting) {
            if ((command[i] & confw_ver[i]) != command[i])
                return 0;
        } else {
            if (command[i] & confw_ver[i])
                return 0;
        }
    }

    return 1;
}

#if USE_VIP
static void h2c_vip_init(const panel_timing_p input, const panel_timing_p panel)
{
    // Enable VIP for upscaling and clear flags
    static const uint8_t vip_control[] = {0x78, 0x00, 0x0F, 0x00};
    h2c_write(H2C_ADDR, H2C_VIP_CONTROL, vip_control, sizeof(vip_control));
    
    // Bypass the deinterlacer, color converter
    static const uint8_t bypass[] = {0x00, 0x01, 0x10, 0x00};
    h2c_write(H2C_ADDR, H2C_VBEMS_COM_TEST, bypass, sizeof(bypass));

    // Set up the LCD Controller part of the VIP with the desired output timings
    // Set line timing from VBP + VSW - 2
    uint8_t go_lines[] = {panel->v_back + panel->v_pulse - 2, 0x00, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_GO_LINES, go_lines, sizeof(go_lines));
    // From the spreadsheet, clock cycles to delay VSYNC
    static const uint8_t vd_delay[] = {0x3C, 0x2E, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_VD_DELAY, vd_delay, sizeof(vd_delay));
    // Set LCD controller parameters
    uint8_t vsw[] = {panel->v_pulse - 1, ((panel->v_pulse - 1) >> 8) & 0x07, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_VIP_VSW, vsw, sizeof(vsw));
    uint8_t vbp[] = {panel->v_back - 1, ((panel->v_back - 1) >> 8) & 0x07, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_VIP_VBP, vbp, sizeof(vbp));
    uint8_t val[] = {panel->v_active - 1, ((panel->v_active - 1) >> 8) & 0x0F, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_VIP_VAL, val, sizeof(val));
    uint8_t vfp[] = {panel->v_front - 1, ((panel->v_front - 1) >> 8) & 0x07, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_VIP_VFP, vfp, sizeof(vfp));
    
    // Formulas from the spreadsheet
    float pclk = panel->h_total * panel->v_total * panel->refresh / 1000000.0F;
    uint16_t hsw = lroundf((float)panel->h_pulse * dsiclk / pclk) - 1;
    uint16_t hbp = lroundf((float)panel->h_back * dsiclk / pclk) - 1;
    uint16_t hfp = lroundf((float)panel->h_front * dsiclk / pclk + (float)panel->h_active * dsiclk / pclk) - panel->h_active - 1;
    uint8_t viphsw[] = {hsw, (hsw >> 8) & 0x0F, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_VIP_HSW, viphsw, sizeof(viphsw));
    uint8_t viphbp[] = {hbp, (hbp >> 8) & 0x0F, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_VIP_HBP, viphbp, sizeof(viphbp));
    uint8_t hap[] = {panel->h_active - 1, ((panel->h_active - 1) >> 8) & 0x0F, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_VIP_HAP, hap, sizeof(hap));
    uint8_t viphfp[] = {hfp, (hfp >> 8) & 0x0F, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_VIP_HFP, viphfp, sizeof(viphfp));
    
    // Set up the coefficients for the scaler
    uint8_t yhvsin[] = {input->v_active, (input->v_active >> 8) & 0x3F, input->h_active, (input->h_active >> 8) & 0x1F};
    h2c_write(H2C_ADDR, H2C_CS_YHVSIN, yhvsin, sizeof(yhvsin));
    h2c_write(H2C_ADDR, H2C_CS_CHVSIN, yhvsin, sizeof(yhvsin));
    uint8_t hszout[] = {panel->v_active, (panel->v_active >> 8) & 0x0F, panel->h_active, (panel->h_active >> 8) & 0x07};
    h2c_write(H2C_ADDR, H2C_CS_HSZOUT, hszout, sizeof(hszout));
    
    uint8_t yhfilmode[] = {0x01, 0x00, panel->h_active, (panel->h_active >> 8) & 0x07};
    h2c_write(H2C_ADDR, H2C_CS_YHFILMODE, yhfilmode, sizeof(yhfilmode));
    uint8_t yhfilpsmode[] = {input->h_active, (input->h_active >> 8) & 0x1F, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_CS_YHFILPSMODE, yhfilpsmode, sizeof(yhfilpsmode));
    // I have no idea what this is, but we need to zero the three LSB
    uint32_t filbase = (((input->h_active - 1) << 16) / (panel->h_active - 1)) & 0x003FFFF8;
    uint8_t yhfilbase[] = {filbase, filbase >> 8, filbase >> 16, 0x00};
    h2c_write(H2C_ADDR, H2C_CS_YMHFILBASE, yhfilbase, sizeof(yhfilbase));
    // The C registers get the same settings as the Y ones
    h2c_write(H2C_ADDR, H2C_CS_CHFILMODE, yhfilmode, sizeof(yhfilmode));
    h2c_write(H2C_ADDR, H2C_CS_CHFILPSMODE, yhfilpsmode, sizeof(yhfilpsmode));
    h2c_write(H2C_ADDR, H2C_CS_CMHFILBASE, yhfilbase, sizeof(yhfilbase));
    
    uint8_t yvfilmode[] = {0x01, 0x00, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_CS_YVFILMODE, yvfilmode, sizeof(yvfilmode));
    uint8_t yvfilpsmode[] = {input->v_active, (input->v_active >> 8) & 0x3F, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_CS_YVFILPSMODE, yvfilpsmode, sizeof(yvfilpsmode));
    // I have no idea what this is, but we need to zero the three LSB
    uint32_t vfilbase = (((input->v_active - 1) << 16) / (panel->v_active - 1)) & 0x003FFFF8;
    uint8_t yvfilbase[] = {vfilbase, vfilbase >> 8, vfilbase >> 16, 0x00};
    h2c_write(H2C_ADDR, H2C_CS_YVFILBASE, yvfilbase, sizeof(yvfilbase));
    // The C registers get the same settings as the Y ones
    h2c_write(H2C_ADDR, H2C_CS_CVFILMODE, yvfilmode, sizeof(yvfilmode));
    h2c_write(H2C_ADDR, H2C_CS_CVFILPSMODE, yvfilpsmode, sizeof(yvfilpsmode));
    h2c_write(H2C_ADDR, H2C_CS_CVFILBASE, yvfilbase, sizeof(yvfilbase));
}
#endif /* USE_VIP */

void h2c_init(void)
{
    // Bridge IRQ
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    gpio_config(BRIDGE_IRQ_PORT, BRIDGE_IRQ_PIN, GPIO_IN);

    SYSCFG_EXTILineConfig(BRIDGE_IRQ_SOURCE_PORT, BRIDGE_IRQ_SOURCE);

    EXTI_InitStructure.EXTI_Line = BRIDGE_IRQ_LINE;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = BRIDGE_IRQ_CHANNEL;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_Init(&NVIC_InitStructure);

    // Bridge standby (and clock enable)
    gpio_config(BRIDGE_STBY_PORT, BRIDGE_STBY_PIN, GPIO_PP);
    GPIO_WriteBit(BRIDGE_STBY_PORT, BRIDGE_STBY_PIN, Bit_RESET);

    // Bridge reset input
    gpio_config(BRIDGE_RESET_PORT, BRIDGE_RESET_PIN, GPIO_PP);
    GPIO_WriteBit(BRIDGE_RESET_PORT, BRIDGE_RESET_PIN, Bit_RESET);

    // Bridge power switch enable
    gpio_config(BRIDGE_PWR_EN_PORT, BRIDGE_PWR_EN_PIN, GPIO_PP);
    GPIO_WriteBit(BRIDGE_PWR_EN_PORT, BRIDGE_PWR_EN_PIN, Bit_RESET);

    // Bridge 1.2V power supply enable
    gpio_config(BRIDGE_1_2V_EN_PORT, BRIDGE_1_2V_EN_PIN, GPIO_PP);
    GPIO_WriteBit(BRIDGE_1_2V_EN_PORT, BRIDGE_1_2V_EN_PIN, Bit_RESET);
    
    // Pull-down on the DDC5V to prevent it from being floating when the
    // HDMI cable is unplugged
    gpio_config(HDMI_5V_PORT, HDMI_5V_PIN, GPIO_IN_PD);
}

void h2c_power_on(void)
{
    // Power sequencing
    // FIXME: On REV <= 3.3 switching BRIDGE_PWR drops the 3.0V rail enough
    // to reset the MCU.  This is a hack that backpowers the 3.0V rail through
    // the H2C part for 20 ms, which reduces the inrush when we do switch
    // 3.0V enough to not brown out.  This will be fixed in REV 3.4 #199
    GPIO_WriteBit(BRIDGE_STBY_PORT, BRIDGE_STBY_PIN, Bit_SET);
    blocking_update(20000);
    GPIO_WriteBit(BRIDGE_PWR_EN_PORT, BRIDGE_PWR_EN_PIN, Bit_SET);
    GPIO_WriteBit(BRIDGE_1_2V_EN_PORT, BRIDGE_1_2V_EN_PIN, Bit_SET);
    // Wait 10 ms after VDDC, VDDIO1, and VDDIO2 are there before coming
    // out of reset
    blocking_update(10000);

    GPIO_WriteBit(BRIDGE_RESET_PORT, BRIDGE_RESET_PIN, Bit_SET);
}

static void h2c_mipi_config(void)
{
#if USE_VIP
    // Setup specific MIPI protocol timings
    static const uint8_t lineinitcnt[] = {0x00, 0x2D, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_LINEINITCNT, lineinitcnt, sizeof(lineinitcnt));
    static const uint8_t lptxtimecnt[] = {0x05, 0x00, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_LPTXTIMECNT, lptxtimecnt, sizeof(lptxtimecnt));
    static const uint8_t tclkheadercnt[] = {0x04, 0x20, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_TCLK_HEADERCNT, tclkheadercnt, sizeof(tclkheadercnt));
    static const uint8_t tclktrailcnt[] = {0x03, 0x00, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_TCLK_TRAILCNT, tclktrailcnt, sizeof(tclktrailcnt));
    static const uint8_t thsheadercnt[] = {0x06, 0x06, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_THS_HEADERCNT, thsheadercnt, sizeof(thsheadercnt));
    static const uint8_t twakeup[] = {0x00, 0x4A, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_TWAKEUP, twakeup, sizeof(twakeup));
#else /* USE_VIP */
    // Setup specific MIPI protocol timings
    static const uint8_t lineinitcnt[] = {0x00, 0x2C, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_LINEINITCNT, lineinitcnt, sizeof(lineinitcnt));
    static const uint8_t lptxtimecnt[] = {0x05, 0x00, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_LPTXTIMECNT, lptxtimecnt, sizeof(lptxtimecnt));
    static const uint8_t tclkheadercnt[] = {0x03, 0x1F, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_TCLK_HEADERCNT, tclkheadercnt, sizeof(tclkheadercnt));
    static const uint8_t tclktrailcnt[] = {0x02, 0x00, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_TCLK_TRAILCNT, tclktrailcnt, sizeof(tclktrailcnt));
    static const uint8_t thsheadercnt[] = {0x04, 0x04, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_THS_HEADERCNT, thsheadercnt, sizeof(thsheadercnt));
    static const uint8_t twakeup[] = {0x00, 0x48, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_TWAKEUP, twakeup, sizeof(twakeup));
#endif /* USE_VIP */

    static const uint8_t tclkpostcnt[] = {0x0A, 0x00, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_TCLK_POSTCNT, tclkpostcnt, sizeof(tclkpostcnt));
    static const uint8_t thstrailcnt[] = {0x04, 0x00, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_THS_TRAILCNT, thstrailcnt, sizeof(thstrailcnt));
    // Enable the voltage regulators for the lanes
    static const uint8_t hstxvregen[] = {0x1F, 0x00, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_HSTXVREGEN, hstxvregen, sizeof(hstxvregen));
    // Enable continuous clk mode
    static const uint8_t txoptioncntrl[] = {0x01, 0x00, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_TXOPTIONCNTRL, txoptioncntrl, sizeof(txoptioncntrl));

    // Start the MIPI PPI
    static const uint8_t startcntrl[] = {0x01, 0x00, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_STARTCNTRL, startcntrl, sizeof(startcntrl));

    // Start DSI Clock, which allows us to config the remaining registers
    static const uint8_t csidsistart[] = {0x01, 0x00, 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_CSI_DSI_START, csidsistart, sizeof(csidsistart));

    // Don't halt on errors
    uint8_t timeout = 10;
    static const uint8_t dont_halt[] = {0xFF, 0xFF, 0x00, 0xC0 | 0x15};
    do {
        h2c_write(H2C_ADDR, H2C_CSI_DSI_CONFW, dont_halt, sizeof(dont_halt));
    } while (!h2c_confw_verify(dont_halt) && --timeout);
}

void h2c_config(void)
{
     // Run I2C off of the PLL
//    RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);

    // Generate the EDID
    uint8_t edid[256] = {0};
    char uuid[UUID_LEN] = {0};
    uuid_get(uuid, UUID_LEN, 0);
    edid_generate(edid, 2014, 10, uuid);
    edid_generate_cea(edid + 128);

    // Set up CPAL
    h2c_config_i2c();

    // Disable audio? Reserved setting from Toshiba's initialization
    static const uint8_t confctl[] = {0x04, 0x00};
    h2c_write(H2C_ADDR, H2C_CONFCTL, confctl, sizeof(confctl));

    // System reset?  Another reserved setting from Toshiba
    static const uint8_t full_reset[] = {0x00, 0x7F};
    h2c_write(H2C_ADDR, H2C_SYSCTL, full_reset, sizeof(full_reset));
    blocking_update(10000);
    // Clear reset and unsleep
    static const uint8_t clear_reset[] = {0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_SYSCTL, clear_reset, sizeof(clear_reset));
    blocking_update(10000);
    // Clear the cached command, since it is gone
    g_h2c.command_len = 0;
    g_h2c.command_type = 0;
    
    // Start the PLLs
    // TODO: Only bump up the pll when going to high pixel clocks
    static const uint8_t pllctl0[] = {FBD & 0xFF, (FBD >> 8) | (PRD << 4)};
    h2c_write(H2C_ADDR, H2C_PLLCTL0, pllctl0, sizeof(pllctl0));
    // 500 MHz - 1 GHz, enable clocks
    static const uint8_t pllctl1[] = {0x13, 0x02};
    h2c_write(H2C_ADDR, H2C_PLLCTL1, pllctl1, sizeof(pllctl1));
    blocking_update(10000);

    // Global config
    panel_timing_t panel;
    panel_current_timing(&panel);
    
#if USE_VIP
    // Set the number of bytes per line
    uint16_t line_pixels = panel.h_active*3;
    uint8_t vwcnt[] = {line_pixels, line_pixels >> 8};
    h2c_write(H2C_ADDR, H2C_VWCNT, vwcnt, sizeof(vwcnt));
    
    // Set the FIFO to the middle of what the spreadsheet says is ok
    static const uint8_t fifoctl[] = {0x03, 0x00};
    h2c_write(H2C_ADDR, H2C_FIFOCTL, fifoctl, sizeof(fifoctl));
    
    // Undocumented command from the scalar spreadsheet
    static const uint8_t waitvsync[] = {0x01, 0x00};
    h2c_write(H2C_ADDR, H2C_WAIT_VSYNC, waitvsync, sizeof(waitvsync));
    
    // Use word count instead of HDMI Rx measurement
    static const uint8_t dbctl[] = {0x40, 0x00};
    h2c_write(H2C_ADDR, H2C_DEBCTL, dbctl, sizeof(dbctl));
    
    // Setup the PLL for the scalar
    static const uint8_t pll11ctl0[] = {0x24, 0x14};
    h2c_write(H2C_ADDR, H2C_PLL11CTL0, pll11ctl0, sizeof(pll11ctl0));
    
    static const uint8_t pll11ctl1[] = {0x08, 0x06};
    h2c_write(H2C_ADDR, H2C_PLL11CTL1, pll11ctl1, sizeof(pll11ctl1));
#else /* USE_VIP */
    // Set the FIFO to where there is not color separation at the end of the row
    static const uint8_t fifoctl[] = {0x0A, 0x00};
    h2c_write(H2C_ADDR, H2C_FIFOCTL, fifoctl, sizeof(fifoctl));
#endif /* USE_VIP */
    
    // Configure the MIPI PPI and Tx
    h2c_mipi_config();
    
    // Do the init here to match what Toshiba's procedure says.  re-init later
    // to get new timings if they changed
    h2c_mipi_init(&panel);
    
    // VIP block is disabled by default, so don't configure it for now

    // Interrupt config
    // Undocumented write to make sync status work properly (and interrupt?)
    h2c_write_byte(H2C_ADDR, 0x85AB, 0);
    // Just enable the HDMI RX Interrupt
    static const uint8_t intmask[] = {0xBF, 0x05};
    h2c_write(H2C_ADDR, H2C_INTMASK, intmask, sizeof(intmask));
    // Within HDMI Rx, enable interrupt on sync change
    h2c_write_byte(H2C_ADDR, H2C_MISC_INTM, 0x1D);

    // HDMI Rx Control
#if USE_MANUAL_HPDO
     // 27 MHz, but don't power PHY yet
    h2c_write_byte(H2C_ADDR, H2C_PHY_CTL0, 0x00);
#else
    // Set 27 MHz, auto phy power
    h2c_write_byte(H2C_ADDR, H2C_PHY_CTL0, 0x01);
#endif /* USE_MANUAL_HPDO */
    // Don't enable the PHY yet if we have manual control
    h2c_write_byte(H2C_ADDR, H2C_PHY_EN, 0x3E);
    // Set 27 MHz again
    h2c_write_byte(H2C_ADDR, H2C_SYS_FREQ0, 0x8C);
    h2c_write_byte(H2C_ADDR, H2C_SYS_FREQ1, 0x0A);
    
    // Phy Auto Reset at 1600 us
    h2c_write_byte(H2C_ADDR, H2C_PHY_CTL1, 0x80);
    // Phy set bias and squelch
    h2c_write_byte(H2C_ADDR, H2C_PHY_BIAS, 0x40);
    h2c_write_byte(H2C_ADDR, H2C_PHY_CSQ, 0x0A);

    // Set DDC5V detect to have 200 ms delay
    h2c_write_byte(H2C_ADDR, H2C_DDC_CTL, 0x33);
#if USE_MANUAL_HPDO
    // Manual control of HPD
    h2c_write_byte(H2C_ADDR, H2C_HPD_CTL, 0x00);
#else
    // Interlock HPD to DDC5V detect (delayed)
    h2c_write_byte(H2C_ADDR, H2C_HPD_CTL, 0x10);
#endif /* USE_MANUAL_HPDO */
    // Enable analog section for audio // TODO: Is this necessary?
    h2c_write_byte(H2C_ADDR, H2C_ANA_CTL, 0x31);
    // AVMute, not sure if we need this
    h2c_write_byte(H2C_ADDR, H2C_AVM_CTL, 0x2D);

    // Use internal EDID at 100 kHz
    h2c_write_byte(H2C_ADDR, H2C_EDID_MODE, 0x01);
    // Use one 256 byte block
    h2c_write_byte(H2C_ADDR, H2C_EDID_LEN, 0x01);

    // Write the EDID into EEPROM
    for (uint16_t i = 0; i < 256; i += 8) {
        h2c_write(H2C_ADDR, 0x8C00 + i, edid + i, 8);
    }

    // Configure HDCP to load keys
    h2c_write_byte(H2C_ADDR, 0x85D1, 0x01);
    // KSV Auto Clear Mode
    h2c_write_byte(H2C_ADDR, H2C_HDCP_MODE, 0x24);
    // EESS_Err auto-unAuth
    h2c_write_byte(H2C_ADDR, 0x8563, 0x11);
    // Data Island Error auto-unauth
    h2c_write_byte(H2C_ADDR, 0x8564, 0x0F);
    
    // Absorb Hsync jitter
    h2c_write_byte(H2C_ADDR, H2C_VI_MODE, 0xFE);
    // Use 444
    h2c_write_byte(H2C_ADDR, H2C_VOUT_SET, 0x00);
    h2c_write_byte(H2C_ADDR, H2C_VI_REP, 0x00);
    // Undocumented write, V/H timing follows input?
//    h2c_write_byte(H2C_ADDR, 0x8571, 0x02);

#if USE_MANUAL_HPDO
    // Manually power on the PHY
    h2c_write_byte(H2C_ADDR, H2C_PHY_EN, 0x3F);
    // Manually enable HPD
    h2c_write_byte(H2C_ADDR, H2C_HPD_CTL, 0x01);
#endif /* USE_MANUAL_HPDO */

#if USE_VIP
    panel_timing_t panel_native;
    panel_get_timing(0, &panel_native);
    h2c_vip_init(&panel, &panel_native);
#endif /* USE_VIP */
    
    g_h2c.mipi_mode = MIPI_MODE_NONE;
}

void h2c_power_off(void)
{
    GPIO_WriteBit(BRIDGE_RESET_PORT, BRIDGE_RESET_PIN, Bit_RESET);
    blocking_update(10000);

    GPIO_WriteBit(BRIDGE_STBY_PORT, BRIDGE_STBY_PIN, Bit_RESET);
    GPIO_WriteBit(BRIDGE_1_2V_EN_PORT, BRIDGE_1_2V_EN_PIN, Bit_RESET);
    GPIO_WriteBit(BRIDGE_PWR_EN_PORT, BRIDGE_PWR_EN_PIN, Bit_RESET);
}

bool h2c_version(uint32_t *version)
{
    uint8_t id[2] = {0};
    h2c_read(H2C_ADDR, H2C_CHIPID, id, sizeof(id));
    *version = id[0] | (id[1] << 8);

    return 1;
}

static uint8_t h2c_sys_status(void)
{
    // Read the HDMI Rx main status register
    uint8_t sys_status = 0;
    h2c_read(H2C_ADDR, H2C_SYS_STATUS, &sys_status, sizeof(sys_status));
    return sys_status;
}

bool h2c_cable_status(void)
{
    // Is DDV power present
    return (h2c_sys_status() & 0x01);
}

bool h2c_hpa_status(void)
{
    // Check what we're outputting for hot plug
    uint8_t hpd_ctl = 0;
    h2c_read(H2C_ADDR, H2C_HPD_CTL, &hpd_ctl, sizeof(hpd_ctl));
    return hpd_ctl & 0x01;
}

bool h2c_tmds_clock_status(void)
{
    return (h2c_sys_status() & 0x02);
}

bool h2c_tmds_pll_status(void)
{
    // Is the HDMI PHY PLL locked
    return (h2c_sys_status() & 0x04);
}

void h2c_hdmi_reset(void)
{
    // Reset HDMI
    static const uint8_t hdmi_reset[] = {0x00, 0x01};
    h2c_write(H2C_ADDR, H2C_SYSCTL, hdmi_reset, sizeof(hdmi_reset));
    blocking_update(10000);
    // Clear reset
    static const uint8_t clear_reset[] = {0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_SYSCTL, clear_reset, sizeof(clear_reset));
}

void h2c_hdmi_phy_suspend(bool suspend)
{
    if (suspend) {
        // Go to manual PHY control with 27 MHz xtal
        h2c_write_byte(H2C_ADDR, H2C_PHY_CTL0, 0x00);
        // Suspend the PHY
        h2c_write_byte(H2C_ADDR, H2C_PHY_EN, 0x3E);
    } else {
        // Go back to auto PHY control with 27 MHz xtal
        h2c_write_byte(H2C_ADDR, H2C_PHY_CTL0, 0x01);
    }
}

bool h2c_hdmi_de_status(void)
{
#if H2C_DEBUG
    h2c_hdmi_debug(0);
#endif /* H2C_DEBUG */

    // Is HDMI DE locked
    return (h2c_sys_status() & 0x08);
}

bool h2c_hdmi_v_status(void)
{
    return (h2c_sys_status() & 0x80);
}

void h2c_hdmi_v_clear(void)
{
    // Clear the V sync changed interrupt.  This needs to be cleared before
    // the full system interrupt can be cleared
    h2c_write_byte(H2C_ADDR, H2C_MISC_INT, 0x02);
    // Clear the general interrupt status
    static const uint8_t clearstatus[2] = {0xFF, 0xFF};
    h2c_write(H2C_ADDR, H2C_INTSTATUS, clearstatus, sizeof(clearstatus));
}

uint16_t h2c_h_active(void)
{
    uint8_t h_size[2] = {0};
    // Undocumented H active registers
    h2c_read(H2C_ADDR, 0x8582, h_size, sizeof(h_size));
    return h_size[0] | (h_size[1] << 8);
}

uint16_t h2c_v_active(void)
{
    uint8_t v_size[2] = {0};
    // Undocumented V active registers
    h2c_read(H2C_ADDR, 0x8588, v_size, sizeof(v_size));
    return v_size[0] | (v_size[1] << 8);
}

void h2c_free_run(bool state)
{
    // Output from SRAM?
    uint8_t debctl[] = {state, 0x00};
    h2c_write(H2C_ADDR, H2C_DEBCTL, debctl, sizeof(debctl));
}

void h2c_mipi_init(const panel_timing_p panel)
{
#if USE_PULSE
    // Set the back porch size when using pulse mode
    uint8_t dsi_vbp[] = {panel->v_back, (panel->v_back >> 8) & 0x03};
    h2c_write(H2C_ADDR, H2C_DSI_VBPR, dsi_vbp, sizeof(dsi_vbp));

    uint16_t v_blank = panel->v_pulse;
#else
    // If we use non-burst with sync events, back porch is included
    uint16_t v_blank = panel->v_pulse + panel->v_back;
#endif /* USE_PULSE */
    uint8_t dsi_vsync[] = {v_blank, v_blank >> 8};
    h2c_write(H2C_ADDR, H2C_DSI_VSW, dsi_vsync, sizeof(dsi_vsync));

    // Set the vertical active
    uint8_t dsi_active[] = {panel->v_active, panel->v_active >> 8};
    h2c_write(H2C_ADDR, H2C_DSI_VACT, dsi_active, sizeof(dsi_active));

#if USE_PULSE
    // Set the back porch size when using pulse mode
    uint8_t dsi_hbp[] = {panel->h_back, panel->h_back >> 8};
    h2c_write(H2C_ADDR, H2C_DSI_HBPR, dsi_hbp, sizeof(dsi_hbp));

    uint16_t h_blank = panel->h_pulse;
#else
    // If we use non-burst with sync events, back porch is included
    uint16_t h_blank = panel->h_pulse + panel->h_back;
#endif /* USE_PULSE */
    float pclk = panel->h_total * panel->v_total * panel->refresh / 1000000.0F;
    uint16_t hsw = lroundf((float)h_blank * dsiclk / pclk);
    // FIXME: The correct hsw causes corruption, implying timings are wrong 
    // elsewhere
//    uint8_t dsi_hsw[] = {hsw, hsw >> 8};
    uint8_t dsi_hsw[] = {h_blank, h_blank >> 8};
    h2c_write(H2C_ADDR, H2C_DSI_HSW, dsi_hsw, sizeof(dsi_hsw));
}

void h2c_mipi_reset(void)
{
    // Reset MIPI
    static const uint8_t mipi_reset[] = {0x00, 0x02};
    h2c_write(H2C_ADDR, H2C_SYSCTL, mipi_reset, sizeof(mipi_reset));
    blocking_update(10000);
    // Clear reset
    static const uint8_t clear_reset[] = {0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_SYSCTL, clear_reset, sizeof(clear_reset));
}

void h2c_video_mode(void)
{
    // 4 lane HS setup
    // Set the bits to go to HS
    uint8_t timeout = 10;
    static const uint8_t hs_video[] = {0x86, 0x00, 0x00, 0xA0 | 0x03};
    do {
        h2c_write(H2C_ADDR, H2C_CSI_DSI_CONFW, hs_video, sizeof(hs_video));
    } while (!h2c_confw_verify(hs_video) && --timeout);

    // Clear the CSI bit
    timeout = 10;
    static const uint8_t hs_video_dsi[] = {0x00, 0x80, 0x00, 0xC0 | 0x03};
    do {
        h2c_write(H2C_ADDR, H2C_CSI_DSI_CONFW, hs_video_dsi, sizeof(hs_video_dsi));
    } while (!h2c_confw_verify(hs_video_dsi) && --timeout);

    // Complete HDMI initialization
    h2c_write_byte(H2C_ADDR, H2C_INIT_END, 0x01);
    
    // Start TX, using sync events if we are not using pulses
    static const uint8_t start_tx[] = {0x01 | (!USE_PULSE << 1), 0x00};
    h2c_write(H2C_ADDR, H2C_DSICTL, start_tx, sizeof(start_tx));

    // Enable the actual video TX buffer
    static const uint8_t enable_tx_buf[] = {0x17, 0x0C};
    h2c_write(H2C_ADDR, H2C_CONFCTL, enable_tx_buf, sizeof(enable_tx_buf));

    g_h2c.mipi_mode = MIPI_MODE_VIDEO;
}

void h2c_mipi_set_tx(bool state)
{
    // Enable or disable transmission over MIPI
    uint8_t set_tx[] = {state ? (0x01 | (!USE_PULSE << 1)) : 0x00, 0x00};
    h2c_write(H2C_ADDR, H2C_DSICTL, set_tx, sizeof(set_tx));
}

static void h2c_generic_mode(void)
{
    // FIXME: chris.cheng@taec.toshiba.com says going back to LP doesn't work
//    // Clear the HS bits to go back to LP
//    static uint8_t lp_mode[] = {0x80, 0x00, 0x00, 0xC3};
//    do {
//        h2c_write(H2C_ADDR, H2C_CSI_DSI_CONFW, lp_mode, sizeof(lp_mode));
//    } while (!h2c_confw_verify(lp_mode));

    g_h2c.mipi_mode = MIPI_MODE_GENERIC;
}

static bool h2c_wait_write(bool blocking)
{
    // Wait for the write to complete before changing any registers
    uint32_t start = timestamper_get_time();
    // The write occurs during v blank, so wait a couple of frames
    while (!timestamper_is_after(start + 35000)) {
        uint8_t dc_start = 0;
        h2c_read(H2C_ADDR, H2C_DSICTL, &dc_start, 1);
        if (!(dc_start & 0x04)) {
            return 1;
        } else if (!blocking) {
            return 0;
        }
        blocking_update(100);
    }

    // If we timed out, the DSI block is likely hung, so reset it
    h2c_mipi_reset();
    h2c_mipi_config();
    panel_timing_t panel;
    panel_current_timing(&panel);
    h2c_mipi_init(&panel);
    h2c_video_mode();
        
    return 0;
}

static bool h2c_mipi_command(uint8_t data_type, uint8_t const *buf, uint8_t len, bool blocking)
{
    // Wait for the write to complete before changing any registers
    if (!h2c_wait_write(blocking))
        return 0;

    // If we're sending the same command repeatedly, we can save a ton of
    // time by just reusing the existing data in the command registers
    // and re-issuing a send command
    if ((data_type != g_h2c.command_type) || (len != g_h2c.command_len) || memcpy(g_h2c.command_buf, buf, len)) {
        uint8_t p_len;
        uint8_t pkt_type;
        // short or long packet type
        if (len > 2) {
            p_len = len;
            pkt_type = 0x40;
        } else {
            p_len = 0; // Use 0 parameter len for short writes
            pkt_type = 0x10;
        }

        // Set the types for the command
        uint8_t cmd_type[] = {data_type, pkt_type};
        h2c_write(H2C_ADDR, H2C_DCSCMD_TYPE, cmd_type, sizeof(cmd_type));

        // Set the length of the parameters
        uint8_t cmdlen[] = {p_len, p_len >> 8};
        h2c_write(H2C_ADDR, H2C_DCSCMD_WC, cmdlen, sizeof(cmdlen));

        uint16_t reg = H2C_DCSCMD_WDSTART;
        uint8_t i = 0;
        while (len > 1) {
            // Write the actual command
            h2c_write(H2C_ADDR, reg+i, buf+i, 2);
            len -= 2;
            i += 2;
        }

        // If we had an odd number, write the last byte padded
        if (len) {
            uint8_t last_byte[] = {buf[i], 0x00};
            h2c_write(H2C_ADDR, reg+i, last_byte, sizeof(last_byte));
        }
        
        // Cache the command so we can repeat it much more quickly
        if (len <= MAX_CACHED_COMMAND) {
            g_h2c.command_len = len;
            g_h2c.command_type = data_type;
            memcpy(g_h2c.command_buf, buf, len);
        }
    }

    // Actually perform the command
    static const uint8_t dsictl[] = {0x05, 0x00};
    h2c_write(H2C_ADDR, H2C_DSICTL, dsictl, sizeof(dsictl));

    return 1;
}

static uint8_t h2c_read_fifo_status(void)
{
    uint8_t fifostatus = 0;
    h2c_read(H2C_ADDR, H2C_CSI_DSI_STATUS, &fifostatus, 1);
    if (fifostatus & 0x80) {
        return 2;
    } else if (fifostatus & 0x20) {
        return 1;
    } else {
        return 0;
    }
}

static bool h2c_mipi_read_data_type(uint8_t *data_type, uint8_t mode, uint8_t command_len)
{
    // Get the actual DSI Packet Data ID for Processor -> Peripheral
    if (mode == MIPI_MODE_GENERIC_READ) {
        switch (command_len) {
            case 0:
                *data_type = MIPI_DATA_GENR0;
                break;
            case 1:
                *data_type = MIPI_DATA_GENR1;
                break;
            case 2:
                *data_type = MIPI_DATA_GENR2;
                break;
            default:
                return 0;
        }
    } else if (mode == MIPI_MODE_DCS_READ) {
        switch (command_len - 1) {
            case 0:
                *data_type = MIPI_DATA_DCSR;
                break;
            default:
                return 0;
        }
    } else {
        // Just handle reads
        return 0;
    }
    
    return 1;
}

static bool h2c_mipi_read_return_type(uint8_t *return_type, uint8_t mode, uint8_t read_len)
{
    // Get the actual DSI Packet Data ID for Peripheral -> Processor
    if (mode == MIPI_MODE_GENERIC_READ) {
        switch (read_len) {
            case 1:
                *return_type = 0x11;
                break;
            case 2:
                *return_type = 0x12;
                break;
            default:
                *return_type = 0x1A;
                break;
        }
    } else if (mode == MIPI_MODE_DCS_READ) {
        switch (read_len) {
            case 1:
                *return_type = 0x21;
                break;
            case 2:
                *return_type = 0x22;
                break;
            default:
                *return_type = 0x1C;
                break;
        }
    } else {
        // Just handle reads
        return 0;
    }
    
    return 1;
}

bool h2c_mipi_read(uint8_t mode, uint8_t const *command, uint8_t command_len, uint8_t *buf, uint8_t *buf_len, bool blocking)
{
    if (g_h2c.mipi_mode != MIPI_MODE_GENERIC)
        h2c_generic_mode();

    uint8_t data_type = 0;
    if (!h2c_mipi_read_data_type(&data_type, mode, command_len))
        return 0;

    // Only set the maximum return packet size if it's different from the last
    if (g_h2c.max_return != *buf_len) {
        // Wait for the write to complete before changing any registers
        if (!h2c_wait_write(blocking))
            return 0;
        g_h2c.max_return = *buf_len;
        static const uint8_t smpr[] = {MIPI_DATA_SMPR, 0x10};
        h2c_write(H2C_ADDR, H2C_DCSCMD_TYPE, smpr, sizeof(smpr));
        // Word count zero
        static const uint8_t cmdlen[] = {0x00, 0x00};
        h2c_write(H2C_ADDR, H2C_DCSCMD_WC, cmdlen, sizeof(cmdlen));
        // Set the size of the read
        uint8_t read_len[] = {g_h2c.max_return, 0};
        h2c_write(H2C_ADDR, H2C_DCSCMD_WDSTART, read_len, sizeof(read_len));
        // Perform the actual command
        static const uint8_t dsictl[] = {0x05, 0x00};
        h2c_write(H2C_ADDR, H2C_DSICTL, dsictl, sizeof(dsictl));
    }

    // On blocking calls where we absolutely need the right data at the right
    // time, make sure to flush the rx first
    if (blocking) {
        // Empty the receive FIFO
        while (h2c_read_fifo_status()) {
            uint8_t entry[4];
            h2c_read(H2C_ADDR, H2C_DSICMD_RDFIFO, entry, sizeof(entry));
        }
    }

    if (!h2c_mipi_command(data_type, command, command_len, blocking))
        return 0;

    if (!blocking) {
        return 1;
    }
    
    // TODO: wait for the interrupt asynchronously
    // Wait for the read to complete
    int16_t timeout = 200;
    while (timeout--) {
        if (h2c_read_fifo_status())
            break;

        // Also give up if there was an error
        uint8_t rxerr[3] = {0x00};
        h2c_read(H2C_ADDR, H2C_DSI_RXERR, rxerr, sizeof(rxerr));
        if (rxerr[2] & 0x18)
            return 0;
    }

    // Read the data from the FIFO
    return h2c_mipi_complete_read(mode, command_len, buf, buf_len);
}

void h2c_mipi_flush_read(void)
{
    static const uint8_t reset_fifo[] = {0x10};
    h2c_write(H2C_ADDR, H2C_DSI_RESET, reset_fifo, sizeof(reset_fifo));
    
    uint8_t entry[4] = {0x00};
    // Empty the read FIFO
    while (h2c_read_fifo_status()) {
        h2c_read(H2C_ADDR, H2C_DSICMD_RDFIFO, entry, sizeof(entry));
    }
}

bool h2c_mipi_complete_read(uint8_t mode, uint8_t command_len, uint8_t *buf, uint8_t *buf_len)
{
    // TODO: Distinguish pending reads from actual errors by return value?
    // Don't block on the complete
    if (!h2c_read_fifo_status())
        return 0;
    
    uint8_t data_type = 0;
    if (!h2c_mipi_read_return_type(&data_type, mode, *buf_len))
        return 0;
    
    // TODO: Check the ECC on the reads, #52
    // Short reads are a single FIFO entry
    if (*buf_len <= 2) {
        uint8_t entry[4] = {0x00};
        do {
            // Note that sometimes there is a false entry of zeroes ahead
            // of our real entry, so keep reading entries until we either
            // get the type we are looking for or the FIFO is empty
            h2c_read(H2C_ADDR, H2C_DSICMD_RDFIFO, entry, sizeof(entry));
        } while ((entry[0] != data_type) && h2c_read_fifo_status());

        // Fail if the data didn't match
        if (entry[0] != data_type)
            return 0;
        
        // Otherwise copy the actual payload bytes
        memcpy(buf, entry + 1, *buf_len);
    } else {
        // Long reads are multi FIFO entry, with the first being a header
        uint8_t remaining = *buf_len;
        bool read_header = 0;
        while (remaining) {
            // Read an entry from the read FIFO
            uint8_t entry[4] = {0x00};
            h2c_read(H2C_ADDR, H2C_DSICMD_RDFIFO, entry, sizeof(entry));

            if (!read_header) {
                if (entry[0] == data_type)
                    read_header = 1;
            } else {
                // The entries are padded, so only copy the valid bytes
                uint8_t valid_bytes = remaining > 4 ? 4 : remaining;
                memcpy(buf, entry, valid_bytes);
                remaining -= valid_bytes;
                buf += valid_bytes;
            }
            
            // If we still need more, check if there is another FIFO entry
            if (remaining) {
                if (!h2c_read_fifo_status()) {
                    // notify that the read buffer is short
                    *buf_len = *buf_len - remaining;
                    return 0;
                }
            }
        }
    }
    
    return 1;
}

bool h2c_mipi_write(uint8_t mode, const uint8_t *buf, uint8_t len)
{
    if (g_h2c.mipi_mode != MIPI_MODE_GENERIC)
        h2c_generic_mode();

    uint8_t data_type = 0;

    // Get the actual DSI Packet Data ID
    if (mode == MIPI_MODE_GENERIC) {
        switch (len) {
            case 0:
                data_type = MIPI_DATA_GENSW0;
                break;
            case 1:
                data_type = MIPI_DATA_GENSW1;
                break;
            case 2:
                data_type = MIPI_DATA_GENSW2;
                break;
            default:
                data_type = MIPI_DATA_GENLW;
                break;
        }
    } else if (mode == MIPI_MODE_DCS) {
        // Subtract the command byte from the parameters for DCS
        switch (len-1) {
            case 0:
                data_type = MIPI_DATA_DCSSW0;
                break;
            case 1:
                data_type = MIPI_DATA_DCSSW1;
                break;
            default:
                data_type = MIPI_DATA_DCSLW;
                break;
        }
    } else {
        // For now, just handle writes
        return 0;
    }

    if (h2c_mipi_command(data_type, buf, len, 1)) {
        return 1;
    }
        
    return 0;
}

uint16_t h2c_mipi_ack_err(void)
{
    uint8_t status[] = {0x00, 0x00};
    h2c_read(H2C_ADDR, H2C_DSI_ACKERR, status, sizeof(status));
    return status[0] | (status[1] << 8);
}

uint16_t h2c_mipi_rx_err(void)
{
    uint8_t status[] = {0x00, 0x00};
    h2c_read(H2C_ADDR, H2C_DSI_RXERR, status, sizeof(status));
    return status[0] | (status[1] << 8);
}

uint16_t h2c_mipi_dsi_err(void)
{
    uint8_t status[] = {0x00, 0x00};
    h2c_read(H2C_ADDR, H2C_CSI_DSI_ERR, status, sizeof(status));
    return status[0] | (status[1] << 8);
}

uint16_t h2c_mipi_dsi_status(void)
{
    uint8_t status[] = {0x00, 0x00};
    h2c_read(H2C_ADDR, H2C_CSI_DSI_STATUS, status, sizeof(status));
    return status[0] | (status[1] << 8);
}
