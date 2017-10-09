/************************************************************************************

Filename    :   h2c_register_map.h
Content     :   Register map for the Toshiba H2C+
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright (c) 2013-present, Facebook, Inc.
                All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree. An additional grant 
of patent rights can be found in the PATENTS file in the same directory.

*************************************************************************************/

#ifndef _H2C_REGISTER_MAP_H_
#define _H2C_REGISTER_MAP_H_

// Global Configuration Registers
#define H2C_CHIPID          0x0000
#define H2C_SYSCTL          0x0002
#define H2C_CONFCTL         0x0004
#define H2C_FIFOCTL         0x0006
#define H2C_AWCNT           0x0008
#define H2C_VWCNT           0x000A
#define H2C_PACKETID1       0x000C
#define H2C_PACKETID2       0x000E
#define H2C_PACKETID3       0x0010
#define H2C_FCCTL           0x0012
#define H2C_AUDFTPREM       0x001C
#define H2C_SLMBCONFIG      0x001E
#define H2C_PLLCTL0         0x0020
#define H2C_PLLCTL1         0x0022
#define H2C_PLL11CTL0       0x0024
#define H2C_PLL11CTL1       0x0026
#define H2C_WAIT_VSYNC      0x0060
#define H2C_IOCTL0          0x0080
#define H2C_IOCTL1          0x0082

// Interrupt Registers
#define H2C_INTSTATUS       0x0014
#define H2C_INTMASK         0x0016
#define H2C_INTFLAG         0x0018
#define H2C_INTSYSSTATUS    0x001A

// IR Registers

// CSI2/DSI TX PHY Registers
#define H2C_CLW_CNTRL       0x0140
#define H2C_D0W_CNTRL       0x0144
#define H2C_D1W_CNTRL       0x0148
#define H2C_D2W_CNTRL       0x014C
#define H2C_D3W_CNTRL       0x0150

// CSI2/DSI TX PPI Registers
#define H2C_STARTCNTRL      0x0204
#define H2C_PPISTATUS       0x0208
#define H2C_LINEINITCNT     0x0210
#define H2C_LPTXTIMECNT     0x0214
#define H2C_TCLK_HEADERCNT  0x0218
#define H2C_TCLK_TRAILCNT   0x021C
#define H2C_THS_HEADERCNT   0x0220
#define H2C_TWAKEUP         0x0224
#define H2C_TCLK_POSTCNT    0x0228
#define H2C_THS_TRAILCNT    0x022C
#define H2C_HSTXVREGCNT     0x0230
#define H2C_HSTXVREGEN      0x0234
#define H2C_TXOPTIONCNTRL   0x0238

// CSI2/DSI TX CTRL Registers
#define H2C_CSI_DSI_CONTROL 0x040C
#define H2C_CSI_DSI_STATUS  0x0410
#define H2C_DSICMD_RDFIFO   0x0430
#define H2C_DSI_ACKERR      0x0434
#define H2C_DSI_RXERR       0x0440
#define H2C_DSI_RXERR_HALT  0x0448
#define H2C_CSI_DSI_ERR     0x044C
#define H2C_CSI_DSI_ERR_HALT  0x0454
#define H2C_CSI_DSI_CONFW   0x0500
#define H2C_CS_DSI_LPCMD    0x0502
#define H2C_DSI_RESET       0x0504
#define H2C_CSI_DSI_START   0x0518

// CEC Registers

// DSI CTRL Registers
#define H2C_DSICTL          0x0700
#define H2C_DSI_VSW         0x0702
#define H2C_DSI_VBPR        0x0704
#define H2C_DSI_VACT        0x0706
#define H2C_DSI_HSW         0x0708
#define H2C_DSI_HBPR        0x070A
#define H2C_DCSCMD_TYPE     0x070C
#define H2C_DCSCMD_WC       0x070E
#define H2C_DCSCMD_WDSTART  0x0710

// VIP Registers
#define H2C_VBEMS_COM_TEST  0x4000

#define H2C_CS_YHVSIN       0x5044
#define H2C_CS_CHVSIN       0x5048
#define H2C_CS_HSZOUT       0x504C
#define H2C_CS_YHFILMODE    0x5050
#define H2C_CS_YHFILPSMODE  0x5054
#define H2C_CS_YMHFILBASE   0x505C
#define H2C_CS_CHFILMODE    0x5070
#define H2C_CS_CHFILPSMODE  0x5074
#define H2C_CS_CMHFILBASE   0x507C
#define H2C_CS_YVFILMODE    0x5090
#define H2C_CS_YVFILPSMODE  0x5094
#define H2C_CS_YVFILBASE    0x509C
#define H2C_CS_CVFILMODE    0x50A0
#define H2C_CS_CVFILPSMODE  0x50A4
#define H2C_CS_CVFILBASE    0x50AC

#define H2C_VIP_CONTROL     0x6000
#define H2C_GO_LINES        0x6004
#define H2C_VD_DELAY        0x6008
#define H2C_VIP_VSW         0x600C
#define H2C_VIP_VBP         0x6010
#define H2C_VIP_VAL         0x6014
#define H2C_VIP_VFP         0x6018
#define H2C_VIP_HSW         0x601C
#define H2C_VIP_HBP         0x6020
#define H2C_VIP_HAP         0x6024
#define H2C_VIP_HFP         0x6028
#define H2C_VIP_VAS         0x602C
#define H2C_VIP_DEBUG       0x6500
#define H2C_VIP_FIFO_THRESHOLD  0x6504
#define H2C_VIP_INIT_DELAY  0x650C
#define H2C_VIP_FIFO_MAX    0x6514
#define H2C_VIP_FIFO_MIN    0x6518
#define H2C_VIP_FIFO_PIXEL  0x6524
#define H2C_PP_FIFO_THRESHOLD   0x6508
#define H2C_PP_FIFO_REACH   0x6510
#define H2C_PP_FIFO_MAX     0x651C
#define H2C_PP_FIFO_MIN     0x6520
#define H2C_PP_FIFO_PIXEL   0x6528

// Internal Colorbar & Debug Registers
#define H2C_DEBCTL          0x7080

// HDMI RX Interrupts Registers
#define H2C_HDMI_INT0       0x8500
#define H2C_CLK_INT         0x8503
#define H2C_HDCP_INT        0x8508
#define H2C_MISC_INT        0x850B
#define H2C_KEY_INT         0x850F
#define H2C_SYS_INTM        0x8512
#define H2C_CLK_INTM        0x8513
#define H2C_PACKET_INTM     0x8514
#define H2C_HDCP_INTM       0x8518
#define H2C_MISC_INTM       0x851B
#define H2C_KEY_INTM        0x851F

// HDMI RX Status Registers
#define H2C_SYS_STATUS      0x8520
#define H2C_VI_STATUS       0x8521
#define H2C_VI_STATUS1      0x8522
#define H2C_VI_STATUS2      0x8525
#define H2C_CLK_STATUS      0x8526
#define H2C_PHYERR_STATUS   0x8527
#define H2C_VI_STATUS3      0x8528

// HDMI RX Control Registers
#define H2C_PHY_CTL0        0x8531
#define H2C_PHY_CTL1        0x8532
#define H2C_PHY_EN          0x8534
#define H2C_PHY_RST         0x8535
#define H2C_PHY_BIAS        0x8536
#define H2C_PHY_CSQ         0x853F
#define H2C_SYS_FREQ0       0x8540
#define H2C_SYS_FREQ1       0x8541
#define H2C_DDC_CTL         0x8543
#define H2C_HPD_CTL         0x8544
#define H2C_ANA_CTL         0x8545
#define H2C_AVM_CTL         0x8546
#define H2C_SOFT_RST        0x8547
#define H2C_INIT_END        0x854A
#define H2C_HDCP_MODE       0x8560
#define H2C_VI_MODE         0x8570
#define H2C_VOUT_SET        0x8573
#define H2C_VI_REP          0x8576
#define H2C_FH_MIN0         0x85AA
#define H2C_HV_RST          0x85AF

#define H2C_EDID_MODE       0x85C7
#define H2C_EDID_LEN        0x85CB

// HDMI RX Audio Control Registers
#define H2C_LKDET_FREQ      0x8630
#define H2C_NCO_MOD         0x8670

// HDMI RX Info Frame Data Registers

// HDMI RX HDCP Registers

#endif /* _H2C_REGISTER_MAP_H_ */
