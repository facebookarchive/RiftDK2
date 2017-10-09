/**
  ******************************************************************************
  * @file    usb_desc.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Descriptors for Custom HID Demo
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_desc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USB Standard Device Descriptor */
const uint8_t CustomHID_DeviceDescriptor[CUSTOMHID_SIZ_DEVICE_DESC] =
  {
    0x12,                       /*bLength */
    USB_DEVICE_DESCRIPTOR_TYPE, /*bDescriptorType*/
    0x00,                       /*bcdUSB */
    0x02,
    0x00,                       /*bDeviceClass*/
    0x00,                       /*bDeviceSubClass*/
    0x00,                       /*bDeviceProtocol*/
    0x40,                       /*bMaxPacketSize 64*/
    0x33,                       /*idVendor (0x2833)*/
    0x28,
    0x21,                       /*idProduct = 0x0021*/
    0x00,
    0x12,                       /*bcdDevice minor version .12*/
#ifdef DK2_REV3
    0x01,                       /*Rev3 major version is 1*/
#else /* DK2_REV3 */
    0x02,                       /*Rev3.3 major version is 2*/
#endif /* DK2_REV3 */
    1,                          /*Index of string descriptor describing
                                              manufacturer */
    2,                          /*Index of string descriptor describing
                                             product*/
    3,                          /*Index of string descriptor describing the
                                             device serial number */
    0x01                        /*bNumConfigurations*/
  }
  ; /* CustomHID_DeviceDescriptor */


/* USB Configuration Descriptor */
/*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor */
const uint8_t CustomHID_ConfigDescriptor[CUSTOMHID_SIZ_CONFIG_DESC] =
  {
    0x09, /* bLength: Configuration Descriptor size */
    USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: Configuration */
    CUSTOMHID_SIZ_CONFIG_DESC,
    /* wTotalLength: Bytes returned */
    0x00,
    0x01,         /* bNumInterfaces: 1 interface */
    0x01,         /* bConfigurationValue: Configuration value */
    0x00,         /* iConfiguration: Index of string descriptor describing
                                 the configuration*/
    0xA0,         /* bmAttributes: Bus powered, supporting remote wakeup */
    0x32,         /* MaxPower 100 mA: The hub reports the remaining 400 mA */

    /************** Descriptor of Custom HID interface ****************/
    /* 09 */
    0x09,         /* bLength: Interface Descriptor size */
    USB_INTERFACE_DESCRIPTOR_TYPE,/* bDescriptorType: Interface descriptor type */
    0x00,         /* bInterfaceNumber: Number of Interface */
    0x00,         /* bAlternateSetting: Alternate setting */
    0x01,         /* bNumEndpoints */
    0x03,         /* bInterfaceClass: HID */
    0x00,         /* bInterfaceSubClass : 1=BOOT, 0=no boot */
    0x00,         /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
    0,            /* iInterface: Index of string descriptor */
    /******************** Descriptor of Custom HID HID ********************/
    /* 18 */
    0x09,         /* bLength: HID Descriptor size */
    HID_DESCRIPTOR_TYPE, /* bDescriptorType: HID */
    0x10,         /* bcdHID: HID Class Spec release number */
    0x01,
    0x00,         /* bCountryCode: Hardware target country */
    0x01,         /* bNumDescriptors: Number of HID class descriptors to follow */
    0x22,         /* bDescriptorType */
    (CUSTOMHID_SIZ_REPORT_DESC & 0xFF),/* wItemLength: Total length of Report descriptor */
    (CUSTOMHID_SIZ_REPORT_DESC >> 8),
    /******************** Descriptor of Custom HID endpoints ******************/
    /* 27 */
    0x07,          /* bLength: Endpoint Descriptor size */
    USB_ENDPOINT_DESCRIPTOR_TYPE, /* bDescriptorType: */

    0x81,          /* bEndpointAddress: Endpoint Address (IN) */
    0x03,          /* bmAttributes: Interrupt endpoint */
    0x40,          /* wMaxPacketSize: 64 Bytes max */
    0x00,
    0x01,          /* bInterval: Polling Interval (1 ms) */
    /* 34 */
  }
  ; /* CustomHID_ConfigDescriptor */
const uint8_t CustomHID_ReportDescriptor[CUSTOMHID_SIZ_REPORT_DESC] =
  {
    0x05, 0x03,                    // USAGE_PAGE (VR Controls)
    0x09, 0x05,                    // USAGE (Head Tracker)
    0xa1, 0x01,                    // COLLECTION (Application)

    // Sub-Collection for DK set of reports
    0x06, 0x00, 0xff,              //   USAGE_PAGE (Vendor Defined Page 1)
    0x09, 0x01,                    //   USAGE (Vendor Defined - DK compatibility)
    0xa1, 0x02,                    //   COLLECTION (Logical)
    0x05, 0x01,                    //   USAGE_PAGE (Generic Desktop)

    // IN Report for sensor data
    0x85, 0x01,                    //   REPORT_ID (1)
    // Sample Count
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    // Timestamp
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    // LastCommandID
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    // Temperature
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    // Gyro/Accel Sample 0
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x10,                    //   REPORT_COUNT (16)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    // Gyro/Accel Sample 1
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x10,                    //   REPORT_COUNT (16)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    // Gyro/Accel Sample 2
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x10,                    //   REPORT_COUNT (16)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    // Mag Sample
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x03,                    //   REPORT_COUNT (3)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)

    // Config Report
    0x85, 0x02,                    //   REPORT_ID (2)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x04,                    //   REPORT_COUNT (4)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)

    // Calibration Report
    0x85, 0x03,                    //   REPORT_ID (3)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x42,                    //   REPORT_COUNT (66)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)

    // Full Scale Range Report
    0x85, 0x04,                    //   REPORT_ID (4)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x05,                    //   REPORT_COUNT (5)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)

    // Register Report
    0x85, 0x05,                    //   REPORT_ID (5)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x03,                    //   REPORT_COUNT (3)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)

    // DFU Report
    0x85, 0x06,                    //   REPORT_ID (6)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)

    // GPIO Report
    0x85, 0x07,                    //   REPORT_ID (7)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x02,                    //   REPORT_COUNT (2)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)

    // Keep Alive Report
    0x85, 0x08,                    //   REPORT_ID (8)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x02,                    //   REPORT_COUNT (2)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)

    // Display Info Report
    0x85, 0x09,                    //   REPORT_ID (9)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x35,                    //   REPORT_COUNT (53)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)

    // Serial Report
    0x85, 0x0A,                    //   REPORT_ID (10)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x0C,                    //   REPORT_COUNT (12)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)

    0xc0,                          //   END_COLLECTION

    // Sub-Collection for DK2 set of reports
    0x06, 0x00, 0xff,              //   USAGE_PAGE (Vendor Defined Page 1)
    0x09, 0x02,                    //   USAGE (Vendor Defined - DK2 compatibility)
    0xa1, 0x02,                    //   COLLECTION (Logical)
    0x05, 0x01,                    //   USAGE_PAGE (Generic Desktop)
    // TODO: Add real usages for these
    // DK2 IN Report
    0x85, 0x0b,                    //   REPORT_ID (11)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x3d,                    //   REPORT_COUNT (61)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    
    // Tracking Report
    0x85, 0x0c,                    //   REPORT_ID (12)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x0a,                    //   REPORT_COUNT (10)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    
    // Display Report
    0x85, 0x0d,                    //   REPORT_ID (13)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x0b,                    //   REPORT_COUNT (11)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    
    // Magnetometer Calibration Report
    0x85, 0x0e,                    //   REPORT_ID (14)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x31,                    //   REPORT_COUNT (49)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    
    // Position Calibration Report
    0x85, 0x0f,                    //   REPORT_ID (15)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x1b,                    //   REPORT_COUNT (27)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    
    // Custom Pattern Report
    0x85, 0x10,                    //   REPORT_ID (16)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x09,                    //   REPORT_COUNT (9)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    
    // Keep Alive Mux Report
    0x85, 0x11,                    //   REPORT_ID (17)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x03,                    //   REPORT_COUNT (3)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    
    // Manufacturing Report
    0x85, 0x12,                    //   REPORT_ID (18)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x0D,                    //   REPORT_COUNT (13)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    
    // UUID Report
    0x85, 0x13,                    //   REPORT_ID (19)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x14,                    //   REPORT_COUNT (20)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)

    // Temperature Report
    0x85, 0x14,                    //   REPORT_ID (20)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x15,                    //   REPORT_COUNT (21)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)

    // GyroOffset Report
    0x85, 0x15,                    //   REPORT_ID (21)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x0F,                    //   REPORT_COUNT (15)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)

    // LensDistortion Report
    0x85, 0x16,                    //   REPORT_ID (22)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    0x09, 0x3b,                    //   USAGE (Byte Count)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x3d,                    //   REPORT_COUNT (61)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)
    
    0xc0,                          //   END_COLLECTION
    
    0xc0                           //   END_COLLECTION
  }; /* CustomHID_ReportDescriptor */

/* USB String Descriptors (optional) */
const uint8_t CustomHID_StringLangID[CUSTOMHID_SIZ_STRING_LANGID] =
  {
    CUSTOMHID_SIZ_STRING_LANGID,
    USB_STRING_DESCRIPTOR_TYPE,
    0x09,
    0x04
  }
  ; /* LangID = 0x0409: U.S. English */

const uint8_t CustomHID_StringVendor[CUSTOMHID_SIZ_STRING_VENDOR] =
  {
    CUSTOMHID_SIZ_STRING_VENDOR, /* Size of Vendor string */
    USB_STRING_DESCRIPTOR_TYPE,  /* bDescriptorType*/
    /* Manufacturer: "Oculus VR, Inc." */
    'O', 0, 'c', 0, 'u', 0, 'l', 0, 'u', 0, 's', 0, ' ', 0, 'V', 0,
    'R', 0, ',', 0, ' ', 0, 'I', 0, 'n', 0, 'c', 0, '.', 0
  };

const uint8_t CustomHID_StringProduct[CUSTOMHID_SIZ_STRING_PRODUCT] =
  {
    CUSTOMHID_SIZ_STRING_PRODUCT,          /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
    'R', 0, 'i', 0, 'f', 0, 't', 0, ' ', 0, 'D', 0, 'K', 0, '2', 0
  };

#if UUID_LEN != 20
    #error "UUID LEN has changed, update size of default serial string below"
#endif

uint8_t CustomHID_StringSerial[CUSTOMHID_SIZ_STRING_SERIAL] =
  {
    CUSTOMHID_SIZ_STRING_SERIAL,           /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
    'A', 0, 'B', 0, 'C', 0, 'D', 0, 'E', 0, 'F', 0, 'G', 0, 'H', 0, 'I', 0, 'J', 0,
    '1', 0, '2', 0, '3', 0, '4', 0, '5', 0, '6', 0, '7', 0, '8', 0, '9', 0, '0', 0
  };

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

