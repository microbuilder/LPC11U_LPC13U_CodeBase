/**************************************************************************/
/*!
    @file     descriptors.h
    @author   Thach Ha (tinyusb.net)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2012, K. Townsend (microBuilder.eu)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#ifndef _DESCRIPTORS_H_
#define _DESCRIPTORS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"
#include "app_usbd_cfg.h"
#include "romdriver/mw_usbd_rom_api.h"
#include "usbd.h"

#ifdef CFG_USB

/* USB Serial uses the MCUs unique 128-bit chip ID via an IAP call = 32 hex chars */
#define USB_STRING_SERIAL_LEN     32

#define USB_STRING_LEN(n) (2 + ((n)<<1))

typedef PRE_PACK struct POST_PACK _USB_STR_DESCRIPTOR
{
  USB_COMMON_DESCRIPTOR LangID;
  uint16_t strLangID[1];

  USB_COMMON_DESCRIPTOR Manufacturer;
  uint16_t strManufacturer[sizeof(CFG_USB_STRING_MANUFACTURER)-1]; // exclude null-character

  USB_COMMON_DESCRIPTOR Product;
  uint16_t strProduct[sizeof(CFG_USB_STRING_PRODUCT)-1]; // exclude null-character

  USB_COMMON_DESCRIPTOR Serial;
  uint16_t strSerial[USB_STRING_SERIAL_LEN];
} USB_STR_DESCRIPTOR;

// USB Interface Assosication Descriptor
#define  USB_DEVICE_CLASS_IAD        USB_DEVICE_CLASS_MISCELLANEOUS
#define  USB_DEVICE_SUBCLASS_IAD     0x02
#define  USB_DEVICE_PROTOCOL_IAD     0x01

// USB Interface Association Descriptor
typedef PRE_PACK struct POST_PACK _USB_INTERFACE_ASSOCIATION_DESCRIPTOR
{
  uint8_t bLength;           /**< Size of descriptor*/
  uint8_t bDescriptorType;   /**< Other_speed_Configuration Type*/

  uint8_t bFirstInterface;   /**< Index of the first associated interface. */
  uint8_t bInterfaceCount;   /**< Total number of associated interfaces. */

  uint8_t bFunctionClass;    /**< Interface class ID. */
  uint8_t bFunctionSubClass; /**< Interface subclass ID. */
  uint8_t bFunctionProtocol; /**< Interface protocol ID. */

  uint8_t iFunction;         /**< Index of the string descriptor describing the interface association. */
} USB_INTERFACE_ASSOCIATION_DESCRIPTOR;

///////////////////////////////////////////////////////////////////////
// Interface Assosication Descriptor if device is CDC + other class
#define IAD_DESC_REQUIRED ( defined(CFG_USB_CDC) && ( defined(CFG_USB_HID) || defined(CFG_USB_MSC) || defined(CFG_USB_CUSTOM_CLASS)) )

#ifndef USB_PRODUCT_ID
// Bitmap: MassStorage | Generic | Mouse | Key | CDC
#define PRODUCTID_BITMAP(interface, n)  ( (INTERFACES_OF_##interface ? 1 : 0) << (n) )
#define USB_PRODUCT_ID                  (0x2000 | ( PRODUCTID_BITMAP(CDC, 0) | PRODUCTID_BITMAP(HID_KEYBOARD, 1) |\
                                         PRODUCTID_BITMAP(HID_MOUSE, 2) | PRODUCTID_BITMAP(HID_GENERIC, 3) |\
                                         PRODUCTID_BITMAP(MSC, 4) | PRODUCTID_BITMAP(CUSTOM, 5) ) )
#endif

///////////////////////////////////////////////////////////////////////
typedef struct
{
  USB_CONFIGURATION_DESCRIPTOR                Config;

#if IAD_DESC_REQUIRED
  USB_INTERFACE_ASSOCIATION_DESCRIPTOR        CDC_IAD;
#endif

#ifdef CFG_USB_CDC
  //CDC - Serial
  //CDC Control Interface
  USB_INTERFACE_DESCRIPTOR                    CDC_CCI_Interface;
  CDC_HEADER_DESCRIPTOR                       CDC_Header;
  CDC_ABSTRACT_CONTROL_MANAGEMENT_DESCRIPTOR  CDC_ACM;
  CDC_UNION_1SLAVE_DESCRIPTOR                 CDC_Union;
  USB_ENDPOINT_DESCRIPTOR                     CDC_NotificationEndpoint;

  //CDC Data Interface
  USB_INTERFACE_DESCRIPTOR                    CDC_DCI_Interface;
  USB_ENDPOINT_DESCRIPTOR                     CDC_DataOutEndpoint;
  USB_ENDPOINT_DESCRIPTOR                     CDC_DataInEndpoint;
#endif

#ifdef CFG_USB_HID_KEYBOARD
  //Keyboard HID Interface
  USB_INTERFACE_DESCRIPTOR                    HID_KeyboardInterface;
  HID_DESCRIPTOR                              HID_KeyboardHID;
  USB_ENDPOINT_DESCRIPTOR                     HID_KeyboardEndpoint;
#endif

#ifdef CFG_USB_HID_MOUSE
  //Mouse HID Interface
  USB_INTERFACE_DESCRIPTOR                    HID_MouseInterface;
  HID_DESCRIPTOR                              HID_MouseHID;
  USB_ENDPOINT_DESCRIPTOR                     HID_MouseEndpoint;
#endif

#ifdef CFG_USB_HID_GENERIC
  //Generic HID Interface
  USB_INTERFACE_DESCRIPTOR                    HID_GenericInterface;
  HID_DESCRIPTOR                              HID_GenericHID;
  USB_ENDPOINT_DESCRIPTOR                     HID_GenericINEndpoint;
  USB_ENDPOINT_DESCRIPTOR                     HID_GenericOUTEndpoint;

#endif

#ifdef CFG_USB_MSC
  USB_INTERFACE_DESCRIPTOR                    MSC_Interface;
  USB_ENDPOINT_DESCRIPTOR                     MSC_BulkIN;
  USB_ENDPOINT_DESCRIPTOR                     MSC_BulkOUT;
#endif

#ifdef CFG_USB_CUSTOM_CLASS
  USB_INTERFACE_DESCRIPTOR                    Custom_Interface;
  USB_ENDPOINT_DESCRIPTOR                     Custom_BulkIN;
  USB_ENDPOINT_DESCRIPTOR                     Custom_BulkOUT;
#endif

  unsigned char                               ConfigDescTermination;
} USB_FS_CONFIGURATION_DESCRIPTOR;

extern const USB_DEVICE_DESCRIPTOR USB_DeviceDescriptor;
extern const USB_FS_CONFIGURATION_DESCRIPTOR USB_FsConfigDescriptor;
extern USB_STR_DESCRIPTOR USB_StringDescriptor;

extern const uint8_t HID_KeyboardReportDescriptor[];
extern const uint8_t HID_MouseReportDescriptor[];
extern const uint8_t HID_GenericReportDescriptor[];

#endif /* CFG_USB */

#ifdef __cplusplus
}
#endif 

#endif
