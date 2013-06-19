/**************************************************************************/
/*!
    @file     app_usbd_cfg.h
    @author   Thach Ha (tinyusb.net)

    @section DESCRIPTION

    Common USB daemon config settings

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
#ifndef __APP_USBD_CFG_H__
#define __APP_USBD_CFG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "romdriver/mw_usbd.h"

#define USB_MAX_IF_NUM                      (8)
#define USB_MAX_EP_NUM                      (5)

#define USB_FS_MAX_BULK_PACKET              (64)
#define USB_HS_MAX_BULK_PACKET              (USB_FS_MAX_BULK_PACKET) /* Full speed device only */

// Control Endpoint
#define USB_MAX_PACKET0                     (64)

#ifdef CFG_USB_CDC
  #define INTERFACES_OF_CDC                 (2)
#else
  #define INTERFACES_OF_CDC                 (0)
#endif

#ifdef CFG_USB_HID_KEYBOARD
  #define INTERFACES_OF_HID_KEYBOARD        (1)
#else
  #define INTERFACES_OF_HID_KEYBOARD        (0)
#endif

#ifdef CFG_USB_HID_MOUSE
  #define INTERFACES_OF_HID_MOUSE           (1)
#else
  #define INTERFACES_OF_HID_MOUSE           (0)
#endif


#ifdef CFG_USB_HID_GENERIC
  #define INTERFACES_OF_HID_GENERIC         (1)
#else
  #define INTERFACES_OF_HID_GENERIC         (0)
#endif

#ifdef CFG_USB_MSC
  #define INTERFACES_OF_MSC                 (1)
#else
  #define INTERFACES_OF_MSC                 (0)
#endif

#ifdef CFG_USB_CUSTOM_CLASS
  #define INTERFACES_OF_CUSTOM              (1)
#else
  #define INTERFACES_OF_CUSTOM              (0)
#endif

#define INTERFACE_INDEX_CDC                 (0)
#define INTERFACE_INDEX_HID_KEYBOARD        (INTERFACE_INDEX_CDC          + INTERFACES_OF_CDC          )
#define INTERFACE_INDEX_HID_MOUSE           (INTERFACE_INDEX_HID_KEYBOARD + INTERFACES_OF_HID_KEYBOARD )
#define INTERFACE_INDEX_HID_GENERIC         (INTERFACE_INDEX_HID_MOUSE    + INTERFACES_OF_HID_MOUSE    )
#define INTERFACE_INDEX_MSC                 (INTERFACE_INDEX_HID_GENERIC  + INTERFACES_OF_HID_GENERIC  )
#define INTERFACE_INDEX_CUSTOM              (INTERFACE_INDEX_MSC          + INTERFACES_OF_MSC          )

#define TOTAL_INTEFACES                     (INTERFACES_OF_CDC + INTERFACES_OF_HID_KEYBOARD + INTERFACES_OF_HID_MOUSE +\
    INTERFACES_OF_HID_GENERIC + INTERFACES_OF_MSC + INTERFACES_OF_CUSTOM)

// Number of Endpoint IN used by CDC, HID is equal to the number of its interface, here we used INTERFACES_OF_CLASS as EP_IN_USED_BY_CLASS
#define  CDC_NUMBER_OF_EP_IN                (INTERFACES_OF_CDC)
#define  HID_KEYBOARD_NUMBER_OF_EP_IN       (INTERFACES_OF_HID_KEYBOARD)
#define  HID_MOUSE_NUMBER_OF_EP_IN          (INTERFACES_OF_HID_MOUSE)
#define  HID_GENERIC_NUMBER_OF_EP_IN        (INTERFACES_OF_HID_GENERIC)
#define  MSC_NUMBER_OF_EP_IN                (INTERFACES_OF_MSC)
#define  CUSTOM_NUMBER_OF_EP_IN             (INTERFACES_OF_CUSTOM)

#define  EP_IN_TOTAL                        (CDC_NUMBER_OF_EP_IN + HID_KEYBOARD_NUMBER_OF_EP_IN + HID_MOUSE_NUMBER_OF_EP_IN+\
    HID_GENERIC_NUMBER_OF_EP_IN + MSC_NUMBER_OF_EP_IN + CUSTOM_NUMBER_OF_EP_IN)

/* CDC Endpoint Address */
#define  CDC_NOTIFICATION_EP                (USB_ENDPOINT_IN(1))
#define  CDC_DATA_EP_IN                     (USB_ENDPOINT_IN(2))
#define  CDC_DATA_EP_OUT                    (USB_ENDPOINT_OUT(1))
#define  CDC_NOTIFICATION_EP_MAXPACKETSIZE  (8)
#define  CDC_DATA_EP_MAXPACKET_SIZE         (16)

/*       HID                                In/Out                Endpoint  Address  */
#define  HID_KEYBOARD_EP_IN                 (CDC_NOTIFICATION_EP  + CDC_NUMBER_OF_EP_IN)
#define  HID_MOUSE_EP_IN                    (HID_KEYBOARD_EP_IN   + HID_KEYBOARD_NUMBER_OF_EP_IN)
#define  HID_GENERIC_EP_IN                  (HID_MOUSE_EP_IN      + HID_MOUSE_NUMBER_OF_EP_IN)
#define  HID_GENERIC_EP_OUT                 (USB_ENDPOINT_OUT(2))

//       MSC                                Enpoint               Address
#define  MSC_EP_IN                          (HID_GENERIC_EP_IN    + HID_GENERIC_NUMBER_OF_EP_IN)
#define  MSC_EP_OUT                         (USB_ENDPOINT_OUT(3))

// CUSTOM CLASS
#define  CUSTOM_EP_IN                       (MSC_EP_IN + MSC_NUMBER_OF_EP_IN)
#define  CUSTOM_EP_OUT                      (USB_ENDPOINT_OUT(4))

#if (EP_IN_TOTAL > 4)
  #error lpc11uxx and lpc13uxx only has up to 4 IN endpoints
#endif

#ifdef __cplusplus
}
#endif 

#endif  /* __USBCFG_H__ */
