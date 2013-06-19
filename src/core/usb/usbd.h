/**************************************************************************/
/*!
    @file     usbd.h
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
#ifndef __USBD_H__
#define __USBD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"
#include "romdriver/mw_usbd_rom_api.h"
#include "../power_api.h"
#include "descriptors.h"

#ifdef CFG_USB_HID
  #include "usb_hid.h"
#endif

#ifdef CFG_USB_CDC
  #include "usb_cdc.h"
#endif

#ifdef CFG_USB_MSC
  #include "usb_msc.h"
#endif

#ifdef CFG_USB_CUSTOM_CLASS
  #include "usb_custom_class.h"
#endif

#define USBD_API     ((*(ROM **)(0x1FFF1FF8))->pUSBD)

ErrorCode_t usb_init(void);
bool usb_isConfigured(void);

extern USBD_HANDLE_T g_hUsb;

#define ASSERT_USB_STATUS_MESSAGE(sts, message) \
        do{\
          ErrorCode_t status = (sts);\
          if (LPC_OK != status) {\
            _PRINTF("Assert: '%s' at line %d: 0x%X %s%s", __func__, __LINE__, (uint32_t) status, message, CFG_PRINTF_NEWLINE);\
            return status;\
          }\
        }while(0)

#define ASSERT_USB_STATUS(sts)                ASSERT_USB_STATUS_MESSAGE(sts, NULL)

#ifdef __cplusplus
}
#endif 

#endif
