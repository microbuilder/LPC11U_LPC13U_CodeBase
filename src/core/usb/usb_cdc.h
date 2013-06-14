/**************************************************************************/
/*!
    @file     usb_cdc.h
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
#ifndef __USB_CDC_H__
#define __USB_CDC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"
#include "romdriver/mw_usbd_rom_api.h"

// #define CDC_BUFFER_SIZE (2*CDC_DATA_EP_MAXPACKET_SIZE)
#define CDC_BUFFER_SIZE (4*CDC_DATA_EP_MAXPACKET_SIZE)

bool usb_cdc_putc(uint8_t c);
bool usb_cdc_getc(uint8_t *c);
bool usb_cdc_isConnected();

uint16_t usb_cdc_send(uint8_t* buffer, uint16_t count);
uint16_t usb_cdc_recv(uint8_t* buffer, uint16_t max);

ErrorCode_t usb_cdc_init(USBD_HANDLE_T hUsb, USB_INTERFACE_DESCRIPTOR const *const pControlIntfDesc, USB_INTERFACE_DESCRIPTOR const *const pDataIntfDesc, uint32_t* mem_base, uint32_t* mem_size);
ErrorCode_t usb_cdc_configured(USBD_HANDLE_T hUsb);

#ifdef __cplusplus
}
#endif 

#endif
