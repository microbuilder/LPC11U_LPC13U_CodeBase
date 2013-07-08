/**************************************************************************/
/*!
    @file     usb_custom_class.h
    @author   hathach (tinyusb.org)

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

/** \ingroup TBD
 *  \defgroup TBD
 *  \brief TBD
 *
 *  @{
 */

#ifndef __USB_CUSTOM_CLASS_H__
#define __USB_CUSTOM_CLASS_H__

#include "projectconfig.h"
#include "romdriver/mw_usbd_rom_api.h"

#ifdef __cplusplus
 extern "C" {
#endif

//--------------------------------------------------------------------+
// APPLICATION API
//--------------------------------------------------------------------+
bool usb_custom_is_ready_to_send(void);
ErrorCode_t usb_custom_send(uint8_t const * p_data, uint32_t length);
void usb_custom_received_isr(uint8_t * p_buffer, uint32_t length) __attribute__((weak));

//--------------------------------------------------------------------+
// USBD-CLASS API
//--------------------------------------------------------------------+
ErrorCode_t usb_custom_init(USBD_HANDLE_T husb, USB_INTERFACE_DESCRIPTOR const * p_interface);
ErrorCode_t usb_custom_configured(USBD_HANDLE_T husb);


#ifdef __cplusplus
 }
#endif

#endif /* __USB_CUSTOM_CLASS_H__ */

/** @} */
