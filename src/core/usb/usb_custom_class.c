/**************************************************************************/
/*!
 @file     usb_custom_class.c
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

//--------------------------------------------------------------------+
// INCLUDE
//--------------------------------------------------------------------+
#include <string.h>
#include "usbd.h"

#ifdef CFG_USB_CUSTOM_CLASS

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+
//static ErrorCode_t endpoint_control_isr(USBD_HANDLE_T hUsb, void* data, uint32_t event)
//{
//  return LPC_OK;
//}

static ErrorCode_t endpoint_bulk_in_isr (USBD_HANDLE_T husb, void* data, uint32_t event)
{
//  if (usb_custom_received_isr)
//  {
//    usb_custom_received_isr()
//  }

  if (USB_EVT_IN == event)
  {
    static uint32_t magic_number = 0;
    magic_number += 2;
    uint32_t buffer[16] = { magic_number }; // size is 64
    USBD_API->hw->WriteEP(husb, CUSTOM_EP_IN, (uint8_t*) buffer, 64); // write data to EP
  }

  return LPC_OK;
}

static ErrorCode_t endpoint_bulk_out_isr (USBD_HANDLE_T husb, void* data, uint32_t event)
{
  if (USB_EVT_OUT == event)
  {
    uint8_t buffer[64] = { 0 }; // size is 64
    USBD_API->hw->ReadEP(husb, CUSTOM_EP_OUT, buffer);
  }
  return LPC_OK;
}

//--------------------------------------------------------------------+
// IMPLEMENTATION
//--------------------------------------------------------------------+
ErrorCode_t usb_custom_init (USBD_HANDLE_T husb, USB_INTERFACE_DESCRIPTOR const * p_interface)
{
  (void) p_interface;

//  ASSERT_USB_STATUS ( USBD_API->core->RegisterClassHandler(husb, endpoint_control_isr, NULL) );

  ASSERT_USB_STATUS ( USBD_API->core->RegisterEpHandler (husb, ((CUSTOM_EP_IN & 0x0F) << 1) +1, endpoint_bulk_in_isr , NULL) );
  ASSERT_USB_STATUS ( USBD_API->core->RegisterEpHandler (husb, (CUSTOM_EP_OUT & 0x0F) << 1 , endpoint_bulk_out_isr, NULL) );
  return LPC_OK;
}

ErrorCode_t usb_custom_configured (USBD_HANDLE_T husb)
{
  uint8_t dummy;
  USBD_API->hw->WriteEP(husb , CUSTOM_EP_IN , &dummy , 1); // initial packet for IN endpoint , will not work if omitted
  return LPC_OK;
}

#endif
