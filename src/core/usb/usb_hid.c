/**************************************************************************/
/*!
    @file     usb_hid.c
    @author   Thach Ha (tinyusb.net)

    @section DESCRIPTION

    HID support functions for USB

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
#include <string.h>
#include "usbd.h"
#include "../systick/systick.h"

#ifdef CFG_USB_HID

#ifdef CFG_USB_HID_KEYBOARD
USB_HID_KeyboardReport_t hid_keyboard_report;
volatile static bool bKeyChanged = false;
#endif

#ifdef CFG_USB_HID_MOUSE
USB_HID_MouseReport_t hid_mouse_report;
volatile static bool bMouseChanged = false;
#endif

#ifdef CFG_USB_HID_GENERIC
#define ALIAS(f) __attribute__ ((weak, alias (#f)))

USB_HID_GenericReport_t hid_generic_report;
volatile static bool bGenericChanged= false;

void usb_hid_generic_recv_isr(USB_HID_GenericReport_t *out_report) ALIAS(usb_hid_generic_recv_isr_default);

/**************************************************************************/
/*!
    @brief      Weak ISR handler for HID Generic out reports (PC to LPC).

    @param[in]  report
                Pointer to the USB_HID_GenericReport_t that holds the
                incoming report data

    @note       Since this is a 'weak' function, to override it you
                simply need to declare a new function with the same name
                somewhere else in your code.

    @code
    // Buffer to hold incoming HID data
    static USB_HID_GenericReport_t hid_out_report;
    static bool is_received_report = false;

    int main(void)
    {
      ...
      while(1)
      {
        ...
        #ifdef CFG_USB_HID_GENERIC
          if(usb_isConfigured())
          {
            if(is_received_report)
            {
              uint32_t i;
              for (i=0; i< sizeof(USB_HID_GenericReport_t); i++)
              {
                // Display incoming HID data with CDC using printf
                printf("%02x ", hid_out_report.report[i]);
              }
              printf(CFG_PRINTF_NEWLINE);
              is_received_report = false;
            }
          }
        #endif
      }
    }

    void usb_hid_generic_recv_isr(USB_HID_GenericReport_t *out_report)
    {
      // Copy out_report to a buffer in case new data comes in
      memcpy(&hid_out_report, out_report, sizeof(USB_HID_GenericReport_t));
      is_received_report = true;
    }
    @endcode
*/
/**************************************************************************/
void usb_hid_generic_recv_isr_default(USB_HID_GenericReport_t *out_report)
{
  return;
}
#endif

/**************************************************************************/
/*!
    @brief Handler for HID_GetReport in the USB ROM driver
*/
/**************************************************************************/
ErrorCode_t HID_GetReport( USBD_HANDLE_T hHid, USB_SETUP_PACKET* pSetup, uint8_t** pBuffer, uint16_t* plength)
{
  USB_HID_CTRL_T* pHidCtrl = (USB_HID_CTRL_T*) hHid;

  /* ReportID = SetupPacket.wValue.WB.L; */
  if (pSetup->wValue.WB.H != HID_REPORT_INPUT)
    return (ERR_USBD_STALL);          /* Not Supported */

  switch (pHidCtrl->protocol)
  {
#ifdef CFG_USB_HID_KEYBOARD
    case HID_PROTOCOL_KEYBOARD:
      *pBuffer = (uint8_t*) &hid_keyboard_report;
      *plength = sizeof(USB_HID_KeyboardReport_t);

      if (!bKeyChanged)
      {
        memset(pBuffer, 0, *plength);
      }
      bKeyChanged = false;
      break;
#endif

#ifdef CFG_USB_HID_MOUSE
    case HID_PROTOCOL_MOUSE:
      *pBuffer = (uint8_t*) &hid_mouse_report;
      *plength = sizeof(USB_HID_MouseReport_t);

      if (!bMouseChanged)
      {
        memset(pBuffer, 0, *plength);
      }
      bMouseChanged = false;
      break;
#endif

    default:
#ifdef CFG_USB_HID_GENERIC
      if (pHidCtrl->epin_adr == HID_GENERIC_EP_IN)
      {
        *pBuffer = (uint8_t*) &hid_generic_report;
        *plength = sizeof(USB_HID_GenericReport_t);

        if (!bGenericChanged)
        {
          memset(pBuffer, 0x00, *plength);
        }
        bGenericChanged = false;
      }
#endif
    break;
  }

  return (LPC_OK);
}

/**************************************************************************/
/*!
    @brief Handler for HID_SetReport in the USB ROM driver
*/
/**************************************************************************/
ErrorCode_t HID_SetReport( USBD_HANDLE_T hHid, USB_SETUP_PACKET* pSetup, uint8_t** pBuffer, uint16_t length)
{
  USB_HID_CTRL_T* pHidCtrl = (USB_HID_CTRL_T*) hHid;

  /* Reuse standard EP0Buf */
  if (length == 0)
    return LPC_OK;

  /* ReportID = SetupPacket.wValue.WB.L; */
  if (pSetup->wValue.WB.H != HID_REPORT_OUTPUT)
    return (ERR_USBD_STALL);          /* Not Supported */

  switch (pHidCtrl->protocol)
  {
#ifdef CFG_USB_HID_KEYBOARD
    case HID_PROTOCOL_KEYBOARD:

    break;
#endif

#ifdef CFG_USB_HID_MOUSE
    case HID_PROTOCOL_MOUSE:

    break;
#endif

    default:
#ifdef CFG_USB_HID_GENERIC
      if (pHidCtrl->epout_adr == HID_GENERIC_EP_OUT)
      {
        ASSERT_USB_STATUS(CFG_USB_HID_GENERIC_REPORT_SIZE != length);
        usb_hid_generic_recv_isr((USB_HID_GenericReport_t*) (*pBuffer));
      }
#endif
    break;
  }
  return (LPC_OK);
}

/**************************************************************************/
/*!
    @brief HID endpoint in handler for the USB ROM driver
*/
/**************************************************************************/
ErrorCode_t HID_EpIn_Hdlr (USBD_HANDLE_T hUsb, void* data, uint32_t event)
{
  if (USB_EVT_IN == event)
  {
    USB_HID_CTRL_T* pHidCtrl = (USB_HID_CTRL_T*)data;
    switch(pHidCtrl->protocol)
    {
      #ifdef CFG_USB_HID_KEYBOARD
        case HID_PROTOCOL_KEYBOARD:
          if (!bKeyChanged)
          {
            memset(&hid_keyboard_report, 0, sizeof(USB_HID_KeyboardReport_t));
          }
          USBD_API->hw->WriteEP(hUsb, pHidCtrl->epin_adr, (uint8_t*) &hid_keyboard_report, sizeof(USB_HID_KeyboardReport_t));
          bKeyChanged = false;
        break;
      #endif

      #ifdef CFG_USB_HID_MOUSE
        case HID_PROTOCOL_MOUSE:
          if (!bMouseChanged)
          {
            memset(&hid_mouse_report, 0, sizeof(USB_HID_MouseReport_t));
          }
          USBD_API->hw->WriteEP(hUsb, pHidCtrl->epin_adr, (uint8_t*) &hid_mouse_report, sizeof(USB_HID_MouseReport_t));
          bMouseChanged = false;
        break;
      #endif

      default:
      #ifdef CFG_USB_HID_GENERIC
        if (pHidCtrl->epin_adr == HID_GENERIC_EP_IN)
        {
          if (bGenericChanged)
          {
            USBD_API->hw->WriteEP(hUsb, pHidCtrl->epin_adr, (uint8_t*) &hid_generic_report, sizeof(USB_HID_GenericReport_t));
          }else
          {
            USBD_API->hw->WriteEP(hUsb, pHidCtrl->epin_adr, NULL, 0); // write size = 0 for NAK TODO need to be confirmed
          }
          bGenericChanged = false;
        }
      #endif
        break;
    }
  }

  return LPC_OK;
}

/**************************************************************************/
/*!
    @brief HID endpoint out handler for the USB ROM driver
*/
/**************************************************************************/
ErrorCode_t HID_EpOut_Hdlr (USBD_HANDLE_T hUsb, void* data, uint32_t event)
{
  if (USB_EVT_OUT == event)
  {
    USB_HID_CTRL_T* pHidCtrl = (USB_HID_CTRL_T*)data;

#ifdef CFG_USB_HID_GENERIC
    if (pHidCtrl->epout_adr == HID_GENERIC_EP_OUT)
    {
      USB_HID_GenericReport_t out_report;
      uint32_t length;

      length = USBD_API->hw->ReadEP(hUsb, pHidCtrl->epout_adr, (uint8_t*) &out_report);
      ASSERT_USB_STATUS(CFG_USB_HID_GENERIC_REPORT_SIZE != length);

      usb_hid_generic_recv_isr(&out_report);
    }
#endif
  }
  return LPC_OK;
}

/**************************************************************************/
/*!
    @brief Initialises USB HID using the ROM based drivers
*/
/**************************************************************************/
ErrorCode_t usb_hid_init(USBD_HANDLE_T hUsb, USB_INTERFACE_DESCRIPTOR const *const pIntfDesc, uint8_t const * const pHIDReportDesc, uint32_t ReportDescLength, uint32_t* mem_base, uint32_t* mem_size)
{
  USB_HID_REPORT_T reports_data =
  {
      .desc      = (uint8_t*) pHIDReportDesc,
      .len       = ReportDescLength,
      .idle_time = 0,
  };

  USBD_HID_INIT_PARAM_T hid_param =
  {
      .mem_base       = *mem_base,
      .mem_size       = *mem_size,

      .intf_desc      = (uint8_t*)pIntfDesc,
      .report_data    = &reports_data,
      .max_reports    = 1,

      /* user defined functions */
      .HID_GetReport  = HID_GetReport,
      .HID_SetReport  = HID_SetReport,
      .HID_EpIn_Hdlr  = HID_EpIn_Hdlr,
      .HID_EpOut_Hdlr = HID_EpOut_Hdlr
  };

  ASSERT( (pIntfDesc != NULL) && (pIntfDesc->bInterfaceClass == USB_DEVICE_CLASS_HUMAN_INTERFACE), ERR_FAILED);

  ASSERT_USB_STATUS( USBD_API->hid->init(hUsb, &hid_param) );

  /* update memory variables */
  ASSERT_MESSAGE(*mem_size > hid_param.mem_size, ERR_FAILED, "not enough memory");

  *mem_base += (*mem_size - hid_param.mem_size);
  *mem_size = hid_param.mem_size;

  return LPC_OK;
}

/**************************************************************************/
/*!
    @brief  Callback when the USB Set Configured request is received
*/
/**************************************************************************/
ErrorCode_t usb_hid_configured(USBD_HANDLE_T hUsb)
{
  #ifdef  CFG_USB_HID_KEYBOARD
    USBD_API->hw->WriteEP(hUsb , HID_KEYBOARD_EP_IN , (uint8_t* ) &hid_keyboard_report , sizeof(USB_HID_KeyboardReport_t) ); // initial packet for IN endpoint , will not work if omitted
  #endif

  #ifdef  CFG_USB_HID_MOUSE
    USBD_API->hw->WriteEP(hUsb , HID_MOUSE_EP_IN    , (uint8_t* ) &hid_mouse_report    , sizeof(USB_HID_MouseReport_t) ); // initial packet for IN endpoint, will not work if omitted
  #endif

  #ifdef CFG_USB_HID_GENERIC
    USBD_API->hw->WriteEP(hUsb , HID_GENERIC_EP_IN  , (uint8_t* ) &hid_generic_report  , sizeof(USB_HID_GenericReport_t) ); // initial packet for IN endpoint, will not work if omitted
  #endif

  return LPC_OK;
}

#ifdef CFG_USB_HID_KEYBOARD
/**************************************************************************/
/*!
    @brief Send the supplied key codes out via HID USB keyboard emulation

    @param[in]  modifier
                KB modifier code bits (see USB_HID_KB_KEYMODIFIER_CODE)
    @param[in]  keycodes
                A buffer containing up to six keycodes
    @param[in]  numkey
                The number of keys to send (max 6)

    @note Note that for HID KBs, letter codes are not case sensitive. To
          create an upper-case letter, you need to include the correct
          KB modifier code(s), for ex: HID_KEYMODIFIER_LEFTSHIFT

    @code
    // Send an unmodified 'a' character
    if (usb_isConfigured())
    {
      uint8_t keys[6] = {HID_USAGE_KEYBOARD_aA};
      usb_hid_keyboard_sendKeys(0x00, keys, 1);
    }

    // Send Windows + 'e' (shortcut for 'explorer.exe')
    if (usb_isConfigured())
    {
      uint8_t keys[6] = {HID_USAGE_KEYBOARD_aA + 'e' - 'a'};
      usb_hid_keyboard_sendKeys(HID_KEYMODIFIER_LEFTGUI, keys, 1);
    }
    @endcode
*/
/**************************************************************************/
ErrorCode_t usb_hid_keyboard_sendKeys(uint8_t modifier, uint8_t keycodes[], uint8_t numkey)
{
  uint32_t start_time = systickGetSecondsActive();
  while (bKeyChanged) // TODO blocking while previous key has yet sent - can use fifo to improve this
  {
    ASSERT_MESSAGE(systickGetSecondsActive() - start_time < 5, ERR_FAILED, "HID Keyboard Timeout");
  }
  ASSERT(keycodes && numkey && numkey <=6, ERR_FAILED);

  hid_keyboard_report.Modifier = modifier;
  memset(hid_keyboard_report.KeyCode, 0, 6);
  memcpy(hid_keyboard_report.KeyCode, keycodes, numkey);

  bKeyChanged = true;

  return LPC_OK;
}
#endif

#ifdef CFG_USB_HID_MOUSE
/**************************************************************************/
/*!
    @brief Send the supplied mouse event out via HID USB mouse emulation

    @param[in]  buttons
                Indicate which button(s) are being pressed (see
                USB_HID_MOUSE_BUTTON_CODE)
    @param[in]  x
                Position adjustment on the X scale
    @param[in]  y
                Position adjustment on the Y scale
    @param[in]  wheel
                Position adjustment of the vertical scroll wheel
    @param[in]  pan
                Position adjustment on the horizontal scroll wheel

    @code
    if (usb_isConfigured())
    {
      // Move the mouse +10 in the X direction and + 10 in the Y direction
      usb_hid_mouse_send(0, 10, 10, 0, 0);

      // Click the back mouse button
      usb_hid_mouse_send(HID_MOUSEBUTTON_BACKWARD, 0, 0, 0, 0);

      // Advance the middle scroll wheel
      usb_hid_mouse_send(0, 0, 0, 5, 0);
    }
    @endcode
*/
/**************************************************************************/
ErrorCode_t usb_hid_mouse_send(uint8_t buttons, int8_t x, int8_t y, int8_t wheel, int8_t pan)
{
  uint32_t start_time = systickGetSecondsActive();
  while (bMouseChanged) // TODO Block while previous key hasn't been sent - can use fifo to improve this
  {
    ASSERT_MESSAGE(systickGetSecondsActive() - start_time < 5, ERR_FAILED, "HID Mouse Timeout");
  }

  hid_mouse_report.Button = buttons;
  hid_mouse_report.X      = x;
  hid_mouse_report.Y      = y;
  hid_mouse_report.Wheel  = wheel;
  hid_mouse_report.Pan    = pan;

  bMouseChanged = true;

  return LPC_OK;
}
#endif

#ifdef CFG_USB_HID_GENERIC


/**************************************************************************/
/*!
    @brief Send the specified HID report to the host

    @param[in]  report
                The USB_HID_GenericReport_t instance containing the
                report values to transmit to the host

    @code
    if (usb_isConfigured())
    {
      uint32_t currentSecond = systickGetSecondsActive();
      USB_HID_GenericReport_t in_report = { .report = {currentSecond % 100} };
      usb_hid_generic_send(&in_report);
    }
    @endcode
*/
/**************************************************************************/
ErrorCode_t usb_hid_generic_send(USB_HID_GenericReport_t *report)
{
  uint32_t start_time = systickGetSecondsActive();

  ASSERT(report, ERR_FAILED);
  while (bGenericChanged) // TODO Block while previous key hasn't been sent - can use fifo to improve this
  {
    ASSERT_MESSAGE(systickGetSecondsActive() - start_time < 5, ERR_FAILED, "HID Generic Timeout");
  }

  memcpy(&hid_generic_report, report, sizeof(USB_HID_GenericReport_t));
  bGenericChanged = true;

  return LPC_OK;
}
#endif

#endif
