/**************************************************************************/
/*!
    @file     usb_hid.h
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
#ifndef __USB_HID_H__
#define __USB_HID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"
#include "romdriver/mw_usbd_rom_api.h"

#define HID_USAGE_CONSUMER_ACPAN     0x38, 0x02 // 0x0238
#define HID_Usage_2Bytes(x)          0x0a, x

ErrorCode_t usb_hid_init(USBD_HANDLE_T hUsb, USB_INTERFACE_DESCRIPTOR const *const pIntfDesc, uint8_t const * const pHIDReportDesc, uint32_t ReportDescLength, uint32_t* mem_base, uint32_t* mem_size);
ErrorCode_t usb_hid_configured(USBD_HANDLE_T hUsb);

ErrorCode_t usb_hid_keyboard_sendKeys(uint8_t modifier, uint8_t keycodes[], uint8_t numkey);
ErrorCode_t usb_hid_mouse_send(uint8_t buttons, int8_t x, int8_t y, int8_t wheel, int8_t pan);
ErrorCode_t usb_hid_generic_send(uint8_t const* p_report_in, uint32_t length);
/**************************************************************************/
/*!
    @brief      Weak ISR handler for HID Generic out reports (PC to LPC).

    @param[in]  report
                Pointer to the buffer that holds the
                incoming report data

    @param[in]  length
                For most of the time it is CFG_USB_HID_GENERIC_REPORT_SIZE
                except for few time (if any) host sends out short-packet

    @note       Since this is a 'weak' function, to override it you
                simply need to declare a new function with the same name
                somewhere else in your code.

    @code
    // Buffer to hold incoming HID data
    static uint8_t hid_out_report[CFG_USB_HID_GENERIC_REPORT_SIZE];
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
              for (uint32_t i=0; i<CFG_USB_HID_GENERIC_REPORT_SIZE; i++)
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

    void usb_hid_generic_recv_isr(uint8_t out_report[], uint32_t length)
    {
      // Copy out_report to a buffer in case new data comes in
      memcpy(hid_out_report, out_report, length);
      is_received_report = true;
    }
    @endcode
*/
/**************************************************************************/
void usb_hid_generic_recv_isr(uint8_t * p_buffer, uint32_t length) __attribute__((weak));

// receive report in request from HOST, but have nothing to report
bool usb_hid_generic_report_request_isr(uint8_t in_report[]) __attribute__((weak));


/** \brief Standard HID Boot Protocol Mouse Report.
 *
 *  Type define for a standard Boot Protocol Mouse report
 */
typedef PRE_PACK struct
{
  uint8_t Button;      /**< Button mask for currently pressed buttons in the mouse. */
  int8_t  X;           /**< Current delta X movement of the mouse. */
  int8_t  Y;           /**< Current delta Y movement on the mouse. */
  int8_t  Wheel;
  int8_t  Pan;
} POST_PACK USB_HID_MouseReport_t;

/** \brief Standard HID Boot Protocol Keyboard Report.
 *
 *  Type define for a standard Boot Protocol Keyboard report
 */
typedef PRE_PACK struct
{
  uint8_t Modifier;    /**< Keyboard modifier byte, indicating pressed modifier keys (a combination of HID_KEYBOARD_MODIFER_* masks). */
  uint8_t Reserved;    /**< Reserved for OEM use, always set to 0. */
  uint8_t KeyCode[6];  /**< Key codes of the currently pressed keys. */
} POST_PACK USB_HID_KeyboardReport_t;

/* Button codes for HID mouse */
enum USB_HID_MOUSE_BUTTON_CODE
{
  HID_MOUSEBUTTON_LEFT     = 1,
  HID_MOUSEBUTTON_RIGHT    = 2,
  HID_MOUSEBUTTON_MIDDLE   = 4,
  HID_MOUSEBUTTON_BACKWARD = 8,
  HID_MOUSEBUTTON_FORWARD  = 16
};

/* KB modifier codes for HID KB */
enum USB_HID_KB_KEYMODIFIER_CODE
{
  HID_KEYMODIFIER_LEFTCTRL   = 1,
  HID_KEYMODIFIER_LEFTSHIFT  = 2,
  HID_KEYMODIFIER_LEFTALT    = 4,
  HID_KEYMODIFIER_LEFTGUI    = 8,
  HID_KEYMODIFIER_RIGHTCTRL  = 16,
  HID_KEYMODIFIER_RIGHTSHIFT = 32,
  HID_KEYMODIFIER_RIGHTALT   = 64,
  HID_KEYMODIFIER_RIGHTGUI   = 128
};

enum USB_HID_LOCAL_CODE
{
  HID_Local_NotSupported = 0,
  HID_Local_Arabic,
  HID_Local_Belgian,
  HID_Local_Canadian_Bilingual,
  HID_Local_Canadian_French,
  HID_Local_Czech_Republic,
  HID_Local_Danish,
  HID_Local_Finnish,
  HID_Local_French,
  HID_Local_German,
  HID_Local_Greek,
  HID_Local_Hebrew,
  HID_Local_Hungary,
  HID_Local_International,
  HID_Local_Italian,
  HID_Local_Japan_Katakana,
  HID_Local_Korean,
  HID_Local_Latin_American,
  HID_Local_Netherlands_Dutch,
  HID_Local_Norwegian,
  HID_Local_Persian_Farsi,
  HID_Local_Poland,
  HID_Local_Portuguese,
  HID_Local_Russia,
  HID_Local_Slovakia,
  HID_Local_Spanish,
  HID_Local_Swedish,
  HID_Local_Swiss_French,
  HID_Local_Swiss_German,
  HID_Local_Switzerland,
  HID_Local_Taiwan,
  HID_Local_Turkish_Q,
  HID_Local_UK,
  HID_Local_US,
  HID_Local_Yugoslavia,
  HID_Local_Turkish_F
};

#ifdef __cplusplus
}
#endif 

#endif
