/**************************************************************************/
/*!
    @file     main.c
    @author   K. Townsend (microBuilder.eu)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, K. Townsend
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
#include "projectconfig.h"
#include "core/systick/systick.h"
#include "boards/board.h"

#ifdef CFG_INTERFACE
  #include "cli/cli.h"
#endif

#ifdef CFG_USB
  #include "core/usb/usbd.h"
  #ifdef CFG_USB_HID_GENERIC
    /* Buffer to hold incoming HID data */
    static USB_HID_GenericReport_t hid_out_report;
    static bool is_received_report = false;
  #endif
#endif

#if defined(__CODE_RED)
  #include <cr_section_macros.h>
  #include <NXP/crp.h>
  __CRP const unsigned int CRP_WORD = CRP_NO_CRP ;
#endif

int main(void)
{
  uint32_t currentSecond, lastSecond;
  currentSecond = lastSecond = 0;

  /* Configure common and board-level peripherals */
  boardInit();

  while (1)
  {
    currentSecond = systickGetSecondsActive();
    if (currentSecond != lastSecond)
    {
      lastSecond = currentSecond;

      /* Toggle LED once per second */
      boardLED(lastSecond % 2);

      /* Display any incoming HID data if HID GENERIC is enabled */
      #ifdef CFG_USB_HID_GENERIC
        if(usb_isConfigured())
        {
          if(is_received_report)
          {
            uint32_t i;
            for (i=0; i< sizeof(USB_HID_GenericReport_t); i++)
            {
              printf("%02x ", hid_out_report.report[i]);
            }
            printf(CFG_PRINTF_NEWLINE);
            is_received_report = false;
          }
        }
      #endif
    }

    /* Poll for CLI input if CFG_INTERFACE is enabled */
    #ifdef CFG_INTERFACE
      cliPoll();
    #endif
  }

  return 0;
}

#ifdef CFG_USB_HID_GENERIC
void usb_hid_generic_recv_isr(USB_HID_GenericReport_t *out_report)
{
  /* Copy out_report to a buffer in case new data comes in */
  memcpy(&hid_out_report, out_report, sizeof(USB_HID_GenericReport_t));
  is_received_report = true;
}
#endif
