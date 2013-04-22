/**************************************************************************/
/*!
    @file     printf-retarget.c
    @author   K. Townsend (microBuilder.eu)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2012, K. Townsend
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
#include <stdarg.h>

#include "projectconfig.h"

#ifdef __CROSSWORKS_ARM
  #if defined(CFG_PRINTF_DEBUG) || defined(CFG_PRINTF_USBCDC)
    #include <cross_studio_io.h>
  #endif
#endif

#ifdef CFG_USB
  #include "core/usb/usbd.h"
#endif

#ifdef CFG_PRINTF_UART
  #include "core/uart/uart.h"
#endif

/**************************************************************************/
/*!
    @brief  Sends a single byte to a pre-determined peripheral (UART, etc.).

    @param  c
            Byte value to send
*/
/**************************************************************************/
void __putchar(const char c)
{
  #if defined(CFG_USB) && defined(CFG_PRINTF_USBCDC)
    if (usb_isConfigured())
    {
      while(usb_cdc_isConnected() && !usb_cdc_putc(c) ) // blocking
      {
        ASM("nop");
      }
    }
  #endif

  #ifdef CFG_PRINTF_UART
    uartSendByte(c);
  #endif

  /* Handle PRINTF_DEBUG redirection for Crossworks for ARM */
  #ifdef __CROSSWORKS_ARM
    #ifdef CFG_PRINTF_DEBUG
      #if defined CFG_MCU_FAMILY_LPC13UXX
        /* On the M3 we can check if a debugger is connected */
        if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk)==CoreDebug_DHCSR_C_DEBUGEN_Msk)
        {
          debug_putchar(c);
        }
      #else
        /* On the M0 the processor doesn't have access to CoreDebug, so this
         * will cause problems if no debugger is connected! */
        debug_putchar(c);
      #endif
    #endif
  #endif
}

/**************************************************************************/
/*!
    @brief  Sends a string to a pre-determined peripheral (UART, etc.).
            This function is called by core/libc/stdio.c

    @param  c
            Byte value to send
*/
/**************************************************************************/
int puts(const char * str)
{
  while(*str) __putchar(*str++);

  return 0;
}

#ifdef __CC_ARM // keil

struct __FILE {
  uint32_t handle;
};

int fputc(int ch, FILE *f)
{
  __putchar(ch);
  return ch;
}

void _ttywrch(int ch)
{
  __putchar(ch);
}

#endif
