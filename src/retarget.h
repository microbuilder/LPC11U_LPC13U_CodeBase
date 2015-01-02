/**************************************************************************/
/*!
    @file     retarget.h

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2014, Adafruit Industries (adafruit.com)
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
#ifndef _RETARGET_H_
#define _RETARGET_H_

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "projectconfig.h"
#include "core/uart/uart.h"
#include "core/usb/usbd.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Define UNISTD.h replacements */
#define STDIN_FILENO          0
#define STDOUT_FILENO         1
#define STDERR_FILENO         2
#define FILENO_NONSTD_START   3

#define STDIN_FILENAME        NULL
#define STDOUT_FILENAME       NULL
#define STDERR_FILENAME       NULL

#if defined CFG_PRINTF_UART

static inline int std_getc(void)
{
  return uartRxBufferDataPending() ? (int) uartRxBufferRead() : EOF;
}

static inline int std_putc(char ch)
{
  uartSendByte( (uint8_t) ch);
  return ch;
}


#elif defined CFG_PRINTF_USBCDC

static inline int std_getc(void)
{
  uint8_t ch;
  return usb_cdc_getc(&ch) ? ch : EOF;
}

static inline int std_putc(char ch)
{
  if ( !usb_cdc_isConnected() ) return EOF;

  usb_cdc_putc(ch);
  return ch;
}


#elif defined CFG_PRINTF_DEBUG

#else
  #error "no default target"
#endif

// Lookup table: (filename, write, read, peek)
//  - 1st one is standard target in stdin
//  - 2nd one is standard target out stdout
//  - 3rd one is starndard erro for stderr
//  - from 3rd is for non-standard target

#define RETARGET_LOOKUP_TABLE(XPAND)\
    XPAND(STDIN_FILENAME    , NULL     , std_getc, NULL)\
    XPAND(STDOUT_FILENAME   , std_putc , NULL    , NULL)\
    XPAND(STDERR_FILENAME   , std_putc , NULL    , NULL)\
        /* Non Standard Target */               \


#define RETARGET_XPAND_FILENAME(filename, target_putc, target_getc, target_peek) filename,

// lookup putc/get expansion
typedef struct
{
  int (* const write)(char);
  int (* const read)(void);
  int (* const peekAt) (uint16_t index);
}retarget_func_t;

#define RETARGET_XPAND_FUNCTABLE(filename, target_putc, target_getc, target_peek)\
  { target_putc, target_getc, target_peek },

#ifdef __cplusplus
 }
#endif

#endif /* _RETARGET_H_ */
