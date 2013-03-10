/**************************************************************************/
/*!
    @file     log.h
    @author   K. Townsend (microBuilder.eu)

    @brief    Various LOG macros to simplify data logging when debugging.
    @ingroup  Errors

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
#ifndef _LOG_H_
#define _LOG_H_

#include "projectconfig.h"

// #define LOG_ENABLE

/*! Compiler specific macro returning a string containing the current line number */
#define LOG_LINE __LINE__
/*! Compiler specific macro returning a string containing the current function */
#define LOG_FUNC __func__

#ifdef LOG_ENABLE
  #define LOG_PRINTF(...)      printf(__VA_ARGS__)
#else
  #define LOG_PRINTF(...)
#endif

#define LOG_MESSAGE "Log: %s: line %d: "

/**************************************************************************/
/*!
    @brief  Inserts the specified number of tabs in the data log
*/
/**************************************************************************/
#define LOG_TAB(n) \
  do{\
    for(uint32_t run=0; run<(n); run++)\
      LOG_PRINTF("\t");\
  } while(0)

/**************************************************************************/
/*!
    @brief  Logs the specified message, including the function name an
            the line where the function is present

    @param  format  The entire 'printf' parameter set, including optional
                    parameters if values like %s, %d are used.

    @code
    LOG("HID In report" CFG_PRINTF_NEWLINE
        "\tStatus   = %d - %s" CFG_PRINTF_NEWLINE
        "\tCount    = %d" CFG_PRINTF_NEWLINE
        "\tSequence = %d",
        in_report->Status, REPORT_STATUS_STR(in_report->Status), in_report->Count, in_report->Sequence);
    LOG_TAB(1);
    LOG_ARR(in_report->ReadData, 15, "%08X");
    @endcode
*/
/**************************************************************************/
#define LOG(format, ...) LOG_PRINTF(LOG_MESSAGE format "%s", LOG_FUNC, LOG_LINE, __VA_ARGS__, CFG_PRINTF_NEWLINE);

/**************************************************************************/
/*!
    @brief  Sends the array contents to the data log

    @param  array   Pointer the the array in memory
    @param  size    Length of the array
    @param  format  Format to use when displaying each array item via printf
                    (ex. "%02X ", "%d", "0x%08X", etc.)

    @code
    LOG_ARR(in_report->ReadData, 15, "%08X");
    @endcode
*/
/**************************************************************************/
#define LOG_ARR(array, size, format) \
  do{\
    for(uint32_t run=0; run<(size); run++)\
      LOG_PRINTF(format " ", (array)[run]);\
    LOG_PRINTF(CFG_PRINTF_NEWLINE);\
  } while(0)

#define LOG_STR(str) LOG("%s", str)

#endif
