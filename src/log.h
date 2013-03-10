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

#define LOG_TAB(n) \
  do{\
    for(uint32_t run=0; run<(n); run++)\
      LOG_PRINTF("\t");\
  } while(0)

#define LOG(format, ...) LOG_PRINTF(LOG_MESSAGE format "%s", LOG_FUNC, LOG_LINE, __VA_ARGS__, CFG_PRINTF_NEWLINE);

#define LOG_ARR(array, size, format) \
  do{\
    for(uint32_t run=0; run<(size); run++)\
      LOG_PRINTF(format " ", (array)[run]);\
    LOG_PRINTF(CFG_PRINTF_NEWLINE);\
  }while(0)

#define LOG_STR(str) LOG("%s", str)

#endif
