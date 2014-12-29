/**************************************************************************/
/*!
    @file     cmd_rtc_read.c
    @author   K. Townsend (microBuilder.eu)

    @brief    Reads the time on the RTC
    @ingroup  CLI

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, K. Townsend (microbuilder.eu)
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
#include <stdio.h>

#include "projectconfig.h"

#if defined(CFG_RTC)

#include "cli/commands.h"
#include "drivers/rtc/rtc.h"
#include "drivers/rtc/pcf2129/pcf2129.h"

/**************************************************************************/
/*!
    'cmd_rtc_read' command handler
*/
/**************************************************************************/
void cmd_rtc_read(uint8_t argc, char **argv)
{
  err_t error;
  rtcTime_t time;

  error = pcf2129ReadTime(&time);
  if (error)
  {
    printf("%s%s", STRING(LOCALISATION_TEXT_No_response_on_the_I2C_bus), CFG_PRINTF_NEWLINE);
    return;
  }

  printf("%04d/%02d/%02d%s", time.years+1900, time.months+1, time.days, CFG_PRINTF_NEWLINE);
  printf("%02d:%02d:%02d%s", time.hours, time.minutes, time.seconds, CFG_PRINTF_NEWLINE);
  printf("%u%s", rtcToEpochTime(&time), CFG_PRINTF_NEWLINE);
}

#endif /* CFG_RTC */
