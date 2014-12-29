/**************************************************************************/
/*!
    @file     cmd_rtc_write.c
    @author   K. Townsend (microBuilder.eu)

    @brief    Sets the time on the RTC
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
    'cmd_rtc_write' command handler
*/
/**************************************************************************/
void cmd_rtc_write(uint8_t argc, char **argv)
{
  err_t error;
  rtcTime_t time;
  int32_t year, month, day, hour, minute, second;

  getNumber(argv[0], &year);
  getNumber(argv[1], &month);
  getNumber(argv[2], &day);
  getNumber(argv[3], &hour);
  getNumber(argv[4], &minute);
  getNumber(argv[5], &second);

  /* Make sure values are valid */
  if ((year < 2000) || (year > 2038))
  {
    printf("%s%s", "Year must be between 2000 and 2023", CFG_PRINTF_NEWLINE);
    return;
  }
  if ((month < RTC_MONTHS_JANUARY) || (month > RTC_MONTHS_DECEMBER))
  {
    printf("%s%s", "Month must be between 1 and 12", CFG_PRINTF_NEWLINE);
    return;
  }
  if ((day < 1) || (day > 31))
  {
    printf("%s%s", "Day must be between 1 and 31", CFG_PRINTF_NEWLINE);
    return;
  }
  if ((hour < 0) || (hour > 23))
  {
    printf("%s%s", "Hour must be between 0 and 23", CFG_PRINTF_NEWLINE);
    return;
  }
  if ((minute < 0) || (minute > 59))
  {
    printf("%s%s", "Minute must be between 0 and 59", CFG_PRINTF_NEWLINE);
    return;
  }
  if ((second < 0) || (second > 59))
  {
    printf("%s%s", "Second must be between 0 and 59", CFG_PRINTF_NEWLINE);
    return;
  }

  /* Try to create a date */
  error = rtcCreateTime(year, month, day, hour, minute, second, 0, &time);
  if (error)
  {
    printf("%s%s", "Invalid timestamp", CFG_PRINTF_NEWLINE);
    return;
  }

  /* Write the time to the RTC */
  error = pcf2129SetTime(time);
  if (error)
  {
    printf("%s%s", STRING(LOCALISATION_TEXT_No_response_on_the_I2C_bus), CFG_PRINTF_NEWLINE);
    return;
  }
}

#endif /* CFG_RTC */
