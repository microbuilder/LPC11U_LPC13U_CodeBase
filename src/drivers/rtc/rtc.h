/**************************************************************************/
/*!
    @file rtc.h

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, K. Townsend (microBuilder.eu)
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
#ifndef _RTC_H_
#define _RTC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"
#include "errors.h"

#define RTC_LEAP_YEAR       (1)
#define RTC_NON_LEAP_YEAR   (0)
#define RTC_MAX_EPOCH_TIME  (2147483647UL)
#define RTC_MIN_EPOCH_YEAR  (70)
#define RTC_MAX_EPOCH_YEAR  (138)

/**************************************************************************/
/*!
 * @brief     RTC weekdays structure (weekdays are zero-based and start
              on Monday)
 */
/**************************************************************************/
typedef enum rtcWeekdays_en
{
    RTC_WEEKDAYS_MONDAY = 0,
    RTC_WEEKDAYS_TUESDAY,
    RTC_WEEKDAYS_WEDNESDAY,
    RTC_WEEKDAYS_THURSDAY,
    RTC_WEEKDAYS_FRIDAY,
    RTC_WEEKDAYS_SATURDAY,
    RTC_WEEKDAYS_SUNDAY
} rtcWeekdays_t;

/**************************************************************************/
/*!
 * @brief     RTC months structure (months are one-based)
 */
/**************************************************************************/
typedef enum rtcMonths_en
{
    RTC_MONTHS_JANUARY = 1,
    RTC_MONTHS_FEBRUARY,
    RTC_MONTHS_MARCH,
    RTC_MONTHS_APRIL,
    RTC_MONTHS_MAY,
    RTC_MONTHS_JUNE,
    RTC_MONTHS_JULY,
    RTC_MONTHS_AUGUST,
    RTC_MONTHS_SEPTEMBER,
    RTC_MONTHS_OCTOBER,
    RTC_MONTHS_NOVEMBER,
    RTC_MONTHS_DECEMBER
} rtcMonths_t;

/**************************************************************************/
/*!
 * @brief     RTC time structure
 * @warning:  To ensure that this structure always contains valid time in
 *            the EPOCH range, do not assign field values directly. Please
 *            call rtcAdd..., rtcCreateTime functions instead.
 */
/**************************************************************************/
typedef struct rtcTime_st
{
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t days;
    uint8_t weekdays;   /**< Zero-based, first day of week = Monday (use rtcWeekdays_t) */
    uint8_t months;     /**< One-based (use rtcMonths_t) */
    uint8_t years;      /**< Years since 1900 */
    int8_t timezone;    /**< Local time adjustment relative to GMT */
} rtcTime_t;

uint8_t   rtcDecToBCD ( uint8_t val );
uint8_t   rtcBCDToDec ( uint8_t val );
err_t   rtcCreateTime ( uint32_t year, rtcMonths_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, int8_t timezone, rtcTime_t *t );
err_t   rtcCreateTimeFromEpoch ( uint32_t epochTime, rtcTime_t *time );
err_t   rtcCreateTimeFromSecondsSince1980 ( uint32_t seconds, rtcTime_t *time );
err_t   rtcAssignWeekday ( rtcTime_t *t );
err_t   rtcAddSeconds ( rtcTime_t *t, int32_t s );
err_t   rtcAddMinutes ( rtcTime_t *t, int32_t m );
err_t   rtcAddHours ( rtcTime_t *t, int32_t h );
err_t   rtcAddDays ( rtcTime_t *t, int32_t d );
err_t   rtcAddMonths ( rtcTime_t *t, int32_t d );
err_t   rtcAddYears ( rtcTime_t *t, int32_t y );
err_t   rtcGetDifference ( rtcTime_t *t1, rtcTime_t *t2, int32_t *seconds );
err_t   rtcGetWeekday ( uint32_t year, rtcMonths_t month, uint8_t day, rtcWeekdays_t *weekDay );
err_t   rtcGetWeekNumber ( rtcTime_t *t, uint8_t *weekNumber );
int32_t   rtcGetDaysInYear ( int32_t year );
uint32_t  rtcGetEpochDate ( uint32_t year, rtcMonths_t month, uint8_t day );
uint32_t  rtcToEpochTime ( rtcTime_t *t );
uint32_t  rtcToSecondsSince1980 ( rtcTime_t *t );
bool      rtcIsLeapYear ( int32_t year );

#ifdef __cplusplus
}
#endif 

#endif
