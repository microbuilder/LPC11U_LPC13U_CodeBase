/**************************************************************************/
/*!
    @file rtc.c

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
#include <string.h>
#include "rtc.h"

/**************************************************************************/
/*!
 @brief Checks whether a date is valid.

 @note  Does not check the EPOCH range

 @param[in] year    1900-based year (ex. 113 = 2013)
 @param[in] month   month based on rtcMonths_t
 @param[in] day     day of month

 @return error code indicate whether the date is valid
 @retval ERROR_NONE the date is valid
 @retval ERROR_INVALIDPARAMETER the date is invalid
 */
/**************************************************************************/
static err_t rtcCheckValidDate(uint32_t year, uint8_t month, uint32_t day)
{
    /* Basic validation of values */
    if (year >= 1900)
    {
        year -= 1900;
    }
    if ((month > RTC_MONTHS_DECEMBER) || (month < RTC_MONTHS_JANUARY))
    {
        return ERROR_INVALIDPARAMETER;
    }
    if (month == 2)
    {
        if (rtcIsLeapYear(year))
        {
            if (day > 29)
            {
                return ERROR_INVALIDPARAMETER;
            }
        }
        else
        {
            if (day > 28)
            {
                return ERROR_INVALIDPARAMETER;
            }
        }
    }
    else if ((month == 4) || (month == 6) || (month == 9) || (month == 11))
    {
        if (day > 30)
        {
            return ERROR_INVALIDPARAMETER;
        }
    }
    else
    {
        if (day > 31)
        {
            return ERROR_INVALIDPARAMETER;
        }
    }
    return ERROR_NONE;
}

/**************************************************************************/
/*!
 @brief   Standard decimal to binary coded decimal
 @param   val[in]:  import number in Decimal
 @return  number in BCD
 */
/**************************************************************************/
uint8_t rtcDecToBCD(uint8_t val)
{
    return ((val / 10) << 4) | (val % 10);
}

/**************************************************************************/
/*!
 @brief   Binary coded decimal to standard decimal
 @param   val[in]: import number in BCD
 @return  number in Decimal
 */
/**************************************************************************/
uint8_t rtcBCDToDec(uint8_t val)
{
    return (val >> 4) * 10 + (val & 0x0F);
}

/**************************************************************************/
/*!
 @brief  Automatically calculates the weekday value, and takes into
         account if the supplied year is 'epoch' (8-bit value) or full
         year (ex. 2013)
 @param  year[in]:      year to assign to rtcTime_t
         month[in]:     month to assign to rtcTime_t
         day[in]:       day to assign to rtcTime_t
         hour[in]:      hour to assign to rtcTime_t
         minute[in]:    minute to assign to rtcTime_t
         second[in]:    second to assign to rtcTime_t
         timezone[in]:  timezone to assign to rtcTime_t
         t[out]:        rtcTime_t reference to manipulate
 @return errorCode
 */
/**************************************************************************/
err_t rtcCreateTime(uint32_t year, rtcMonths_t month, uint8_t day,
    uint8_t hour, uint8_t minute, uint8_t second, int8_t timezone, rtcTime_t *t)
{
    err_t errorCode;
    rtcTime_t newTime;

    /* Year is 8-bit value from 1900 */
    if (year >= 1900)
    {
        year -= 1900;
    }

    /* Keep with the limits of epoch time: 1 Jan 1970 to 19 Jan 2038 */
    if ((year < RTC_MIN_EPOCH_YEAR) || (year > RTC_MAX_EPOCH_YEAR))
    {
        return ERROR_RTC_OUTOFEPOCHRANGE;
    }

    /* Basic validation */
    errorCode = rtcCheckValidDate(year, (uint8_t)month, day);
    if (errorCode != ERROR_NONE)
    {
        return errorCode;
    }
    if ((hour > 23) || (minute > 59) || (second > 59))
    {
        return ERROR_INVALIDPARAMETER;
    }
    if ((timezone > 14) || (timezone < -12))
    {
        return ERROR_INVALIDPARAMETER;
    }

    newTime.years = year;
    newTime.months = month;
    newTime.days = day;
    newTime.hours = hour;
    newTime.minutes = minute;
    newTime.seconds = second;
    newTime.timezone = timezone;

    if (rtcToEpochTime(&newTime) > RTC_MAX_EPOCH_TIME)
    {
        return ERROR_RTC_OUTOFEPOCHRANGE;
    }
    rtcAssignWeekday(&newTime);
    memcpy(t, &newTime, sizeof(rtcTime_t));
    return ERROR_NONE;
}

/**************************************************************************/
/*!
 @brief  Convert Unix epoch time to RTC time
 @param  epochTime[in]: Unix epoch time
         time[out]:     rtcTime_t reference to manipulate
 @return errorCode
 */
/**************************************************************************/
err_t rtcCreateTimeFromEpoch(uint32_t epochTime, rtcTime_t *time)
{
    int32_t j, g, dg, c, dc, b, db, a, da, y, m, d;

    if (epochTime > RTC_MAX_EPOCH_TIME)
    {
        return ERROR_RTC_OUTOFEPOCHRANGE;
    }
    j = epochTime / 86400 + 2472632;
    g = j / 146097;
    dg = j % 146097;
    c = (dg / 36524 + 1) * 3 / 4;
    dc = dg - c * 36524;
    b = dc / 1461;
    db = dc % 1461;
    a = (db / 365 + 1) * 3 / 4;
    da = db - a * 365;
    y = g * 400 + c * 100 + b * 4 + a;
    m = (da * 5 + 308) / 153 - 2;
    d = da - (m + 4) * 153 / 5 + 122;
    time->years = y - 6700 + (m + 2) / 12;
    time->months = (m + 2) % 12 + 1;
    time->days = d + 1;
    time->weekdays = (j + 2) % 7;

    j = epochTime % 86400;
    time->hours = j / 3600;
    time->minutes = (j % 3600) / 60;
    time->seconds = j % 60;
    time->timezone = 0;
    return ERROR_NONE;
}

/**************************************************************************/
/*!
 @brief  Convert seconds since 1980 to RTC Time
 @param  seconds[in]: seconds since 1980
         time[out]:   rtcTime_t reference to populate with the equivalent
                      seconds since 1980
 @return errorCode
 */
/**************************************************************************/
err_t rtcCreateTimeFromSecondsSince1980(uint32_t seconds, rtcTime_t *time)
{
    return rtcCreateTimeFromEpoch(seconds + 315532800 /*epochTime of 1980 */,
        time);
}

/**************************************************************************/
/*!
 @brief   Calculates and updates the day of the week to weekDay field of
          the supplied rtcTime_t object based on the other values (year,
          day, etc.)
 @param   t[in/out]   rtcTime_t object containing the date to calculate
                      the weekday for (assumes .weekdays is empty)
 @return  errorCode
 */
/**************************************************************************/
err_t rtcAssignWeekday(rtcTime_t *t)
{
    uint32_t NrOfDay;
    NrOfDay = rtcGetEpochDate(t->years, t->months, t->days);
    t->weekdays = (NrOfDay + 3) % 7;
    return ERROR_NONE;
}

/**************************************************************************/
/*!
 @brief   Add seconds to rtcTime_t
 @param   t[in/out]:  rtcTime_t to manipulate
          s[in]:      Number of seconds to add (positive or negative)
 @return  errorCode
 */
/**************************************************************************/
err_t rtcAddSeconds(rtcTime_t *t, int32_t s)
{
    /* mls: We assume that t is a valid time in EPOCH range*/
    int32_t epochTime = rtcToEpochTime(t);
    epochTime += s;
    if (epochTime < 0)
    {
        return ERROR_RTC_OUTOFEPOCHRANGE;
    }
    rtcCreateTimeFromEpoch(epochTime, t);
    return ERROR_NONE;
}

/**************************************************************************/
/*!
 @brief   Add minutes to rtcTime_t
 @param   t[in/out]:  rtcTime_t to manipulate
          m[in]:      Number of minutes to add (positive or negative)
 @return  errorCode
 */
/**************************************************************************/
err_t rtcAddMinutes(rtcTime_t *t, int32_t m)
{
    if ((35791394 < m) || (-35791394 > m))
    {
        return ERROR_RTC_OUTOFEPOCHRANGE;
    }
    return rtcAddSeconds(t, m * 60);
}

/**************************************************************************/
/*!
 @brief   Add hours to rtcTime_t
 @param   t[in/out]:  rtcTime_t to manipulate
          h[in]:      import hours
 @return  errorCode
 */
/**************************************************************************/
err_t rtcAddHours(rtcTime_t *t, int32_t h)
{
    if ((596523 < h) || (-596523 > h))
    {
        return ERROR_RTC_OUTOFEPOCHRANGE;
    }
    return rtcAddSeconds(t, h * 3600);
}

/**************************************************************************/
/*!
 @brief   Add days to RTC
 @param   t[in/out]:  rtcTime_t to manipulate
          d[in]:      Number of days to add (positive or negative)
 @return  errorCode
 */
/**************************************************************************/
err_t rtcAddDays(rtcTime_t *t, int32_t d)
{
    if ((24855 < d) || (-24855 > d))
    {
        return ERROR_RTC_OUTOFEPOCHRANGE;
    }
    return rtcAddSeconds(t, d * 86400);
}

/**************************************************************************/
/*!
 @brief   Add month to RTC
 @param   t[in/out]:  rtcTime_t to manipulate
          m[in]:      Number of months to add (positive or negative)
 @return  errorCode
 */
/**************************************************************************/
err_t rtcAddMonths(rtcTime_t *t, int32_t m)
{
    rtcTime_t newTime;
    int32_t monthCount = (int32_t) t->years * 12 + t->months;
    monthCount += m;
    if ((monthCount < RTC_MIN_EPOCH_YEAR * 12 + 1) || (monthCount > RTC_MIN_EPOCH_YEAR
        * 12 + 1/*Jan 2038*/))
    {
        return ERROR_RTC_OUTOFEPOCHRANGE;
    }

    /* Check if new time is in EPOCH range */
    memcpy(&newTime, t, sizeof(rtcTime_t));
    newTime.years = (monthCount - 1) / 12;
    newTime.months = (monthCount - 1) % 12 + 1;
    if (rtcToEpochTime(&newTime) > RTC_MAX_EPOCH_TIME)
    {
        return ERROR_RTC_OUTOFEPOCHRANGE;
    }
    rtcAssignWeekday(&newTime);
    memcpy(t, &newTime, sizeof(rtcTime_t));
    return ERROR_NONE;
}

/**************************************************************************/
/*!
 @brief   Add year to RTC
 @param   t[in/out]:  rtcTime_t to manipulate
          y[in]:      Number of years to add (positive or negative)
 @return  errorCode
 */
/**************************************************************************/
err_t rtcAddYears(rtcTime_t *t, int32_t y)
{
    rtcTime_t newTime;
    /* Check if new time is in EPOCH range */
    memcpy(&newTime, t, sizeof(rtcTime_t));
    newTime.years += y;
    if ((newTime.years < RTC_MIN_EPOCH_YEAR) || (newTime.years > RTC_MAX_EPOCH_YEAR)
        || (rtcToEpochTime(&newTime) > RTC_MAX_EPOCH_TIME))
    {
        return ERROR_RTC_OUTOFEPOCHRANGE;
    }
    rtcAssignWeekday(&newTime);
    memcpy(t, &newTime, sizeof(rtcTime_t));
    return ERROR_NONE;
}


/**************************************************************************/
/*!
 @brief   Return the seconds between t1 - t2
 @param   t1[in]:       import RTC time t1
          t2[in]:       import RTC time t2
          seconds[out]: result (t1 - t2) in seconds
 */
/**************************************************************************/
err_t rtcGetDifference(rtcTime_t *t1, rtcTime_t *t2, int32_t *seconds)
{
    /* If t1 and t2 are valid time in EPOCH range, overflow won't occur */
    *seconds = rtcToEpochTime(t1) - rtcToEpochTime(t2);
    return ERROR_NONE;
}

/**************************************************************************/
/*!
 @brief   Calculates the day of the week for the provided data.  Works for
          any positive year, and checks for leap year if you attempt to
          provide 29 Feb on a non-leap-year.
 @param:  year[in]:     import year to Calculate
          month[in]:    import month to Calculate
          day[in]:      import day to Calculate
          weekDay[out]: The calculate day of the week for year, month, day
 @return  errorCode
 */
/**************************************************************************/
err_t rtcGetWeekday(uint32_t year, rtcMonths_t month, uint8_t day,
    rtcWeekdays_t *weekDay)
{
    /* ToDo: validate the supplied parameters */
    err_t errorCode = ERROR_NONE;
    uint32_t NrOfDay;
    if (year >= 1900)
    {
        year -= 1900;
    }
    errorCode = rtcCheckValidDate(year, (uint8_t)month, day);
    if (errorCode != ERROR_NONE)
    {
        return errorCode;
    }
    NrOfDay = rtcGetEpochDate(year, month, day);
    *weekDay = (NrOfDay + 3) % 7;

    return ERROR_NONE;
}

/**************************************************************************/
/*!
 @brief   Calculate the week number for the given date (9 January = week 2)
 @param   t[in]:            import RTC time to calculate week number for
          weekNumber[out]:  result after calculating
 @return  errorCode
 */
/**************************************************************************/
err_t rtcGetWeekNumber(rtcTime_t *t, uint8_t *weekNumber)
{
    uint32_t NrOfDay = rtcGetEpochDate(t->years, 1, 1);
    uint8_t nextMondayDay = 8 - NrOfDay;

    if ((t->months == 1) && (t->days < nextMondayDay))
    {
        *weekNumber = 1;
    }
    else
    {
        *weekNumber = (rtcGetEpochDate(t->years, t->months, t->days)
            - rtcGetEpochDate(t->years, 1, nextMondayDay)) / 7 + 2;
    }
    return ERROR_NONE;
}

/**************************************************************************/
/*!
 @brief   Number of days in the supplied year
 @param   year[in]: import year to take number of day
 @return  365 days or 366 days
 */
/**************************************************************************/
int32_t rtcGetDaysInYear(int32_t year)
{
    if (year >= 1900)
    {
        year -= 1900;
    }
    if (((year % 400) == 100) || (((year % 4) == 0) && ((year % 100) != 0)))
    {
        return 366;
    }
    return 365;
}

/**************************************************************************/
/*!
 @brief   Calculate number of days from Jan 1 1970
 @prama   year[in]:   import year to calculate
          month[in]:  import month to calculate
          day[in]:    import day to calculate
 @return  Number of day from Jan 1 1970
 */
/**************************************************************************/
uint32_t rtcGetEpochDate(uint32_t year, rtcMonths_t month, uint8_t day)
{
    if (year >= 1900)
    {
        year -= 1900;
    }
    if ((year < RTC_MIN_EPOCH_YEAR) || (year > RTC_MAX_EPOCH_YEAR))
    {
        return 0;
    }
    /* month must be cast to uint8_t or this fails on some platforms */
    uint8_t m = (uint8_t)(month & 0xFF);
    return (1461 * (year + 6700 + (m - 14) / 12)) / 4 + (367 *
        (m - 2 - 12 * ((m - 14) / 12))) / 12 -
        (3 * ((year + 6800 + (m - 14) / 12) / 100)) / 4 +
        day - 2472663;
}

/**************************************************************************/
/*!
 @brief   Convert the supplied rtcTime_t to the equivalent epoch time
 @param   t[in]: import RTC time
 @return  seconds since Jan 1 1970 to RTC time inputs
 */
/**************************************************************************/
uint32_t rtcToEpochTime(rtcTime_t *t)
{
    if (t->years < 70)
    {
        return 0;
    }
    return rtcGetEpochDate(t->years, t->months, t->days) * 86400 + t->hours
        * 3600 + t->minutes * 60 + t->seconds;
}

/**************************************************************************/
/*!
 @brief   Calculate seconds since Jan 1 1980 for the supplied rtcTime_t
 @param   t[in]: import RTC time
 @return  seconds since Jan 1 1980 to RTC time inputs
 */
/**************************************************************************/
uint32_t rtcToSecondsSince1980(rtcTime_t *t)
{
    if (t->years < 80)
    {
        return 0;
    }
    return rtcGetEpochDate(t->years, t->months, t->days) * 86400 + t->hours
        * 3600 + t->minutes * 60 + t->seconds - 315532800;
}


/**************************************************************************/
/*!
 @brief   Checks if the supplied year is a leap year or not
 @param   year[in]: import year to take number of day
 @return  RTC_LEAP_YEAR or RTC_NON_LEAP_YEAR
 */
/**************************************************************************/
bool rtcIsLeapYear(int32_t year)
{
    if (year >= 1900)
    {
        year -= 1900;
    }
    if (((year % 400) == 100) || (((year % 4) == 0) && ((year % 100) != 0)))
    {
        return RTC_LEAP_YEAR;
    }
    return RTC_NON_LEAP_YEAR;
}
