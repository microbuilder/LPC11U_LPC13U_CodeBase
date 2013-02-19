/**************************************************************************/
/*!
    @file     test_rtc.c
    @ingroup  Unit Tests

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
#include "unity_fixture.h"
#include "drivers/rtc/rtc.h"

static rtcTime_t rtctime;

TEST_GROUP(rtc);

TEST_SETUP(rtc)
{
  /* Clear rtctime */
  memset(&rtctime, 0, sizeof(rtcTime_t));
}

TEST_TEAR_DOWN(rtc)
{
}

/**************************************************************************/
/*
 Tests decimal to BCD conversion
 */
/**************************************************************************/
TEST(rtc, dec2bcd)
{
  TEST_ASSERT_EQUAL_HEX8(0x45, rtcDecToBCD(45));
}

/**************************************************************************/
/*
 Tests BCD to decimal conversion
 */
/**************************************************************************/
TEST(rtc, bcd2dec)
{
  TEST_ASSERT_EQUAL_UINT8(45, rtcBCDToDec(0x45));
}

/**************************************************************************/
/*
 Tests rtcAddSeconds
 */
/**************************************************************************/
TEST(rtc, AddSeconds)
{
  rtcTime_t t;
  rtcTime_t p;

  /* Wed 2 Jan 2013 @ 10:59:30 + 10 seconds = 10:59:30 */
  rtcCreateTime(2013, 1, 2, 10, 59, 30, 0, &t);
  rtcAddSeconds(&t, 10);
  TEST_ASSERT_EQUAL_UINT8(2013 - 1900, t.years);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_MONTHS_JANUARY, t.months);
  TEST_ASSERT_EQUAL_UINT8(2, t.days);
  TEST_ASSERT_EQUAL_UINT8(10, t.hours);
  TEST_ASSERT_EQUAL_UINT8(59, t.minutes);
  TEST_ASSERT_EQUAL_UINT8(40, t.seconds);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_WEEKDAYS_WEDNESDAY, t.weekdays);

  /* Wed 2 Jan 2013 @ 10:59:30 - 45 seconds = 10:58:45 */
  rtcCreateTime(2013, 1, 2, 10, 59, 30, 0, &p);
  rtcAddSeconds(&p, -45);
  TEST_ASSERT_EQUAL_UINT8(2013 - 1900, p.years);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_MONTHS_JANUARY, t.months);
  TEST_ASSERT_EQUAL_UINT8(2, p.days);
  TEST_ASSERT_EQUAL_UINT8(10, p.hours);
  TEST_ASSERT_EQUAL_UINT8(58, p.minutes);
  TEST_ASSERT_EQUAL_UINT8(45, p.seconds);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_WEEKDAYS_WEDNESDAY, p.weekdays);

  /* Epoch range tests */
  TEST_ASSERT_EQUAL_UINT32(ERROR_RTC_OUTOFEPOCHRANGE,
    rtcAddSeconds(&t, -2000000000));
  TEST_ASSERT_EQUAL_UINT32(ERROR_RTC_OUTOFEPOCHRANGE,
    rtcAddSeconds(&p, 2000000000));
  rtcCreateTime(2038, 1, 10, 10, 59, 30, 0, &p);
  TEST_ASSERT_EQUAL_UINT32(ERROR_RTC_OUTOFEPOCHRANGE,
    rtcAddSeconds(&p, 20000000));
  rtcCreateTime(1970, 1, 10, 10, 59, 30, 0, &p);
  TEST_ASSERT_EQUAL_UINT32(ERROR_RTC_OUTOFEPOCHRANGE,
    rtcAddSeconds(&p, -20000000));
}

/**************************************************************************/
/*
 Tests rtcAddMinutes
 */
/**************************************************************************/
TEST(rtc, AddMinutes)
{
  rtcTime_t t;

  /* Wed 2 Jan 2013 @ 10:59:30 + 18 minutes = 11:17:30 */
  rtcCreateTime(2013, 1, 2, 10, 59, 30, 0, &t);
  TEST_ASSERT_EQUAL_UINT32(ERROR_NONE, rtcAddMinutes(&t, 18));
  TEST_ASSERT_EQUAL_UINT8(2013 - 1900, t.years);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_MONTHS_JANUARY, t.months);
  TEST_ASSERT_EQUAL_UINT8(2, t.days);
  TEST_ASSERT_EQUAL_UINT8(11, t.hours);
  TEST_ASSERT_EQUAL_UINT8(17, t.minutes);
  TEST_ASSERT_EQUAL_UINT8(30, t.seconds);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_WEEKDAYS_WEDNESDAY, t.weekdays);

  /* Wed 2 Jan 2013 @ 10:59:30 - 75 minutes = 09:44:30 */
  rtcCreateTime(2013, 1, 2, 10, 59, 30, 0, &t);
  rtcAddMinutes(&t, -75);
  TEST_ASSERT_EQUAL_UINT8(2013 - 1900, t.years);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_MONTHS_JANUARY, t.months);
  TEST_ASSERT_EQUAL_UINT8(2, t.days);
  TEST_ASSERT_EQUAL_UINT8(9, t.hours);
  TEST_ASSERT_EQUAL_UINT8(44, t.minutes);
  TEST_ASSERT_EQUAL_UINT8(30, t.seconds);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_WEEKDAYS_WEDNESDAY, t.weekdays);

  /* Epoch range tests */
  TEST_ASSERT_EQUAL_UINT32(ERROR_RTC_OUTOFEPOCHRANGE,
    rtcAddMinutes(&t, -300000000));
  TEST_ASSERT_EQUAL_UINT32(ERROR_RTC_OUTOFEPOCHRANGE,
    rtcAddMinutes(&t, 300000000));
  rtcCreateTime(2038, 1, 1, 10, 59, 30, 0, &t);
  TEST_ASSERT_EQUAL_UINT32(ERROR_RTC_OUTOFEPOCHRANGE,
    rtcAddMinutes(&t, 300000));
  rtcCreateTime(1970, 1, 1, 10, 59, 30, 0, &t);
  TEST_ASSERT_EQUAL_UINT32(ERROR_RTC_OUTOFEPOCHRANGE,
    rtcAddMinutes(&t, -300000));
}

/**************************************************************************/
/*
 Tests rtcAddHours
 */
/**************************************************************************/
TEST(rtc, AddHours)
{
  rtcTime_t t;

  /* Wed 2 Jan 2013 @ 10:59:30 + 15 hours = Thu 3 Jan 2013 @ 01:59:30 */
  rtcCreateTime(2013, 1, 2, 10, 59, 30, 0, &t);
  TEST_ASSERT_EQUAL_UINT32(ERROR_NONE, rtcAddHours(&t, 15));
  TEST_ASSERT_EQUAL_UINT8(2013 - 1900, t.years);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_MONTHS_JANUARY, t.months);
  TEST_ASSERT_EQUAL_UINT8(3, t.days);
  TEST_ASSERT_EQUAL_UINT8(1, t.hours);
  TEST_ASSERT_EQUAL_UINT8(59, t.minutes);
  TEST_ASSERT_EQUAL_UINT8(30, t.seconds);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_WEEKDAYS_THURSDAY, t.weekdays);
  rtcCreateTime(2013, 1, 2, 10, 59, 30, 0, &t);

  /* Epoch range tests */
  TEST_ASSERT_EQUAL_UINT32(ERROR_RTC_OUTOFEPOCHRANGE,
    rtcAddHours(&t, -5000000));
  rtcCreateTime(2013, 1, 2, 10, 59, 30, 0, &t);
  TEST_ASSERT_EQUAL_UINT32(ERROR_RTC_OUTOFEPOCHRANGE,
    rtcAddHours(&t, 5000000));
  rtcCreateTime(2038, 1, 1, 10, 59, 30, 0, &t);
  TEST_ASSERT_EQUAL_UINT32(ERROR_RTC_OUTOFEPOCHRANGE, rtcAddHours(&t, 30000));
  rtcCreateTime(1970, 1, 2, 10, 59, 30, 0, &t);
  TEST_ASSERT_EQUAL_UINT32(ERROR_RTC_OUTOFEPOCHRANGE, rtcAddHours(&t, -30000));
}

/**************************************************************************/
/*
 Tests rtcAddDays
 */
/**************************************************************************/
TEST(rtc, AddDays)
{
  rtcTime_t t;

  /* Wed 2 Jan 2013 @ 10:59:30 + 30 days = Fri 1 Feb 2013 @ 10:59:30 */
  rtcCreateTime(2013, 1, 2, 10, 59, 30, 0, &t);
  TEST_ASSERT_EQUAL_UINT32(ERROR_NONE, rtcAddDays(&t, 30));
  TEST_ASSERT_EQUAL_UINT8(2013 - 1900, t.years);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_MONTHS_FEBRUARY, t.months);
  TEST_ASSERT_EQUAL_UINT8(1, t.days);
  TEST_ASSERT_EQUAL_UINT8(10, t.hours);
  TEST_ASSERT_EQUAL_UINT8(59, t.minutes);
  TEST_ASSERT_EQUAL_UINT8(30, t.seconds);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_WEEKDAYS_FRIDAY, t.weekdays);

  /* Zune test */
  rtcCreateTime(2008, RTC_MONTHS_JANUARY, 1, 0, 0, 0, 0, &t);
  TEST_ASSERT_EQUAL_UINT32(ERROR_NONE, rtcAddDays(&t, 366));
  TEST_ASSERT_EQUAL_UINT8(2009 - 1900, t.years);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_MONTHS_JANUARY, t.months);
  TEST_ASSERT_EQUAL_UINT8(1, t.days);
  TEST_ASSERT_EQUAL_UINT8(0, t.hours);
  TEST_ASSERT_EQUAL_UINT8(0, t.minutes);
  TEST_ASSERT_EQUAL_UINT8(0, t.seconds);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_WEEKDAYS_THURSDAY, t.weekdays);

  /* Epoch range tests */
  rtcCreateTime(2013, 1, 2, 10, 59, 30, 0, &t);
  TEST_ASSERT_EQUAL_UINT32(ERROR_RTC_OUTOFEPOCHRANGE,
    rtcAddDays(&t, -10000000));
  rtcCreateTime(2013, 1, 2, 10, 59, 30, 0, &t);
  TEST_ASSERT_EQUAL_UINT32(ERROR_RTC_OUTOFEPOCHRANGE,
    rtcAddDays(&t, 10000000));
  rtcCreateTime(2038, 1, 1, 10, 59, 30, 0, &t);
  TEST_ASSERT_EQUAL_UINT32(ERROR_RTC_OUTOFEPOCHRANGE, rtcAddDays(&t, 1000));
  rtcCreateTime(1970, 1, 1, 10, 59, 30, 0, &t);
  TEST_ASSERT_EQUAL_UINT32(ERROR_RTC_OUTOFEPOCHRANGE, rtcAddDays(&t, -1000));
}

/**************************************************************************/
/*
 Tests rtcAddYears
 */
/**************************************************************************/
TEST(rtc, AddYears)
{
  rtcTime_t t;

  /* Wed 2 Jan 2013 @ 10:59:30 + 2 years = Fri 2 Jan 2015 @ 10:59:30 */
  rtcCreateTime(2013, 1, 2, 10, 59, 30, 0, &t);
  TEST_ASSERT_EQUAL_UINT32(ERROR_NONE, rtcAddYears(&t, 2));
  TEST_ASSERT_EQUAL_UINT8(2015 - 1900, t.years);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_MONTHS_JANUARY, t.months);
  TEST_ASSERT_EQUAL_UINT8(2, t.days);
  TEST_ASSERT_EQUAL_UINT8(10, t.hours);
  TEST_ASSERT_EQUAL_UINT8(59, t.minutes);
  TEST_ASSERT_EQUAL_UINT8(30, t.seconds);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_WEEKDAYS_FRIDAY, t.weekdays);

  /* Check for epoch overflow (valid range = 1 Jan 1970 to 19 Jan 2038) */
  TEST_ASSERT_EQUAL_UINT32(ERROR_RTC_OUTOFEPOCHRANGE, rtcAddYears(&t, 40));
  TEST_ASSERT_EQUAL_UINT32(ERROR_RTC_OUTOFEPOCHRANGE, rtcAddYears(&t, -50));
}

/**************************************************************************/
/*
 Tests rtcCreateTime
 */
/**************************************************************************/
TEST(rtc, CreateTime)
{
  rtcTime_t t;

  /* Create a valid time within epoch range => return ERROR_NONE */
  TEST_ASSERT_EQUAL_UINT32(ERROR_NONE,
  rtcCreateTime(1970, RTC_MONTHS_JANUARY, 1, 0, 0, 0, 0, &t));
  TEST_ASSERT_EQUAL_UINT32(ERROR_NONE,
  rtcCreateTime(2038, RTC_MONTHS_JANUARY, 19, 3, 14, 7, 0, &t));

  rtcCreateTime(2013, RTC_MONTHS_JANUARY, 1, 10, 0, 0, 0, &t);
  TEST_ASSERT_EQUAL_UINT8(2013 - 1900, t.years);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_MONTHS_JANUARY, t.months);
  TEST_ASSERT_EQUAL_UINT8(1, t.days);
  TEST_ASSERT_EQUAL_UINT8(10, t.hours);
  TEST_ASSERT_EQUAL_UINT8(0, t.minutes);
  TEST_ASSERT_EQUAL_UINT8(0, t.seconds);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_WEEKDAYS_TUESDAY, t.weekdays);

  /* Check for epoch overflow (valid range = 1 Jan 1970 to 19 Jan 2038) */
  TEST_ASSERT_EQUAL_UINT32(ERROR_RTC_OUTOFEPOCHRANGE,
    rtcCreateTime(2038, RTC_MONTHS_JANUARY, 19, 3, 14, 8, 0, &t));
  TEST_ASSERT_EQUAL_UINT32(ERROR_RTC_OUTOFEPOCHRANGE,
    rtcCreateTime(1969, RTC_MONTHS_DECEMBER, 31, 23, 59, 59, 0, &t));

  /* Create an invalid time (year < 2038) => return ERROR_RTC_OUTOFEPOCHRANGE */
  TEST_ASSERT_EQUAL_UINT32(ERROR_RTC_OUTOFEPOCHRANGE,
    rtcCreateTime(2050, RTC_MONTHS_JANUARY, 20, 10, 0, 0, 0, &t));

  /* Create an invalid time (hours > 24) => return ERROR_INVALIDPARAMETER */
  TEST_ASSERT_EQUAL_UINT32(ERROR_INVALIDPARAMETER,
    rtcCreateTime(2000, RTC_MONTHS_JANUARY, 2, 30, 0, 0, 0, &t));

  /* Create an invalid time (minute > 60) => return ERROR_INVALIDPARAMETER */
  TEST_ASSERT_EQUAL_UINT32(ERROR_INVALIDPARAMETER,
    rtcCreateTime(2000, RTC_MONTHS_JANUARY, 2, 10, 70, 0, 0, &t));

  /* Create an invalid time (second > 60) => return ERROR_INVALIDPARAMETER */
  TEST_ASSERT_EQUAL_UINT32(ERROR_INVALIDPARAMETER,
    rtcCreateTime(2000, RTC_MONTHS_JANUARY, 2, 10, 0, 70, 0, &t));
}

/**************************************************************************/
/*
 Tests rtcCreateTimeFromEpoch
 */
/**************************************************************************/
TEST(rtc, CreateTimeFromEpoch)
{
  rtcTime_t t, p;

  /* Tue, 01 Jan 2013 03:00:00 */
  rtcCreateTimeFromEpoch(1357009200, &t);
  TEST_ASSERT_EQUAL_UINT8(2013 - 1900, t.years);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_MONTHS_JANUARY, t.months);
  TEST_ASSERT_EQUAL_UINT8(1, t.days);
  TEST_ASSERT_EQUAL_UINT8(3, t.hours);
  TEST_ASSERT_EQUAL_UINT8(0, t.minutes);
  TEST_ASSERT_EQUAL_UINT8(0, t.seconds);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_WEEKDAYS_TUESDAY, t.weekdays);

  /* Create an out of range value => return ERROR_RTC_OUTOFEPOCHRANGE */
  TEST_ASSERT_EQUAL_UINT32(ERROR_RTC_OUTOFEPOCHRANGE,
    rtcCreateTimeFromEpoch(3000000000, &p));

  /* Tue, 19 Jan 2038 03:14:07 GMT */
  TEST_ASSERT_EQUAL_UINT32(ERROR_NONE, rtcCreateTimeFromEpoch(2147483647, &p));
  TEST_ASSERT_EQUAL_UINT8(2038 - 1900, p.years);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_MONTHS_JANUARY, p.months);
  TEST_ASSERT_EQUAL_UINT8(19, p.days);
  TEST_ASSERT_EQUAL_UINT8(3, p.hours);
  TEST_ASSERT_EQUAL_UINT8(14, p.minutes);
  TEST_ASSERT_EQUAL_UINT8(7, p.seconds);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_WEEKDAYS_TUESDAY, p.weekdays);

  /* Thu, 01 Jan 1970 00:00:00 GMT */
  TEST_ASSERT_EQUAL_UINT32(ERROR_NONE, rtcCreateTimeFromEpoch(0, &p));
  TEST_ASSERT_EQUAL_UINT8(1970 - 1900, p.years);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_MONTHS_JANUARY, p.months);
  TEST_ASSERT_EQUAL_UINT8(1, p.days);
  TEST_ASSERT_EQUAL_UINT8(0, p.hours);
  TEST_ASSERT_EQUAL_UINT8(0, p.minutes);
  TEST_ASSERT_EQUAL_UINT8(0, p.seconds);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_WEEKDAYS_THURSDAY, p.weekdays);

  TEST_ASSERT_EQUAL_UINT32(ERROR_RTC_OUTOFEPOCHRANGE,
    rtcCreateTimeFromEpoch(2147483648, &p));
}

/**************************************************************************/
/*
 Tests rtcCreateTimeFromSecondsSince1980
 */
/**************************************************************************/
TEST(rtc, CreateTimeFrom1980)
{
  rtcTime_t t, p;

  /* Tue, 01 Jan 2013 03:00:00 */
  TEST_ASSERT_EQUAL_UINT32(ERROR_NONE,
    rtcCreateTimeFromSecondsSince1980(1357009200 - 315532800, &t));
  TEST_ASSERT_EQUAL_UINT8(2013 - 1900, t.years);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_MONTHS_JANUARY, t.months);
  TEST_ASSERT_EQUAL_UINT8(1, t.days);
  TEST_ASSERT_EQUAL_UINT8(3, t.hours);
  TEST_ASSERT_EQUAL_UINT8(0, t.minutes);
  TEST_ASSERT_EQUAL_UINT8(0, t.seconds);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_WEEKDAYS_TUESDAY, t.weekdays);

  /* Check out of range value => return ERROR_RTC_OUTOFEPOCHRANGE */
  TEST_ASSERT_EQUAL_UINT32(ERROR_RTC_OUTOFEPOCHRANGE,
    rtcCreateTimeFromSecondsSince1980(3000000000, &t));

  /* Tue, 19 Jan 2038 03:14:07 GMT */
  TEST_ASSERT_EQUAL_UINT32(ERROR_NONE,
    rtcCreateTimeFromSecondsSince1980(2147483647 - 315532800, &p));
  TEST_ASSERT_EQUAL_UINT8(2038 - 1900, p.years);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_MONTHS_JANUARY, p.months);
  TEST_ASSERT_EQUAL_UINT8(19, p.days);
  TEST_ASSERT_EQUAL_UINT8(3, p.hours);
  TEST_ASSERT_EQUAL_UINT8(14, p.minutes);
  TEST_ASSERT_EQUAL_UINT8(7, p.seconds);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_WEEKDAYS_TUESDAY, p.weekdays);

  /* Thu, 01 Jan 1980 00:00:00 GMT */
  TEST_ASSERT_EQUAL_UINT32(ERROR_NONE,
    rtcCreateTimeFromSecondsSince1980(0, &p));
  TEST_ASSERT_EQUAL_UINT8(1980 - 1900, p.years);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_MONTHS_JANUARY, p.months);
  TEST_ASSERT_EQUAL_UINT8(1, p.days);
  TEST_ASSERT_EQUAL_UINT8(0, p.hours);
  TEST_ASSERT_EQUAL_UINT8(0, p.minutes);
  TEST_ASSERT_EQUAL_UINT8(0, p.seconds);
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_WEEKDAYS_TUESDAY, p.weekdays);

  TEST_ASSERT_EQUAL_UINT32(ERROR_RTC_OUTOFEPOCHRANGE,
    rtcCreateTimeFromSecondsSince1980(2147483647 - 315532800 + 1, &p));
}

/**************************************************************************/
/*
 Tests rtcGetDifference
 */
/**************************************************************************/
TEST(rtc, Difference)
{
  rtcTime_t t;
  rtcTime_t p;
  int32_t secondResult;

  /* Direct comparison tests */
  rtcCreateTimeFromEpoch(2147483647, &t);
  rtcCreateTimeFromEpoch(2147483647, &p);
  rtcGetDifference(&t, &p, &secondResult);
  TEST_ASSERT_EQUAL_UINT32(0, secondResult);
  rtcCreateTimeFromEpoch(0, &t);
  rtcCreateTimeFromEpoch(0, &p);
  rtcGetDifference(&t, &p, &secondResult);
  TEST_ASSERT_EQUAL_UINT32(0, secondResult);

  /* Mon, 31 Dec 2012 13:06:40 GMT */
  rtcCreateTimeFromEpoch(0, &t);
  rtcCreateTimeFromEpoch(2147483647, &p);
  rtcGetDifference(&t, &p, &secondResult);
  TEST_ASSERT_EQUAL_UINT32(-2147483647, secondResult);

  /* Mon, 31 Dec 2012 13:06:40 GMT */
  rtcCreateTimeFromEpoch(2147483647, &t);
  rtcCreateTimeFromEpoch(0, &p);
  rtcGetDifference(&t, &p, &secondResult);
  TEST_ASSERT_EQUAL_UINT32(2147483647, secondResult);

  /* Mon, 31 Dec 2012 13:06:40 GMT */
  rtcCreateTimeFromEpoch(2147483647, &t);
  rtcCreateTimeFromEpoch(2147483647 - 50000, &p);
  rtcGetDifference(&t, &p, &secondResult);
  TEST_ASSERT_EQUAL_UINT32(50000, secondResult);

  /* Mon, 31 Dec 2012 13:06:40 GMT */
  rtcCreateTimeFromEpoch(2147483647 - 50000, &t);
  rtcCreateTimeFromEpoch(2147483647, &p);
  rtcGetDifference(&t, &p, &secondResult);
  TEST_ASSERT_EQUAL_UINT32(-50000, secondResult);
}

/**************************************************************************/
/*
 Tests rtcGetWeekday
 */
/**************************************************************************/
TEST(rtc, GetWeekday)
{
  rtcWeekdays_t weekday;

  /* Check for an invalid date (no Feb 29 in 2013) */
  TEST_ASSERT_EQUAL_HEX8(ERROR_INVALIDPARAMETER,
    rtcGetWeekday(2013, RTC_MONTHS_FEBRUARY, 29, &weekday));

  TEST_ASSERT_EQUAL_HEX8(ERROR_INVALIDPARAMETER,
    rtcGetWeekday(2013, 14, 29, &weekday));

  TEST_ASSERT_EQUAL_HEX8(ERROR_INVALIDPARAMETER,
    rtcGetWeekday(2013, RTC_MONTHS_FEBRUARY, 40, &weekday));

  TEST_ASSERT_EQUAL_HEX8(0x0,
    rtcGetWeekday(2013, RTC_MONTHS_JANUARY, 1, &weekday));
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_WEEKDAYS_TUESDAY, weekday);

  TEST_ASSERT_EQUAL_HEX8(0x0,
    rtcGetWeekday(2013, RTC_MONTHS_FEBRUARY, 28, &weekday));
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_WEEKDAYS_THURSDAY, weekday);

  TEST_ASSERT_EQUAL_HEX8(0x0,
    rtcGetWeekday(2000, RTC_MONTHS_FEBRUARY, 29, &weekday));
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_WEEKDAYS_TUESDAY, weekday);
}

/**************************************************************************/
/*
 Tests rtcGetWeekdayFromRTCTime
 */
/**************************************************************************/
TEST(rtc, AssignWeekday)
{
  rtcTime_t t;

  /*Tue, 01 Jan 2013 03:00:00 GMT */
  rtcCreateTimeFromEpoch(1357009200, &t);
  TEST_ASSERT_EQUAL_HEX8(0x0, rtcAssignWeekday(&t));
  TEST_ASSERT_EQUAL_UINT8((uint8_t) RTC_WEEKDAYS_TUESDAY, t.weekdays);
}

/**************************************************************************/
/*
 Tests rtcGetWeekNumber
 */
/**************************************************************************/
TEST(rtc, GetWeekNumber)
{
  rtcTime_t t;
  uint8_t WeekNumResult;

  /* Tue, 01 Jan 2013 03:00:00 GMT */
  rtcCreateTimeFromEpoch(1357009200, &t);
  TEST_ASSERT_EQUAL_HEX8(0x0, rtcGetWeekNumber(&t, &WeekNumResult));
  TEST_ASSERT_EQUAL_HEX8(0x1, WeekNumResult);
}

/**************************************************************************/
/*
 Tests rtcToSecondsSince1980
 */
/**************************************************************************/
TEST(rtc, ToSecondsSince1980)
{
  rtcTime_t t;

  /* Tue, 01 Jan 2013 03:00:00 GMT */
  rtcCreateTime(2013, RTC_MONTHS_JANUARY, 1, 3, 0, 0, 0, &t);
  TEST_ASSERT_EQUAL_UINT32(1357009200 - 315532800, rtcToSecondsSince1980(&t));

  rtcCreateTime(2038, RTC_MONTHS_JANUARY, 19, 3, 14, 7, 0, &t);
  TEST_ASSERT_EQUAL_UINT32(2147483647 - 315532800, rtcToSecondsSince1980(&t));

  rtcCreateTime(1980, RTC_MONTHS_JANUARY, 1, 0, 0, 0, 0, &t);
  TEST_ASSERT_EQUAL_UINT32(0, rtcToSecondsSince1980(&t));
}

/**************************************************************************/
/*
 Tests rtcToEpochTime
 */
/**************************************************************************/
TEST(rtc, ToEpochTime)
{
  rtcTime_t t;

  rtcCreateTime(2013, RTC_MONTHS_JANUARY, 1, 3, 0, 0, 0, &t);
  TEST_ASSERT_EQUAL_UINT32(1357009200, rtcToEpochTime(&t));

  rtcCreateTime(1970, RTC_MONTHS_JANUARY, 1, 0, 0, 0, 0, &t);
  TEST_ASSERT_EQUAL_UINT32(0, rtcToEpochTime(&t));

  rtcCreateTime(2038, RTC_MONTHS_JANUARY, 19, 3, 14, 7, 0, &t);
  TEST_ASSERT_EQUAL_UINT32(2147483647, rtcToEpochTime(&t));
}

/**************************************************************************/
/*
 Tests rtcIsLeapYear
 */
/**************************************************************************/
TEST(rtc, IsLeapYear)
{
  TEST_ASSERT_TRUE(rtcIsLeapYear(2012));
  TEST_ASSERT_FALSE(rtcIsLeapYear(2013));
  TEST_ASSERT_TRUE(rtcIsLeapYear(2000));
}

/**************************************************************************/
/*
 Tests rtcGetDaysInYear
 */
/**************************************************************************/
TEST(rtc, GetDaysInYear)
{
  TEST_ASSERT_EQUAL_INT(366, rtcGetDaysInYear(2000));
  TEST_ASSERT_EQUAL_INT(365, rtcGetDaysInYear(2013));
  TEST_ASSERT_EQUAL_INT(366, rtcGetDaysInYear(2012));
}

TEST_GROUP_RUNNER(rtc)
{
  RUN_TEST_CASE(rtc, dec2bcd);
  RUN_TEST_CASE(rtc, bcd2dec);
  RUN_TEST_CASE(rtc, AddSeconds);
  RUN_TEST_CASE(rtc, AddMinutes);
  RUN_TEST_CASE(rtc, AddHours);
  RUN_TEST_CASE(rtc, AddDays);
  RUN_TEST_CASE(rtc, AddYears);
  RUN_TEST_CASE(rtc, CreateTime);
  RUN_TEST_CASE(rtc, CreateTimeFromEpoch);
  RUN_TEST_CASE(rtc, CreateTimeFrom1980);
  RUN_TEST_CASE(rtc, Difference);
  RUN_TEST_CASE(rtc, GetWeekday);
  RUN_TEST_CASE(rtc, AssignWeekday);
  RUN_TEST_CASE(rtc, GetWeekNumber);
  RUN_TEST_CASE(rtc, ToSecondsSince1980);
  RUN_TEST_CASE(rtc, ToEpochTime);
  RUN_TEST_CASE(rtc, IsLeapYear);
  RUN_TEST_CASE(rtc, GetDaysInYear);
}
