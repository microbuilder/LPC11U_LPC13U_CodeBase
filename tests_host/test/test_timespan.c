/**************************************************************************/
/*!
    @file     test_timespan.c
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
#include "unity.h"
#include "timespan.h"

static timespan_t timespan;

uint32_t SystemCoreClock = 12000000; // overshadow the variable used to determine core clock

void setUp(void)
{
  memset(&timespan, 0, sizeof(timespan_t));
}

void tearDown(void)
{
}

/**************************************************************************/
/*
    Tests timespanCreate
 */
/**************************************************************************/
void test_timespan_create(void)
{
  // 90087581005024 ns = 1 day, 1 hour, 1 minute and 27.58100524 seconds
  TEST_ASSERT_TRUE(ERROR_NONE == timespanCreate(90087581005024, &timespan));
  TEST_ASSERT_EQUAL_INT(1, timespan.days);
  TEST_ASSERT_EQUAL_INT(1, timespan.hours);
  TEST_ASSERT_EQUAL_INT(1, timespan.minutes);
  TEST_ASSERT_EQUAL_INT(27, timespan.seconds);
  TEST_ASSERT_EQUAL_INT(581, timespan.milliseconds);
  TEST_ASSERT_EQUAL_INT(5, timespan.microseconds);
  TEST_ASSERT_EQUAL_INT(24, timespan.nanoseconds);
  memset(&timespan, 0, sizeof(timespan_t));

  /* Test upper limit */
  TEST_ASSERT_TRUE(ERROR_NONE == timespanCreate(TIMESPAN_MAXNANOSECONDS, &timespan));
  TEST_ASSERT_EQUAL_INT(106751, timespan.days);
  TEST_ASSERT_EQUAL_INT(23, timespan.hours);
  TEST_ASSERT_EQUAL_INT(47, timespan.minutes);
  TEST_ASSERT_EQUAL_INT(16, timespan.seconds);
  TEST_ASSERT_EQUAL_INT(854, timespan.milliseconds);
  TEST_ASSERT_EQUAL_INT(775, timespan.microseconds);
  TEST_ASSERT_EQUAL_INT(807, timespan.nanoseconds);
  TEST_ASSERT_TRUE(TIMESPAN_MAXNANOSECONDS == timespan.__ticks);
  memset(&timespan, 0, sizeof(timespan_t));

  /* Test lower limit (negative values) */
  TEST_ASSERT_TRUE(ERROR_NONE == timespanCreate(TIMESPAN_MINNANOSECONDS, &timespan));
  TEST_ASSERT_EQUAL_INT(-106751, timespan.days);
  TEST_ASSERT_EQUAL_INT(-23, timespan.hours);
  TEST_ASSERT_EQUAL_INT(-47, timespan.minutes);
  TEST_ASSERT_EQUAL_INT(-16, timespan.seconds);
  TEST_ASSERT_EQUAL_INT(-854, timespan.milliseconds);
  TEST_ASSERT_EQUAL_INT(-775, timespan.microseconds);
  TEST_ASSERT_EQUAL_INT(-807, timespan.nanoseconds);
  TEST_ASSERT_TRUE(TIMESPAN_MINNANOSECONDS == timespan.__ticks);
  memset(&timespan, 0, sizeof(timespan_t));
}

/**************************************************************************/
/*
    Tests timespanCreateExplicit
 */
/**************************************************************************/
void test_timespan_createExplicit(void)
{
  /* Calculate ticks for this timespan ... should be 90087581005024 */
  TEST_ASSERT_TRUE(ERROR_NONE ==
    timespanCreateExplicit(1, 1, 1, 27, 581, 5, 24, &timespan));
  TEST_ASSERT_TRUE(timespan.__ticks == 90087581005024);

  /* Check negative values ... this should equal -1 minute */
  TEST_ASSERT_TRUE(ERROR_NONE ==
    timespanCreateExplicit(0, -1, 59, 0, 0, 0, 0, &timespan));
  TEST_ASSERT_EQUAL_INT(-1, timespan.minutes);
  TEST_ASSERT_TRUE(timespan.__ticks == -60000000000);

  /* Check for days overflow */
  TEST_ASSERT_TRUE(ERROR_TIMESPAN_OUTOFRANGE ==
    timespanCreateExplicit((int32_t)TIMESPAN_MAXDAYS + 1, 0, 0, 0, 0, 0, 0, &timespan));

  /* Check for days underflow */
  TEST_ASSERT_TRUE(ERROR_TIMESPAN_OUTOFRANGE ==
    timespanCreateExplicit((int32_t)TIMESPAN_MINDAYS - 1, 0, 0, 0, 0, 0, 0, &timespan));

  /* Min/max timespan is +/-106751 days, 23 hours, 47 minutes and 16.854775807 seconds */
  TEST_ASSERT_TRUE(ERROR_TIMESPAN_OUTOFRANGE ==
    timespanCreateExplicit((int32_t)TIMESPAN_MAXDAYS, 23, 47, 16, 854, 775, 808, &timespan));

  /* Check negative values (microseconds exceeds limit but negative nanoseconds compensates) */
  TEST_ASSERT_TRUE(ERROR_NONE ==
    timespanCreateExplicit((int32_t)TIMESPAN_MAXDAYS, 23, 47, 16, 854, 776, -808, &timespan));
}

/**************************************************************************/
/*
    Tests timespanToHours
 */
/**************************************************************************/
void test_timespan_toHours(void)
{
  /* Check upper limit */
  timespanCreate(TIMESPAN_MAXNANOSECONDS, &timespan);
  TEST_ASSERT_EQUAL_INT((int32_t)TIMESPAN_MAXHOURS, timespanToHours(&timespan));
  memset(&timespan, 0, sizeof(timespan_t));

  /* Check lower limit */
  timespanCreate(TIMESPAN_MINNANOSECONDS, &timespan);
  TEST_ASSERT_EQUAL_INT((int32_t)TIMESPAN_MINHOURS, timespanToHours(&timespan));
  memset(&timespan, 0, sizeof(timespan_t));

  /* One hour */
  timespanCreateExplicit(0, 1, 0, 0, 0, 0, 0, &timespan);
  TEST_ASSERT_EQUAL_INT(1, timespanToHours(&timespan));
  memset(&timespan, 0, sizeof(timespan_t));

  /* 25 hours */
  timespanCreateExplicit(1, 1, 0, 0, 0, 0, 0, &timespan);
  TEST_ASSERT_EQUAL_INT(25, timespanToHours(&timespan));
  memset(&timespan, 0, sizeof(timespan_t));
}

/**************************************************************************/
/*
    Tests timespanToMinutes
 */
/**************************************************************************/
void test_timespan_toMinutes(void)
{
  /* Check upper limit */
  timespanCreate(TIMESPAN_MAXNANOSECONDS, &timespan);
  TEST_ASSERT_EQUAL_INT((int32_t)TIMESPAN_MAXMINUTES, timespanToMinutes(&timespan));
  memset(&timespan, 0, sizeof(timespan_t));

  /* Check lower limit */
  timespanCreate(TIMESPAN_MINNANOSECONDS, &timespan);
  TEST_ASSERT_EQUAL_INT(TIMESPAN_MINMINUTES, timespanToMinutes(&timespan));
  memset(&timespan, 0, sizeof(timespan_t));

  /* One minute */
  timespanCreateExplicit(0, 0, 1, 0, 0, 0, 0, &timespan);
  TEST_ASSERT_EQUAL_INT(1, timespanToMinutes(&timespan));
  memset(&timespan, 0, sizeof(timespan_t));

  /* 61 minutes */
  timespanCreateExplicit(0, 1, 1, 0, 0, 0, 0, &timespan);
  TEST_ASSERT_EQUAL_INT(61, timespanToMinutes(&timespan));
  memset(&timespan, 0, sizeof(timespan_t));
}

/**************************************************************************/
/*
    Tests timespanToSeconds
 */
/**************************************************************************/
void test_timespan_toSeconds(void)
{
  /* Check upper limit */
  timespanCreate(TIMESPAN_MAXNANOSECONDS, &timespan);
  TEST_ASSERT_EQUAL_INT((int32_t)TIMESPAN_MAXSECONDS, timespanToSeconds(&timespan));
  memset(&timespan, 0, sizeof(timespan_t));

  /* Check lower limit */
  timespanCreate(TIMESPAN_MINNANOSECONDS, &timespan);
  TEST_ASSERT_EQUAL_INT(TIMESPAN_MINSECONDS, timespanToSeconds(&timespan));
  memset(&timespan, 0, sizeof(timespan_t));

  /* One second */
  timespanCreateExplicit(0, 0, 0, 1, 0, 0, 0, &timespan);
  TEST_ASSERT_EQUAL_INT(1, timespanToSeconds(&timespan));
  memset(&timespan, 0, sizeof(timespan_t));

  /* 61 seconds */
  timespanCreateExplicit(0, 0, 1, 1, 0, 0, 0, &timespan);
  TEST_ASSERT_EQUAL_INT(61, timespanToSeconds(&timespan));
  memset(&timespan, 0, sizeof(timespan_t));
}

/**************************************************************************/
/*
    Tests timespanToMilliseconds
 */
/**************************************************************************/
void test_timespan_toMilliseconds(void)
{
  /* Check upper limit */
  timespanCreate(TIMESPAN_MAXNANOSECONDS, &timespan);
  TEST_ASSERT_EQUAL_INT((int32_t)TIMESPAN_MAXMILLISECONDS, timespanToMilliseconds(&timespan));
  memset(&timespan, 0, sizeof(timespan_t));

  /* Check lower limit */
  timespanCreate(TIMESPAN_MINNANOSECONDS, &timespan);
  TEST_ASSERT_EQUAL_INT(TIMESPAN_MINMILLISECONDS, timespanToMilliseconds(&timespan));
  memset(&timespan, 0, sizeof(timespan_t));

  /* One millisecond */
  timespanCreateExplicit(0, 0, 0, 0, 1, 0, 0, &timespan);
  TEST_ASSERT_EQUAL_INT(1, timespanToMilliseconds(&timespan));
  memset(&timespan, 0, sizeof(timespan_t));

  /* 1001 milliseconds */
  timespanCreateExplicit(0, 0, 0, 1, 1, 0, 0, &timespan);
  TEST_ASSERT_EQUAL_INT(1001, timespanToMilliseconds(&timespan));
  memset(&timespan, 0, sizeof(timespan_t));
}

/**************************************************************************/
/*
    Tests timespanToMicroseconds
 */
/**************************************************************************/
void test_timespan_toMicroseconds(void)
{
  /* Check upper limit */
  timespanCreate(TIMESPAN_MAXNANOSECONDS, &timespan);
  TEST_ASSERT_EQUAL_INT((int32_t)TIMESPAN_MAXMICROSECONDS, timespanToMicroseconds(&timespan));
  memset(&timespan, 0, sizeof(timespan_t));

  /* Check lower limit */
  timespanCreate(TIMESPAN_MINNANOSECONDS, &timespan);
  TEST_ASSERT_EQUAL_INT(TIMESPAN_MINMICROSECONDS, timespanToMicroseconds(&timespan));
  memset(&timespan, 0, sizeof(timespan_t));

  /* One microsecond */
  timespanCreateExplicit(0, 0, 0, 0, 0, 1, 0, &timespan);
  TEST_ASSERT_EQUAL_INT(1, timespanToMicroseconds(&timespan));
  memset(&timespan, 0, sizeof(timespan_t));

  /* 1001 microseconds */
  timespanCreateExplicit(0, 0, 0, 0, 1, 1, 0, &timespan);
  TEST_ASSERT_EQUAL_INT(1001, timespanToMicroseconds(&timespan));
  memset(&timespan, 0, sizeof(timespan_t));
}

/**************************************************************************/
/*
    Tests timespanDifference
 */
/**************************************************************************/
void test_timespan_difference(void)
{
  timespan_t t1, t2;

  /* 2 hours and 2 days = +46 hours difference */
  timespanCreate(TIMESPAN_NANOSPERHOUR * 2, &t1);
  timespanCreate(TIMESPAN_NANOSPERDAY * 2, &t2);
  TEST_ASSERT_TRUE(ERROR_NONE == timespanDifference(&t1, &t2, &timespan));
  TEST_ASSERT_EQUAL_INT(1, timespan.days);
  TEST_ASSERT_EQUAL_INT(22, timespan.hours);
  TEST_ASSERT_TRUE(timespan.__ticks == TIMESPAN_NANOSPERHOUR * 46);
  memset(&timespan, 0, sizeof(timespan_t));

  /* Check for negative values: 2 days and 2 hours = -46 hours */
  timespanCreate(TIMESPAN_NANOSPERDAY * 2, &t1);
  timespanCreate(TIMESPAN_NANOSPERHOUR * 2, &t2);
  TEST_ASSERT_TRUE(ERROR_NONE == timespanDifference(&t1, &t2, &timespan));
  TEST_ASSERT_EQUAL_INT(-1, timespan.days);
  TEST_ASSERT_EQUAL_INT(-22, timespan.hours);
  TEST_ASSERT_TRUE(timespan.__ticks == TIMESPAN_NANOSPERHOUR * -46);
  memset(&timespan, 0, sizeof(timespan_t));
}

/**************************************************************************/
/*
    Tests timespanAdd
 */
/**************************************************************************/
void test_timespan_add(void)
{
  timespan_t tadd;

  /* 2 days + 2 hours */
  timespanCreate(TIMESPAN_NANOSPERDAY * 2, &timespan);
  timespanCreate(TIMESPAN_NANOSPERHOUR * 2, &tadd);
  TEST_ASSERT_TRUE(ERROR_NONE == timespanAdd(&tadd, &timespan));
  TEST_ASSERT_EQUAL_INT(2, timespan.days);
  TEST_ASSERT_EQUAL_INT(2, timespan.hours);
  TEST_ASSERT_TRUE(timespan.__ticks == TIMESPAN_NANOSPERHOUR * 50);

  /* Overflow check */
  timespanCreate(TIMESPAN_MAXNANOSECONDS, &timespan);
  timespanCreate(1, &tadd);
  TEST_ASSERT_TRUE(ERROR_TIMESPAN_OUTOFRANGE == timespanAdd(&tadd, &timespan));

  /* Underflow check */
  timespanCreate(TIMESPAN_MINNANOSECONDS, &timespan);
  timespanCreate(-1, &tadd);
  TEST_ASSERT_TRUE(ERROR_TIMESPAN_OUTOFRANGE == timespanAdd(&tadd, &timespan));
}

/**************************************************************************/
/*
    Tests timespanSubtract
 */
/**************************************************************************/
void test_timespan_subtract(void)
{
  timespan_t tsub;

  /* 2 days - 2 hours */
  timespanCreate(TIMESPAN_NANOSPERDAY * 2, &timespan);
  timespanCreate(TIMESPAN_NANOSPERHOUR * 2, &tsub);
  TEST_ASSERT_TRUE(ERROR_NONE == timespanSubtract(&tsub, &timespan));
  TEST_ASSERT_EQUAL_INT(1, timespan.days);
  TEST_ASSERT_EQUAL_INT(22, timespan.hours);
  TEST_ASSERT_TRUE(timespan.__ticks == TIMESPAN_NANOSPERHOUR * 46);

  /* Overflow check */
  timespanCreate(TIMESPAN_MAXNANOSECONDS, &timespan);
  timespanCreate(-1, &tsub);
  TEST_ASSERT_TRUE(ERROR_TIMESPAN_OUTOFRANGE == timespanSubtract(&tsub, &timespan));

  /* Underflow check */
  timespanCreate(TIMESPAN_MINNANOSECONDS, &timespan);
  timespanCreate(1, &tsub);
  TEST_ASSERT_TRUE(ERROR_TIMESPAN_OUTOFRANGE == timespanSubtract(&tsub, &timespan));
}

/**************************************************************************/
/*
    Tests timespanSystemClockToTicks
 */
/**************************************************************************/
void test_timespan_systemClockToTicks(void)
{
  /* 1000 ticks @ 12MHz should equal 83333 ns */
  TEST_ASSERT_TRUE(83333 == timespanSystemClockToTicks(1000));
}
