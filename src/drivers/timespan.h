/**************************************************************************/
/*!
    @file     timespan.h
    @author   K. Townsend (microBuilder.eu)
    @license  BSD (see license.txt)
*/
/**************************************************************************/
#ifndef _TIMESPAN_H_
#define _TIMESPAN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

#define TIMESPAN_NANOSPERMICROSECOND        (1000LL)
#define TIMESPAN_NANOSPERMILLISECOND        (TIMESPAN_NANOSPERMICROSECOND * 1000LL) /* 1,000,000          */
#define TIMESPAN_NANOSPERSECOND             (TIMESPAN_NANOSPERMILLISECOND * 1000LL) /* 1,000,000,000      */
#define TIMESPAN_NANOSPERMINUTE             (TIMESPAN_NANOSPERSECOND * 60LL)        /* 60,000,000,000     */
#define TIMESPAN_NANOSPERHOUR               (TIMESPAN_NANOSPERMINUTE * 60LL)        /* 3,600,000,000,000  */
#define TIMESPAN_NANOSPERDAY                (TIMESPAN_NANOSPERHOUR * 24LL)          /* 86,400,000,000,000 */

#define TIMESPAN_MAXNANOSECONDS             9223372036854775807LL
#define TIMESPAN_MINNANOSECONDS             -9223372036854775807LL
#define TIMESPAN_MAXMICROSECONDS            (TIMESPAN_MAXNANOSECONDS / TIMESPAN_NANOSPERMICROSECOND)
#define TIMESPAN_MINMICROSECONDS            (TIMESPAN_MINNANOSECONDS / TIMESPAN_NANOSPERMICROSECOND)
#define TIMESPAN_MAXMILLISECONDS            (TIMESPAN_MAXNANOSECONDS / TIMESPAN_NANOSPERMILLISECOND)
#define TIMESPAN_MINMILLISECONDS            (TIMESPAN_MINNANOSECONDS / TIMESPAN_NANOSPERMILLISECOND)
#define TIMESPAN_MAXSECONDS                 (TIMESPAN_MAXNANOSECONDS / TIMESPAN_NANOSPERSECOND)
#define TIMESPAN_MINSECONDS                 (TIMESPAN_MINNANOSECONDS / TIMESPAN_NANOSPERSECOND)
#define TIMESPAN_MAXMINUTES                 (TIMESPAN_MAXNANOSECONDS / TIMESPAN_NANOSPERMINUTE)
#define TIMESPAN_MINMINUTES                 (TIMESPAN_MINNANOSECONDS / TIMESPAN_NANOSPERMINUTE)
#define TIMESPAN_MAXHOURS                   (TIMESPAN_MAXNANOSECONDS / TIMESPAN_NANOSPERHOUR)
#define TIMESPAN_MINHOURS                   (TIMESPAN_MINNANOSECONDS / TIMESPAN_NANOSPERHOUR)
#define TIMESPAN_MAXDAYS                    (TIMESPAN_MAXNANOSECONDS / TIMESPAN_NANOSPERDAY)
#define TIMESPAN_MINDAYS                    (TIMESPAN_MINNANOSECONDS / TIMESPAN_NANOSPERDAY)

/**************************************************************************/
/*!
 * @brief     Timespan structure
 * @warning:  To ensure that this structure always contains a valid
 *            timespan, do not assign field values directly. Please
 *            call timespanCreate functions instead.
 */
/**************************************************************************/
typedef struct
{
  int32_t days;           /**< Number of days */
  int32_t hours;          /**< Number of hours (+/-0..23) */
  int32_t minutes;        /**< Number of minutes (+/-0..59) */
  int32_t seconds;        /**< Number of seconds (+/-0..59) */
  int32_t milliseconds;   /**< Number of milliseconds (+/-0..999) */
  int32_t microseconds;   /**< Number of microseconds (+/-0..999) */
  int32_t nanoseconds;    /**< Number of nanoseconds (+/-0..999) */
  int64_t __ticks;        /**< Total nanoseconds (used to track time internally) */
} timespan_t;

err_t timespanCreate(int64_t ticks, timespan_t *timespan);
err_t timespanCreateExplicit(int32_t days, int32_t hours, int32_t minutes, int32_t seconds, int32_t milliseconds, int32_t microseconds, int32_t nanoseconds, timespan_t *timespan);
err_t timespanDifference(timespan_t *t1, timespan_t *t2, timespan_t *timespan);
err_t timespanAdd(timespan_t *val, timespan_t *timespan);
err_t timespanSubtract(timespan_t *val, timespan_t *timespan);
int32_t timespanToHours(timespan_t *timespan);
int32_t timespanToMinutes(timespan_t *timespan);
int64_t timespanToSeconds(timespan_t *timespan);
int64_t timespanToMilliseconds(timespan_t *timespan);
int64_t timespanToMicroseconds(timespan_t *timespan);
int64_t timespanSystemClockToTicks(int32_t clockTicks);

#ifdef __cplusplus
}
#endif

#endif
