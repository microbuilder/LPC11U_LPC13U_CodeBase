/**************************************************************************/
/*!
    @file     pcf2129.h
    @author   K. Townsend (microBuilder.eu)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2012 K. Townsend
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
#ifndef _PCF2129_H_
#define _PCF2129_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"
#include "core/i2c/i2c.h"
#include "drivers/rtc/rtc.h"

#define PCF2129_ADDRESS           (0x51<<1)
#define PCF2129_READBIT           (0x01)

#define PCF2129_INT_ENABLED       (true)
#define PCF2129_INT_PORT          (0)
#define PCF2129_INT_PIN           (16)
#define PCF2129_INT_FLEXIRQNUM    (1)       // Use FLEXIRQ 1
#define PCF2129_INT_SETPULLUP     do { LPC_IOCON->PIO0_16 = (0<<0) | (2<<3) | (1<<7); } while(0)

#define PCF2129_TIMESTAMP_ENABLED (false)
#define PCF2129_TIMESTAMP_PORT    (0)
#define PCF2129_TIMESTAMP_PIN     (17)

typedef enum
{
  PCF2129_INTEVENT_SECONDTIMER    = (1<<0),
  PCF2129_INTEVENT_MINUTETIMER    = (1<<1),
  PCF2129_INTEVENT_ALARM          = (1<<2),
  PCF2129_INTEVENT_TIMESTAMP      = (1<<3),
  PCF2129_INTEVENT_BATTERYSWITCH  = (1<<4),
  PCF2129_INTEVENT_BATTERYLOW     = (1<<5)
} pcf2129_INTEvent_t;

enum
{
  PCF2129_REG_CONTROL1            = 0x00,
  PCF2129_REG_CONTROL2            = 0x01,
  PCF2129_REG_CONTROL3            = 0x02,
  PCF2129_REG_SECONDS             = 0x03,
  PCF2129_REG_MINUTES             = 0x04,
  PCF2129_REG_HOURS               = 0x05,
  PCF2129_REG_DAYS                = 0x06,
  PCF2129_REG_WEEKDAYS            = 0x07,
  PCF2129_REG_MONTHS              = 0x08,
  PCF2129_REG_YEARS               = 0x09,
  PCF2129_REG_ALARM_SECOND        = 0x0A,
  PCF2129_REG_ALARM_MINUTE        = 0x0B,
  PCF2129_REG_ALARM_HOUR          = 0x0C,
  PCF2129_REG_ALARM_DAY           = 0x0D,
  PCF2129_REG_ALARM_WEEKDAY       = 0x0E,
  PCF2129_REG_CLOCKOUT_CTRL       = 0x0F,
  PCF2129_REG_WATCHDOG_TIM_CTRL   = 0x10,
  PCF2129_REG_WATCHDOG_TIM_VAL    = 0x11,
  PCF2129_REG_TIMESTAMP_CTRL      = 0x12,
  PCF2129_REG_TIMESTAMP_SEC       = 0x13,
  PCF2129_REG_TIMESTAMP_MIN       = 0x14,
  PCF2129_REG_TIMESTAMP_HOUR      = 0x15,
  PCF2129_REG_TIMESTAMP_DAY       = 0x16,
  PCF2129_REG_TIMESTAMP_MONTH     = 0x17,
  PCF2129_REG_TIMESTAMP_YEAR      = 0x18,
  PCF2129_REG_AGINGOFFSET         = 0x19
};

err_t pcf2129Init(void);
void    pcf2129SetCallback (void (*pFunc)(void));
err_t pcf2129ReadTime(rtcTime_t *time);
err_t pcf2129SetTime(rtcTime_t time);
err_t pcf2129SetInterrupt(pcf2129_INTEvent_t eventFlags);

#ifdef __cplusplus
}
#endif 

#endif
