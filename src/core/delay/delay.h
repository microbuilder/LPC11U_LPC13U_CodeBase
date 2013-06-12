/**************************************************************************/
/*!
    @file     delay.h
    @author   K. Townsend (microBuilder.eu)

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
#ifndef _DELAY_H_
#define _DELAY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

/*  STCTRL (System Timer Control and status register)
    The STCTRL register contains control information for the System Tick Timer, and provides
    a status flag.  */

#define SYSTICK_STCTRL_ENABLE                     (0x00000001)    // System tick counter enable
#define SYSTICK_STCTRL_TICKINT                    (0x00000002)    // System tick interrupt enable
#define SYSTICK_STCTRL_CLKSOURCE                  (0x00000004)    // NOTE: This isn't documented but is based on NXP examples
#define SYSTICK_STCTRL_COUNTFLAG                  (0x00010000)    // System tick counter flag

/*  STRELOAD (System Timer Reload value register)
    The STRELOAD register is set to the value that will be loaded into the System Tick Timer
    whenever it counts down to zero. This register is loaded by software as part of timer
    initialization. The STCALIB register may be read and used as the value for STRELOAD if
    the CPU or external clock is running at the frequency intended for use with the STCALIB
    value.  */

#define SYSTICK_STRELOAD_MASK                     (0x00FFFFFF)

/*  STCURR (System Timer Current value register)
    The STCURR register returns the current count from the System Tick counter when it is
    read by software. */

#define SYSTICK_STCURR_MASK                       (0x00FFFFFF)

/*  STCALIB (System Timer Calibration value register) */

#define SYSTICK_STCALIB_TENMS_MASK                (0x00FFFFFF)
#define SYSTICK_STCALIB_SKEW_MASK                 (0x40000000)
#define SYSTICK_STCALIB_NOREF_MASK                (0x80000000)

void     delayInit (void);
void     delay (uint32_t ticks);
uint32_t delayGetTicks(void);
uint32_t delayGetRollovers(void);
uint32_t delayGetSecondsActive(void);

#ifdef __cplusplus
}
#endif

#endif
