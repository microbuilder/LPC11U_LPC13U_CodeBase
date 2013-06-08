/**************************************************************************/
/*!
    @file     dwt.h
    @author   K. Townsend (microBuilder.eu)

    Data Watchpoint and Trace Unit (DWT)

    @section DESCRIPTION

    For more information, see Cortex-M3 Technical Reference Manual 8.3
    This block is optional and not all comparators or functionality may
    be present on all chips, though basic DWT functionality is present
    on the LPC1347 since CYCNT works

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2012, K. Townsend (microbuilder.eu)
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
#ifndef _DWT_H_
#define _DWT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

/* DWT is only supported on the Cortex M3, so make sure this isn't an M0 */
#if defined CFG_MCU_FAMILY_LPC13UXX

// Macro to initialise, reset and enable the cycle counter.
// This can be used for rough timing and performance tests
// by resetting the cycle counter before a function, and
// then reading the value after with "int count = DWT->CYCCNT"
//
//    DWT_RESET_CYCLECOUNTER;
//    ... do something
//    int count = DWT->CYCCNT;

#define DWT_RESET_CYCLECOUNTER    do { CoreDebug->DEMCR = CoreDebug->DEMCR | 0x01000000;  \
                                       DWT->CYCCNT = 0;                                   \
                                       DWT->CTRL = DWT->CTRL | 1 ; } while(0)

/* Inline Functions */
static INLINE void dwtDelay(uint32_t ticks) INLINE_POST;

/**************************************************************************/
/*!
    @brief  Causes a delay for approximately the specified number of ticks

    @note   This function has the following overhead:
            Release mode:   ~21 cycles
            Debug mode:     ~36 cycles

    @param[in]  ticks
                The number of clock cycles to wait

    @section EXAMPLE

    @code

    // Test DWT delay
    uint32_t usec = 3;
    uint32_t ticks = (SystemCoreClock / 1000000) * usec;
    dwtDelay(ticks);                      // Delay (stays low)
    uint32_t oh = DWT->CYCCNT - ticks;    // Calculate function overhead

    @endcode
*/
/**************************************************************************/
static INLINE void dwtDelay(uint32_t ticks)
{
  // Use the DWT core clock cycle counter for reasonably precise
  // microsecond delays.  This is only supported by the Cortex M3 (not
  // the Cortex M0), and is an optional component present on the LPC1343
  // and LPC1347 ... but a less chip-dependent solution should be
  // implemented that also works with the LPC11U37 and LPC11U24.

  // Reset the cycle counter to 0 (good for up to 59 seconds @ 72MHz)
  DWT_RESET_CYCLECOUNTER;
  while (DWT->CYCCNT < ticks)
  {
    __asm volatile ("NOP");
  }
}

#endif // CFG_MCU_FAMILY_LPC13UXX

#ifdef __cplusplus
}
#endif

#endif
