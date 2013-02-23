/**************************************************************************/
/*!
    @file     gpio.h

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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

#define CHANNEL0        (0)
#define CHANNEL1        (1)
#define CHANNEL2        (2)
#define CHANNEL3        (3)
#define CHANNEL4        (4)
#define CHANNEL5        (5)
#define CHANNEL6        (6)
#define CHANNEL7        (7)

#define PORT0           (0)
#define PORT1           (1)

#define GROUP0          (0)
#define GROUP1          (1)

void     GPIOInit ( void );
void     GPIOSetPinInterrupt ( uint32_t channelNum, uint32_t portNum, uint32_t bitPosi, uint32_t sense, uint32_t event );
void     GPIOPinIntEnable ( uint32_t channelNum, uint32_t event );
void     GPIOPinIntDisable ( uint32_t channelNum, uint32_t event );
uint32_t GPIOPinIntStatus ( uint32_t channelNum );
void     GPIOPinIntClear ( uint32_t channelNum );
void     GPIOSetGroupedInterrupt ( uint32_t groupNum, uint32_t *bitPattern, uint32_t logic, uint32_t sense, uint32_t *eventPattern );
uint32_t GPIOGetPinValue ( uint32_t portNum, uint32_t bitPosi );
void     GPIOSetBitValue ( uint32_t portNum, uint32_t bitPosi, uint32_t bitVal );
void     GPIOSetDir ( uint32_t portNum, uint32_t bitPosi, uint32_t dir );

#ifdef __cplusplus
}
#endif

#endif
