/**************************************************************************/
/*!
    @defgroup Boards Board/HW Abstration Layer

    @brief    Board/HW abstraction layer to allow multiple boards to share
              the same code base.

    @details

    Each board that uses this code base is required to implement a few key
    functions that allow you to abstract away HW-specific activities like
    board initialisation, and entering or exiting the low power modes.

    The generic board header file also contains all of the system-wide
    config settings for the project.
*/
/**************************************************************************/

/**************************************************************************/
/*!
    @file     board.h
    @author   K. Townsend (microBuilder.eu)

    @brief    Mandatory functions for the board/HW abstraction layer
    @ingroup  Boards

    @details

    Each board that uses this code base is required to implement the
    functions defined in this header.  This allows the core board-specific
    functions to be abstracted away during startup, and when entering the
    sleep modes, without having to be aware of what the target HW is.

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2012, K. Townsend
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
#ifndef __BOARD_H__
#define __BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

// Any board support file needs to implement these common methods, which
// allow the low-level board-specific details to be abstracted away from
// the higher level stacks and shared peripheral code

/**************************************************************************/
/*!
    @brief Initialises the HW and configures the board for normal operation
*/
/**************************************************************************/
void boardInit(void);

/**************************************************************************/
/*!
    @brief Turns the LED(s) on or off
*/
/**************************************************************************/
void boardLED(uint8_t state);

/**************************************************************************/
/*!
    @brief Configure the board for low power and enter sleep mode
*/
/**************************************************************************/
void boardSleep(void);

/**************************************************************************/
/*!
    @brief  Restores parts and system peripherals to an appropriate
            state after waking up from sleep mode
*/
/**************************************************************************/
void boardWakeup(void);

#ifdef __cplusplus
}
#endif

#endif
