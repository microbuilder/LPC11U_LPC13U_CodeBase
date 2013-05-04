/**************************************************************************/
/*!
    @file     board_simulator.c
    @author   K. Townsend (microBuilder.eu)

    @section DESCRIPTION

    Common, board-specific files for simulators in Crossworks, etc.

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, K. Townsend
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
#include "projectconfig.h"

#if defined CFG_BRD_SIMULATOR

#include "boards/board.h"

/**************************************************************************/
/*!
    @brief Board-specific initialisation function
*/
/**************************************************************************/
void boardInit(void)
{
  SystemCoreClockUpdate();
}

/**************************************************************************/
/*!
    @brief Primary entry point for this project.
*/
/**************************************************************************/
#if !defined(_TEST_)
int main(void)
{
  /* Configure the HW */
  boardInit();

  while (1)
  {
    /* ToDo: Do something! */
  }
}
#endif

/**************************************************************************/
/*!
    @brief Turns the LED(s) on or off
*/
/**************************************************************************/
void boardLED(uint8_t state)
{
  // ToDo!
}

/**************************************************************************/
/*!
    @brief  Configure the board for low power and enter sleep mode
*/
/**************************************************************************/
void boardSleep(void)
{
  // ToDo!
}

/**************************************************************************/
/*!
    @brief  Restores parts and system peripherals to an appropriate
            state after waking up from sleep mode
*/
/**************************************************************************/
void boardWakeup(void)
{
  // ToDo!
}

#endif
