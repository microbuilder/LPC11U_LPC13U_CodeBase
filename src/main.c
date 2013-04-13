/**************************************************************************/
/*!
    @file     main.c
    @author   K. Townsend (microBuilder.eu)

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
#include "core/systick/systick.h"
#include "boards/board.h"

#ifdef CFG_INTERFACE
  #include "cli/cli.h"
#endif

#if defined(__CODE_RED)
  #include <cr_section_macros.h>
  #include <NXP/crp.h>
  __CRP const unsigned int CRP_WORD = CRP_NO_CRP;
#endif

int main(void)
{
  uint32_t currentSecond, lastSecond;
  currentSecond = lastSecond = 0;

  /* Board Initialisation                                                 *
   * ==================================================================== *
   * The target HW is set in projectconfig.h                              *
   *                                                                      *
   * Each board has a dedicated config and initialisation file in the     *
   * boards/ folder, and you can run one of two mandatory functions here: *
   *                                                                      *
   * boardInit() - Initialises all HW peripherals on the target HW, and   *
   *               then continues execution in this main.c file.  This    *
   *               will configure the pins, setup the clock, initialise   *
   *               USB appropriately, etc.                                *
   *                                                                      *
   * boardMain() - This function normally calls boardInit above, but then *
   *               continues to execute code inside the boardMain() file, *
   *               performing the dedicated task(s) the HW was designed   *
   *               for.  Normally, you will never return from this call   *
   *               unless there is an error condition.                    *
   * ==================================================================== */
   boardInit();

  /* Do blinky and scan for input on the CLI */
  while (1)
  {
    currentSecond = systickGetSecondsActive();
    
    if (currentSecond != lastSecond)
    {
      lastSecond = currentSecond;

      /* Toggle LED once per second */
      boardLED(lastSecond % 2);
    }
    
    /* Poll for CLI input */
    #ifdef CFG_INTERFACE
      cliPoll();
    #endif
  }

  return 0;
}
