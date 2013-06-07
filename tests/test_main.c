/**************************************************************************/
/*!
    @file     test_main.c
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
#ifdef _TEST_

#include "unity_fixture.h"
#include "projectconfig.h"
#include "boards/board.h"

#ifdef CFG_USB
  #include "core/usb/usbd.h"
#endif

void runAllTests(void)
{
  RUN_TEST_GROUP(fifo);
  RUN_TEST_GROUP(fixed);
  RUN_TEST_GROUP(iir);
  RUN_TEST_GROUP(sma);
  RUN_TEST_GROUP(rtc);
  RUN_TEST_GROUP(timespan);
  #ifdef CFG_PN532
  RUN_TEST_GROUP(ndef);
  #endif
}

int main(void)
{
  /* Configure common and board-level peripherals.
   * If you're using a simulator and doing SW only tests
   * disable this line since it will likely cause the
   * simulator to hang initialising a non-existant
   * peripheral or waiting for a systick event.            */
  boardInit();

  /* Wait for 't' to start the test if we're using USB CDC */
  #if defined(CFG_USB) && defined(CFG_PRINTF_USBCDC)
    while (! usb_isConfigured()) {}
    uint8_t c = 0;
    while ( ! (usb_cdc_getc(&c) && c == 't' ) ){}
  #endif

  /* Run all test groups in verbose mode */
  char* argv[]= {"unity" , "-v" };
  UnityMain(2, argv, runAllTests);

  /* Wait around forever when test run is complete */
  while (1)
  {
  }

  return 0;
}

#endif
