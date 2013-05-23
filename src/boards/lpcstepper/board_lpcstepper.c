/**************************************************************************/
/*!
    @file     board_lpcxpresso1347.c
    @author   K. Townsend (microBuilder.eu)

    @section DESCRIPTION

    Common, board-specific files for lpcxpresso LPC1347 boards

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

#if defined CFG_BRD_LPCSTEPPER

#include "boards/board.h"
#include "core/gpio/gpio.h"
#include "core/delay/delay.h"
#include "core/eeprom/eeprom.h"
#include "core/pmu/pmu.h"
#include "drivers/motor/stepper/stepper.h"

#ifdef CFG_USB
  #include "core/usb/usbd.h"
  #ifdef CFG_USB_CDC
    #include "core/usb/usb_cdc.h"
  #endif
#endif

#ifdef CFG_INTERFACE
  #include "cli/cli.h"
#endif

#ifdef CFG_ENABLE_UART
  #include "core/uart/uart.h"
#endif

#ifdef CFG_SDCARD
/**************************************************************************/
/*!
    Handles timestamp requests for SD cards (adjust depending on if you
    want to use the RTC, or just return 0, etc.)
*/
/**************************************************************************/
DWORD get_fattime ()
{
  DWORD tmr = 0;

  // tmr =  (((DWORD)rtcYear - 80) << 25)
  //      | ((DWORD)rtcMon << 21)
  //      | ((DWORD)rtcMday << 16)
  //      | (WORD)(rtcHour << 11)
  //      | (WORD)(rtcMin << 5)
  //      | (WORD)(rtcSec >> 1);

  return tmr;
}
#endif

/**************************************************************************/
/*!
    @brief Board-specific initialisation function
*/
/**************************************************************************/
void boardInit(void)
{
  SystemCoreClockUpdate();
  delayInit();
  GPIOInit();

  #ifdef CFG_PRINTF_UART
    uartInit(CFG_UART_BAUDRATE);
  #endif

  /* Set pins to GPIO for MOTOR2 (default = JTAG) */
  LPC_IOCON->TDO_PIO0_13    = (1<<0) | (0<<3) | (1<<7);   // GPIO (not TDO)     no pull-up/down, ADMODE = digital
  LPC_IOCON->TRST_PIO0_14   = (1<<0) | (0<<3) | (1<<7);   // GPIO (not TRST)    no pull-up/down, ADMODE = digital

  /* Set user LED pin to output and disable it */
  LPC_GPIO->DIR[CFG_LED_PORT] |= (1 << CFG_LED_PIN);
  boardLED(CFG_LED_OFF);

  /* Initialise USB */
  #ifdef CFG_USB
    delay(500);
    usb_init();
  #endif

  /* Start the command line interface */
  #ifdef CFG_INTERFACE
    cliInit();
  #endif

  /* Turn the user LED on after init to indicate that everything is OK */
  boardLED(CFG_LED_ON);
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

  stepperInit(200);         // Initialise driver for 200-step motor
  stepperSetSpeed(60);      // Set speed to 120 rpm (2 revolutions per second)

  while (1)
  {
    stepperStep(400);       // Move forward 400 steps
    stepperStep(-200);      // Move backward 200 steps
    delay(1000);     // Wait one second

    // Move 'home' after 10 loops (current position = 2000)
    if (stepperGetPosition() == 2000)
    {
      stepperMoveHome();    // Move back to the starting position
      delay(1000);   // Wait one second
    }

    #ifdef CFG_INTERFACE
      cliPoll();
    #endif
  }

  while (1)
  {
    /* Poll for CLI input if CFG_INTERFACE is enabled */
    #ifdef CFG_INTERFACE
      cliPoll();
    #endif
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
  if (state)
  {
    LPC_GPIO->SET[CFG_LED_PORT] = (1 << CFG_LED_PIN);
  }
  else
  {
    LPC_GPIO->CLR[CFG_LED_PORT] = (1 << CFG_LED_PIN);
  }
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
