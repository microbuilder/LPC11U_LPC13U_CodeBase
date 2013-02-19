/**************************************************************************/
/*!
    @file     board_lpc11u24debugger.c
    @author   K. Townsend (microBuilder.eu)

    @section DESCRIPTION

    Common, board-specific files for the microBuilder LPC11U24 SWD debugger

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

#if defined CFG_BRD_LPC11U24_DEBUGGER

#include "boards/board.h"
#include "board_11u24debugger.h"
#include "core/gpio/gpio.h"
#include "core/systick/systick.h"
#include "core/eeprom/eeprom.h"
#include "core/pmu/pmu.h"

#ifdef CFG_USB
  #include "core/usb/usbd.h"
  #ifdef CFG_USB_CDC
    #include "core/usb/usb_cdc.h"
  #endif
#endif

#ifdef CFG_INTERFACE
  #include "cli/cli.h"
#endif

/**************************************************************************/
/*!
    @brief Board-specific initialisation function
*/
/**************************************************************************/
void boardInit(void)
{
  SystemCoreClockUpdate();
  systickInit(CFG_SYSTICK_DELAY_IN_MS);
  GPIOInit();

  #ifdef CFG_PRINTF_UART
    uartInit(CFG_UART_BAUDRATE);
  #endif

  /* Power down unused blocks */
  uint32_t pdrunconfig = ((0x1<<0)  |  // IRCOUT powered down
                          (0x1<<1)  |  // IRC powered down
                          (0x0<<2)  |  // FLASH powered up
                          (0x1<<3)  |  // BOD powered down
                          (0x1<<4)  |  // ADC powered down
                          (0x0<<5)  |  // SYSOSC powered up
                          (0x1<<6)  |  // WDTOSC powered down
                          (0x0<<7)  |  // SYSPLL powered up
                          (0x0<<8)  |  // USBPLL powered up
                          (0x0<<9)  |  // RESERVED ... set as 0
                          (0x0<<10) |  // USBPAD powered up
                          (0x1<<11) |  // RESERVED ... set as 1
                          (0x0<<12) |  // RESERVED ... set as 0
                          (0x0<<13) |  // RESERVED ... set as 1
                          (0x0<<14) |  // RESERVED ... set as 1
                          (0x0<<15));  // RESERVED ... set as 1
  LPC_SYSCON->PDRUNCFG = pdrunconfig;

  /* Explicitly set all pin functions to a known state and disable any
     unnecessary pull-ups to save some power.  It's important to setup
     unbonded pins as well since they are still available on the die,
     and can save some power setting them to output low, and disabling
     their internal resistors (this cut ~350µA off the sleep current
     in initial testing)

  Pin                       Pin Config Bits               Selected Function     Notes
  =======================   ==========================    ===================   ============== */
  LPC_IOCON->RESET_PIO0_0   = (0<<0) | (0<<3);            // RESET              no pull-up/down
  LPC_IOCON->PIO0_1         = (0<<0) | (2<<3);            // GPIO               pull-up (ISP)
  LPC_IOCON->PIO0_2         = (0<<0) | (0<<3);            // GPIO               no pull-up/down
  LPC_IOCON->PIO0_3         = (1<<0) | (0<<3);            // USBVBUS            no pull-up/down
  LPC_IOCON->PIO0_4         = (1<<0);                     // I2C-SCL            I2C = standard mode
  LPC_IOCON->PIO0_5         = (1<<0);                     // I2C-SDA            I2C = standard mode
  LPC_IOCON->PIO0_6         = (1<<0) | (0<<3);            // USBConnect         no pull-up/down
  LPC_IOCON->PIO0_7         = (0<<0) | (0<<3);            // GPIO               no pull-up/down
  LPC_IOCON->PIO0_8         = (0<<0) | (0<<3);            // GPIO               no pull-up/down
  LPC_IOCON->PIO0_9         = (0<<0) | (0<<3);            // GPIO               no pull-up/down
  LPC_IOCON->SWCLK_PIO0_10  = (0<<0) | (2<<3);            // SWCLK              pull-up (SWCLK)
  LPC_IOCON->TDI_PIO0_11    = (1<<0) | (0<<3) | (1<<7);   // GPIO (not TDI)     no pull-up/down, ADMODE = digital
  LPC_IOCON->TMS_PIO0_12    = (1<<0) | (0<<3) | (1<<7);   // GPIO (not TMS)     no pull-up/down, ADMODE = digital
  LPC_IOCON->TDO_PIO0_13    = (1<<0) | (0<<3) | (1<<7);   // GPIO (not TDO)     no pull-up/down, ADMODE = digital
  LPC_IOCON->TRST_PIO0_14   = (1<<0) | (0<<3) | (1<<7);   // GPIO (not TRST)    no pull-up/down, ADMODE = digital
  LPC_IOCON->SWDIO_PIO0_15  = (0<<0) | (2<<3) | (1<<7);   // SWDIO              pull-up (SWDIO), ADMODE = digital
  LPC_IOCON->PIO0_16        = (0<<0) | (0<<3) | (1<<7);   // GPIO               no pull-up/down, ADMODE = digital
  LPC_IOCON->PIO0_17        = (0<<0) | (0<<3);            // GPIO               no pull-up/down
  #if defined CFG_PRINTF_UART
  LPC_IOCON->PIO0_18        = (1<<0) | (0<<3);            // RXD                no pull-up/down
  LPC_IOCON->PIO0_19        = (1<<0) | (0<<3);            // TXD                no pull-up/down
  #else
  LPC_IOCON->PIO0_18        = (0<<0) | (0<<3);            // GPIO               no pull-up/down
  LPC_IOCON->PIO0_19        = (0<<0) | (0<<3);            // GPIO               no pull-up/down
  #endif
  LPC_IOCON->PIO0_20        = (0<<0) | (0<<3);            // GPIO               no pull-up/down
  LPC_IOCON->PIO0_21        = (0<<0) | (0<<3);            // GPIO               no pull-up/down
  LPC_IOCON->PIO0_22        = (0<<0) | (0<<3) | (1<<7);   // GPIO               no pull-up/down, ADMODE = digital
  LPC_IOCON->PIO0_23        = (0<<0) | (0<<3) | (1<<7);   // GPIO               no pull-up/down, ADMODE = digital

  LPC_IOCON->PIO1_0         = (0<<0) | (0<<3) | (1<<7);   // GPIO               no pull-up/down, ADMODE = digital (pin not present on QFP48)
  LPC_IOCON->PIO1_1         = (0<<0) | (0<<3) | (1<<7);   // GPIO               no pull-up/down, ADMODE = digital (pin not present on QFP48)
  LPC_IOCON->PIO1_2         = (0<<0) | (0<<3) | (1<<7);   // GPIO               no pull-up/down, ADMODE = digital (pin not present on QFP48)
  LPC_IOCON->PIO1_3         = (0<<0) | (0<<3) | (1<<7);   // GPIO               no pull-up/down, ADMODE = digital (pin not present on QFP48)
  LPC_IOCON->PIO1_4         = (0<<0) | (0<<3) | (1<<7);   // GPIO               no pull-up/down, ADMODE = digital (pin not present on QFP48)
  LPC_IOCON->PIO1_5         = (0<<0) | (0<<3) | (1<<7);   // GPIO               no pull-up/down, ADMODE = digital (pin not present on QFP48)
  LPC_IOCON->PIO1_6         = (0<<0) | (0<<3) | (1<<7);   // GPIO               no pull-up/down, ADMODE = digital (pin not present on QFP48)
  LPC_IOCON->PIO1_7         = (0<<0) | (0<<3) | (1<<7);   // GPIO               no pull-up/down, ADMODE = digital (pin not present on QFP48)
  LPC_IOCON->PIO1_8         = (0<<0) | (0<<3) | (1<<7);   // GPIO               no pull-up/down, ADMODE = digital (pin not present on QFP48)
  LPC_IOCON->PIO1_9         = (0<<0) | (0<<3) | (1<<7);   // GPIO               no pull-up/down, ADMODE = digital (pin not present on QFP48)
  LPC_IOCON->PIO1_10        = (0<<0) | (0<<3) | (1<<7);   // GPIO               no pull-up/down, ADMODE = digital (pin not present on QFP48)
  LPC_IOCON->PIO1_11        = (0<<0) | (0<<3) | (1<<7);   // GPIO               no pull-up/down, ADMODE = digital (pin not present on QFP48)
  LPC_IOCON->PIO1_12        = (0<<0) | (0<<3) | (1<<7);   // GPIO               no pull-up/down, ADMODE = digital (pin not present on QFP48)
  LPC_IOCON->PIO1_13        = (0<<0) | (0<<3);            // GPIO               no pull-up/down
  LPC_IOCON->PIO1_14        = (0<<0) | (0<<3);            // GPIO               no pull-up/down
  LPC_IOCON->PIO1_15        = (0<<0) | (2<<3);            // GPIO               pull-up (SLP_TR)
  LPC_IOCON->PIO1_16        = (0<<0) | (0<<3);            // GPIO               no pull-up/down
  LPC_IOCON->PIO1_17        = (0<<0) | (0<<3);            // GPIO               no pull-up/down (pin not present on QFP48)
  LPC_IOCON->PIO1_18        = (0<<0) | (0<<3);            // GPIO               no pull-up/down (pin not present on QFP48)
  LPC_IOCON->PIO1_19        = (0<<0) | (0<<3);            // GPIO               no pull-up/down
  LPC_IOCON->PIO1_20        = (0<<0) | (0<<3);            // GPIO               no pull-up/down
  LPC_IOCON->PIO1_21        = (0<<0) | (0<<3);            // GPIO               no pull-up/down
  LPC_IOCON->PIO1_22        = (0<<0) | (0<<3);            // GPIO               no pull-up/down
  LPC_IOCON->PIO1_23        = (0<<0) | (2<<3);            // GPIO               pull-up (SD_DETECT)
  LPC_IOCON->PIO1_24        = (0<<0) | (0<<3);            // GPIO               no pull-up/down
  LPC_IOCON->PIO1_25        = (0<<0) | (0<<3);            // GPIO               no pull-up/down
  LPC_IOCON->PIO1_26        = (0<<0) | (0<<3);            // GPIO               no pull-up/down
  LPC_IOCON->PIO1_27        = (0<<0) | (0<<3);            // GPIO               no pull-up/down
  LPC_IOCON->PIO1_28        = (0<<0) | (0<<3);            // GPIO               no pull-up/down
  LPC_IOCON->PIO1_29        = (0<<0) | (0<<3);            // GPIO               no pull-up/down
  LPC_IOCON->PIO1_30        = (0<<0) | (0<<3);            // GPIO               no pull-up/down (pin not present on QFP48)
  LPC_IOCON->PIO1_31        = (0<<0) | (0<<3);            // GPIO               no pull-up/down

  /* Set pins to output where possible */
  LPC_GPIO->DIR[0] = ~( (1<<1) );         /* 0.1  = ISP (used as wakeup pin) */
  LPC_GPIO->DIR[1] = 0xFFFFFFFF;

  /* Set user LED pin to output and disable it */
  GPIOSetDir(CFG_LED_PORT, CFG_LED_PIN, 1);
  boardLED(CFG_LED_OFF);

  /* Initialise USB */
  #ifdef CFG_USB
    systickDelay(500);
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
  /* ToDo */
}

/**************************************************************************/
/*!
    @brief  Restores parts and system peripherals to an appropriate
            state after waking up from sleep mode
*/
/**************************************************************************/
void boardWakeup(void)
{
  /* ToDo */
}

#endif
