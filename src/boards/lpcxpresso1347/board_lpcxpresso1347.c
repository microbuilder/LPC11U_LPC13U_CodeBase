/**************************************************************************/
/*!
    @file     board_lpcxpresso1347.c
    @author   K. Townsend (microBuilder.eu)

    @section DESCRIPTION

    Common, board-specific files for lpcxpresso LPC1347 boards

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
#include "projectconfig.h"

#ifdef CFG_BRD_LPCXPRESSO_LPC1347

#include <string.h> /* strlen */

#include "boards/board.h"
#include "core/gpio/gpio.h"
#include "core/delay/delay.h"
#include "core/eeprom/eeprom.h"
#include "core/adc/adc.h"

#ifdef CFG_CMSIS_RTOS
  #include "cmsis_os.h"
#endif

#ifdef CFG_CHIBI
  #include "messages.h"
  #include "drivers/rf/802.15.4/chibi/chb.h"
  #include "drivers/rf/802.15.4/chibi/chb_drvr.h"
#endif

#ifdef CFG_USB
  #include "core/usb/usbd.h"
  #ifdef CFG_USB_CDC
    #include "core/usb/usb_cdc.h"
  #endif
#endif

#ifdef CFG_TFTLCD
  #include "drivers/displays/graphic/lcd.h"
#endif

#ifdef CFG_INTERFACE
  #include "cli/cli.h"
#endif

#ifdef CFG_PROTOCOL
  #include "protocol/protocol.h"
#endif

#ifdef CFG_ENABLE_UART
  #include "core/uart/uart.h"
#endif

#ifdef CFG_CMSIS_RTOS
  #include "RTX_CM_lib.h"
#endif

#ifdef CFG_SDCARD
  #include "drivers/storage/fatfs/diskio.h"
  #include "drivers/storage/fatfs/ff.h"
#endif

#ifdef CFG_CC3000
  #include "drivers/rf/wifi/cc3000/wifi.h"
#endif

#include "drivers/leds/ws2812/ws2812.h"

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

  /* Set user LED pin to output and disable it */
  LPC_GPIO->DIR[CFG_LED_PORT] |= (1 << CFG_LED_PIN);
  boardLED(CFG_LED_OFF);

  /* Start Chibi */
  #ifdef CFG_CHIBI
    /* You may need to write a new address to EEPROM if it doesn't exist */
    // uint16_t nodeaddr = 0xCAFE;
    // uint64_t ieeeaddr = 0x123456780000CAFE;
    // writeEEPROM((uint8_t*)CFG_EEPROM_CHIBI_NODEADDR, (uint8_t*)&nodeaddr, sizeof(nodeaddr));
    // writeEEPROM((uint8_t*)CFG_EEPROM_CHIBI_IEEEADDR, (uint8_t*)&ieeeaddr, sizeof(ieeeaddr));
    chb_init();
  #endif

  /* Initialise USB */
  #ifdef CFG_USB
    delay(500);
    usb_init();
  #endif

  /* Initialise the LCD if requested */
  #ifdef CFG_TFTLCD
    lcdInit();
  #endif

  /* Start the command line interface */
  #ifdef CFG_INTERFACE
    cliInit();
  #endif

  /* Initialise the CC3000 WiFi module and connect to an AP */
  #ifdef CFG_CC3000
    /* Setup the CC3000 pins */
    LPC_IOCON ->TRST_PIO0_14  &= ~0x07;
    LPC_IOCON ->TRST_PIO0_14  |= 0x01;
    LPC_IOCON ->PIO0_17       &= ~0x07;
    LPC_IOCON ->PIO0_16       &= ~0x1F;
    LPC_IOCON ->PIO0_16       |= (1<<4);
  #endif

  /* Initialise the SD Card? */
  #ifdef CFG_SDCARD
    DSTATUS stat = disk_initialize(0);
  #endif

  /* Initialise ADC channel 1 (pin 0.12) */
  LPC_IOCON->TMS_PIO0_12   &= ~0x9F;
  LPC_IOCON->TMS_PIO0_12   |= 0x02;
  adcInit();

  /* Turn the user LED on after init to indicate that everything is OK */
  boardLED(CFG_LED_ON);
}

#ifndef _TEST_
#ifndef CFG_CMSIS_RTOS
/*=========================================================================
  STANDARD ENTRY POINT

  The default program entry point when an RTOS is not required.
  -------------------------------------------------------------------------*/
  volatile uint32_t test[8] = { 0 };
  int main(void)
  {
    uint32_t currentSecond, lastSecond;
    currentSecond = lastSecond = 0;

    /* Configure the HW */
    boardInit();

//                         //  GG    RR    BB
//    uint8_t buffer[30] = { 0xFF, 0x00, 0x00,
//                           0x00, 0xFF, 0x00,
//                           0x00, 0x00, 0xFF,
//                           0xFF, 0xFF, 0x00,
//                           0x00, 0xFF, 0xFF,
//                           0xFF, 0x00, 0xFF,
//                           0xFF, 0x00, 0x00,
//                           0x00, 0xFF, 0x00,
//                           0x00, 0x00, 0xFF,
//                           0xFF, 0xFF, 0x00 };
//
//    ws2812Init();
//    while(1)
//    {
//      ws2812WriteArray(buffer, 30);
//    }






//    ws2812Init();
//    uint8_t buffer[30];
//    uint8_t r,g,b;
//    r = g = b = 0;
//
//    while(1)
//    {
//      r++;
//      g--;
//      b++;
//      buffer[0] = g;
//      buffer[1] = r;
//      buffer[2] = b;
//      buffer[3] = g;
//      buffer[4] = r;
//      buffer[5] = b;
//      buffer[6] = g;
//      buffer[7] = r;
//      buffer[8] = b;
//      buffer[9] = g;
//      buffer[10] = r;
//      buffer[11] = b;
//      buffer[12] = g;
//      buffer[13] = r;
//      buffer[14] = b;
//      buffer[15] = g;
//      buffer[16] = r;
//      buffer[17] = b;
//      buffer[18] = g;
//      buffer[19] = r;
//      buffer[20] = b;
//      buffer[21] = g;
//      buffer[22] = r;
//      buffer[23] = b;
//      buffer[24] = g;
//      buffer[25] = r;
//      buffer[26] = b;
//      buffer[27] = g;
//      buffer[28] = r;
//      buffer[29] = b;
//      ws2812WriteArray(buffer, 30);
//      delay(10);
//    }

    while (1)
    {
      /* Blinky (1Hz) */
      currentSecond = delayGetSecondsActive();
      if (currentSecond != lastSecond)
      {
        lastSecond = currentSecond;
        boardLED(lastSecond % 2);
      }

      /* Check for binary protocol input if CFG_PROTOCOL is enabled */
      #ifdef CFG_PROTOCOL
        prot_task(NULL);
      #endif

      /* Poll for CLI input if CFG_INTERFACE is enabled */
      #ifdef CFG_INTERFACE
        cliPoll();
      #endif

      /* Optionally enter high level sleep mode here via WFI */
    }
  }
/*=========================================================================*/
#endif /* !CFG_CMSIS_RTOS */
#endif /* !_TEST_ */

#ifdef CFG_CMSIS_RTOS
/*=========================================================================
  RTOS ENTRY POINT

  The following code is used if CFG_CMSIS_RTOS (RTX) is defined in the
  board config file.  The RTOS requires a different entry point since the
  content switching timer needs to be configured, and an initial thread
  defined and started.
  -------------------------------------------------------------------------*/
  /* Thread IDs */
  osThreadId tid_mainthread;      /* ID for main thread   */
  osThreadId tid_blinkythread;    /* ID for blinky thread */

  /* Function prototypes */
  void main_thread(void const *argument);
  void blinky_thread (void const *argument);

  /* Thread definitions */
  osThreadDef(blinky_thread, osPriorityHigh, 1, 0);
  osThreadDef(main_thread, osPriorityNormal, 1, 4 * OS_MAINSTKSIZE);

  /**************************************************************************/
  /*!
      @brief Blinky thread, high priority, active very 3ms
  */
  /**************************************************************************/
  void blinky_thread (void const *argument)
  {
   while (1)
   {
     /* Pass control to other tasks for 1s */
     osDelay(1000);
     printf ("Thread 1\n");
   }
  }

  /**************************************************************************/
  /*!
      @brief Main thread, normal priority
  */
  /**************************************************************************/
  void main_thread(void const *argument)
  {
    uint32_t currentSecond, lastSecond;
    currentSecond = lastSecond = 0;

    for (;;)
    {
       osDelay(1000);
       boardLED((currentSecond++) & 1);
    }
  }

  /**************************************************************************/
  /*!
      @brief RTX code entry point (replaces non-RTOS main further down!)
  */
  /**************************************************************************/
  int main(void)
  {
    /* Initiaise the HW */
    boardInit();

    /* Initialise the RTX kernel */
    osKernelInitialize();

    /* Create out threads */
    tid_mainthread = osThreadCreate(osThread(main_thread), NULL);
    tid_blinkythread = osThreadCreate(osThread(blinky_thread), NULL);

    /* Start the kernel and then go into an endless loop */
    osKernelStart();

    /* No return */
    while(1);

    return 1;
  }
/*=========================================================================*/
#endif /* !CFG_CMSIS_RTOS */


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

#endif /* !CFG_BRD_LPCXPRESSO_LPC1347 */
