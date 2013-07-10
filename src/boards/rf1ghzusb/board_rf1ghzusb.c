/**************************************************************************/
/*!
    @file     board_rf1ghzusb.c
    @author   K. Townsend (microBuilder.eu)

    @section DESCRIPTION

    @brief    Board-specific files for the microBuilder LPC1347
              AT86RF212 USB stick

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

#if defined CFG_BRD_RF1GHZUSB

#include "boards/board.h"
#include "core/gpio/gpio.h"
#include "core/delay/delay.h"
#include "core/eeprom/eeprom.h"
#include "core/pmu/pmu.h"

#ifdef CFG_CHIBI
  #include "drivers/rf/802.15.4/chibi/chb.h"
  #include "drivers/rf/802.15.4/chibi/chb_drvr.h"
  #include "drivers/rf/802.15.4/chibi/messages.h"
  static chb_rx_data_t rx_data;
#endif

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

#ifdef CFG_CHIBI
/**************************************************************************/
/*!
    Converts the ED (Energy Detection) value to dBm using the following
    formula: dBm = RSSI_BASE_VAL + 1.03 * ED

    For more information see section 6.5 of the AT86RF212 datasheet
*/
/**************************************************************************/
int edToDBM(uint32_t ed)
{
  #if CFG_CHIBI_MODE == 0 || CFG_CHIBI_MODE == 1 || CFG_CHIBI_MODE == 2
    // Calculate for OQPSK (RSSI Base Value = -100)
    int dbm = (103 * ed - 10000);
  #else
    // Calculate for BPSK (RSSI Base Value = -98)
    int dbm = (103 * ed - 9800);
  #endif

  return dbm / 100;
}

void sendMessage(void)
{
  uint8_t msgbuf[10] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
  if(msgSend(0xFFFF, MSG_MESSAGETYPE_NONE, msgbuf, 10))
  {
    printf("Message TX failure%s", CFG_PRINTF_NEWLINE);
  }
}

void checkForMessages(void)
{
  chb_pcb_t *pcb = chb_get_pcb();

  while (pcb->data_rcv)
  {
    // Enable LED to indicate message reception
    boardLED(CFG_LED_ON);
    // get the length of the data
    rx_data.len = chb_read(&rx_data);
    // make sure the length is nonzero
    if (rx_data.len)
    {
      uint8_t msgType = rx_data.data[2];
      sensors_event_t *event;
      int dbm = edToDBM(pcb->ed);
      /* Handle the message based on the msgType */
      switch(msgType)
      {
        case (MSG_MESSAGETYPE_SENSOREVENT):
          event = (sensors_event_t*)&rx_data.data[9];
          printf("%04X,%d,", rx_data.src_addr, event->timestamp);
          printf("%f,%f,%f%s", event->acceleration.x, event->acceleration.y, event->acceleration.z, CFG_PRINTF_NEWLINE);
          break;
        default:
          printf("Message received from node 0x%04X (len=%d, dBm=%d):%s", rx_data.src_addr, rx_data.len, dbm, CFG_PRINTF_NEWLINE);
          printf("  Message ID:   0x%04X%s", *(uint16_t*)&rx_data.data[0], CFG_PRINTF_NEWLINE);
          printf("  Message Type: 0x%02X%s", *(uint8_t*)&rx_data.data[2], CFG_PRINTF_NEWLINE);
          printf("  Timestamp:    %d%s", *(uint32_t*)&rx_data.data[3], CFG_PRINTF_NEWLINE);
          printf("  Payload:      %d bytes%s", *(uint8_t*)&rx_data.data[8], CFG_PRINTF_NEWLINE);
          if (rx_data.data[8])
          {
            uint8_t i;
            printf("%s", CFG_PRINTF_NEWLINE);
            for (i = 0; i < rx_data.data[8]; i++)
            {
              printf("0x%02X ", *(uint8_t*)&rx_data.data[9+i]);
            }
          }
          printf("%s", CFG_PRINTF_NEWLINE);
          break;
      }
    }
    // Disable LED
    boardLED(CFG_LED_OFF);
  }
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
    /* Set IRQ to GPIO and set ADMODE to digital*/
    LPC_IOCON->TMS_PIO0_12    = (1<<0) | (2<<3) | (1<<7);
    /* Set RST to GPIO and set ADMODE to digital*/
    LPC_IOCON->TDO_PIO0_13    = (1<<0) | (0<<3) | (1<<7);
    /* Enable pull-up on SLPTR, and set ADMODE to digital*/
    LPC_IOCON->TRST_PIO0_14   = (1<<0) | (2<<3) | (1<<7);
    /* You may need to write a new address to EEPROM if it doesn't exist */
    // uint16_t nodeaddr = 0xCAFE;
    // uint64_t ieeeaddr = 0x123456780000CAFE;
    // writeEEPROM((uint8_t*)CFG_EEPROM_CHIBI_NODEADDR, (uint8_t*)&nodeaddr, sizeof(nodeaddr));
    // writeEEPROM((uint8_t*)CFG_EEPROM_CHIBI_IEEEADDR, (uint8_t*)&ieeeaddr, sizeof(ieeeaddr));
    if (chb_init())
    {
      // printf("Chibi init failed%s", CFG_PRINTF_NEWLINE);
    }
  #endif

  /* Initialise USB */
  #ifdef CFG_USB
    delay(500);
    usb_init();
  #endif

  /* Start the command line interface */
  #ifdef CFG_INTERFACE
    cliInit();
  #endif
}

/**************************************************************************/
/*!
    @brief Primary entry point for this project.
*/
/**************************************************************************/
#if !defined(_TEST_)
int main(void)
{
  uint32_t currentSecond, lastSecond;
  currentSecond = lastSecond = 0;

  boardInit();

  while (1)
  {
    currentSecond = delayGetSecondsActive();
    if (currentSecond != lastSecond)
    {
      lastSecond = currentSecond;

      /* Send a message over the air */
      #ifdef CFG_CHIBI
        // sendMessage();
      #endif
    }

    /* Poll for CLI input if CFG_INTERFACE is enabled */
    #ifdef CFG_INTERFACE
      cliPoll();
    #endif

    /* Check for incoming wireless messages */
    #ifdef CFG_CHIBI
      checkForMessages();
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
