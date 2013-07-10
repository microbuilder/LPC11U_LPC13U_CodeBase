/**************************************************************************/
/*!
    @file     cmd_sysinfo.c
    @author   K. Townsend (microBuilder.eu)

    @brief    Displays some basic information about the core HW and system.
    @ingroup  CLI

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
#include <stdio.h>

#include "projectconfig.h"
#include "core/gpio/gpio.h"
#include "core/delay/delay.h"
#include "core/iap/iap.h"
#include "cli/cli.h"
#include "cli/commands.h"       // Generic helper functions

#ifdef CFG_PRINTF_UART
  #include "core/uart/uart.h"
#endif

#ifdef CFG_TFTLCD
  #include "drivers/displays/graphic/lcd.h"
#endif

#ifdef CFG_CHIBI
  #include "drivers/rf/802.15.4/chibi/chb.h"
  #include "drivers/rf/802.15.4/chibi/chb_drvr.h"
#endif

/**************************************************************************/
/*!
    'sysinfo' command handler
*/
/**************************************************************************/
void cmd_sysinfo(uint8_t argc, char **argv)
{
  /* Note: Certain values are only reported if CFG_INTERFACE_LONGSYSINFO
     is set to 1 in projectconfig.h.  These extra values are more useful
     for debugging than real-world use, so there's no point wasting
     flash space storing the text and code for them */

  /* Firmware Version */
  printf("%-25s : %u.%u.%u %s", STRING(LOCALISATION_TEXT_Code_Base_COLON_SPACE), CFG_CODEBASE_VERSION_MAJOR, CFG_CODEBASE_VERSION_MINOR, CFG_CODEBASE_VERSION_REVISION, CFG_PRINTF_NEWLINE);
  printf("%-25s : %u.%u.%u %s", STRING(LOCALISATION_TEXT_Firmware), CFG_FIRMWARE_VERSION_MAJOR, CFG_FIRMWARE_VERSION_MINOR, CFG_FIRMWARE_VERSION_REVISION, CFG_PRINTF_NEWLINE);

  /* MCU */
  #ifdef CFG_MCU_LPC11U24FBD48_401
  printf("%-25s : %s%s", STRING(LOCALISATION_TEXT_MCU), "LPC11U24FBD48/401", CFG_PRINTF_NEWLINE);
  #endif
  #ifdef CFG_MCU_LPC11U37FBD48_401
  printf("%-25s : %s%s", STRING(LOCALISATION_TEXT_MCU), "LPC11U37FBD48/401", CFG_PRINTF_NEWLINE);
  #endif
  #ifdef CFG_MCU_LPC1347FBD48
  printf("%-25s : %s%s", STRING(LOCALISATION_TEXT_MCU), "LPC1347FBD48/401", CFG_PRINTF_NEWLINE);
  #endif

  /* Unique Chip ID */
  #if CFG_INTERFACE_LONGSYSINFO
    uint32_t uid[4];
    iapReadUID(uid);  /* 1st byte is LSB, 4th byte is MSB */
    printf("%-25s : %08X%08X%08X%08X %s",
        STRING(LOCALISATION_TEXT_Serial_Number),
        (unsigned int)uid[3],
        (unsigned int)uid[2],
        (unsigned int)uid[1],
        (unsigned int)uid[0],
        CFG_PRINTF_NEWLINE);
  #endif

  /* Core Frequency */
  printf("%-25s : %u.%u %s %s", STRING(LOCALISATION_TEXT_System_Clock), (unsigned int)(SystemCoreClock / 1000000), (unsigned int)(SystemCoreClock % 1000000), STRING(LOCALISATION_SYMBOL_MHZ), CFG_PRINTF_NEWLINE);

  /* EEPROM Size */
  printf("%-25s : %u %s %s", STRING(LOCALISATION_TEXT_EEPROM_Size_COLON_SPACE), (unsigned int)CFG_EEPROM_SIZE, STRING(LOCALISATION_TEXT_bytes), CFG_PRINTF_NEWLINE);

  /* System Uptime (based on delay timer) */
  printf("%-25s : %u %s %s", STRING(LOCALISATION_TEXT_System_Uptime), (unsigned int)delayGetSecondsActive(), STRING(LOCALISATION_SYMBOL_SECONDS), CFG_PRINTF_NEWLINE);

  /* Wireless Settings (if CFG_CHIBI enabled) */
  #ifdef CFG_CHIBI
    do
    {
      chb_pcb_t *chibipcb = chb_get_pcb();
      /* Transceiver Model */
      printf("%-25s : %s %s", STRING(LOCALISATION_TEXT_RF_Transceiver), "AT86RF212", CFG_PRINTF_NEWLINE);
      /* Indicate promiscuous mode if enabled */
      #if CFG_CHIBI_PROMISCUOUS == 1
        printf("%-25s : %s %s", STRING(LOCALISATION_TEXT_RF_Receive_Mode), STRING(LOCALISATION_TEXT_Promiscuous), CFG_PRINTF_NEWLINE);
      #else
        printf("%-25s : %s %s", STRING(LOCALISATION_TEXT_RF_Receive_Mode), STRING(LOCALISATION_TEXT_Normal), CFG_PRINTF_NEWLINE);
      #endif
      /* 4-byte PAN ID */
      printf("%-25s : 0x%04X (%d) %s", STRING(LOCALISATION_TEST_802154_PAN_ID), CFG_CHIBI_PANID, CFG_CHIBI_PANID, CFG_PRINTF_NEWLINE);
      /* Short (4-byte) Node Address */
      printf("%-25s : 0x%04X (%d) %s", STRING(LOCALISATION_TEXT_802154_Node_Address), chibipcb->src_addr, chibipcb->src_addr, CFG_PRINTF_NEWLINE);
      /* Current 802.15.4 Channel */
      printf("%-25s : %d %s", STRING(LOCALISATION_TEXT_802154_Channel), CFG_CHIBI_CHANNEL, CFG_PRINTF_NEWLINE);
    } while(0);
  #endif

  // CLI and buffer Settings
  #if CFG_INTERFACE_LONGSYSINFO
    /* Max number of characters for a single CLI command */
    printf("%-25s : %d %s %s", STRING(LOCALISATION_TEXT_CLI_Max_Command_Size), CFG_INTERFACE_MAXMSGSIZE, STRING(LOCALISATION_TEXT_bytes), CFG_PRINTF_NEWLINE);
    /* Whether the IRW pin is enabled when a CLI command is complete */
    printf("%-25s : %s %s", STRING(LOCALISATION_TEXT_CLI_IRQ_Enabled), CFG_INTERFACE_ENABLEIRQ ? STRING(LOCALISATION_TEXT_True) : STRING(LOCALISATION_TEXT_False), CFG_PRINTF_NEWLINE);
    #if CFG_INTERFACE_ENABLEIRQ
      /* Location of the IRQ pin */
      printf("%-25s : %d.%d %s", STRING(LOCALISATION_TEXT_CLI_IRQ_Location), CFG_INTERFACE_IRQPORT, CFG_INTERFACE_IRQPIN, CFG_PRINTF_NEWLINE);
    #endif
  #endif

  #ifdef CFG_PRINTF_UART
    do
    {
      uart_pcb_t *uartpcb = uartGetPCB();
      /* UART Baud Rate */
      printf("%-25s : %u %s", STRING(LOCALISATION_TEXT_UART_Baud_Rate), (unsigned int)(uartpcb->baudrate), CFG_PRINTF_NEWLINE);
    } while(0);
  #endif

  // SD Card and FATFS
  #ifdef CFG_SDCARD
    /* Indicate if an SD card has been detected or not */
    printf("%-25s : %s %s", STRING(LOCALISATION_TEXT_SD_Card_Present), GPIOGetPinValue(CFG_SDCARD_CDPORT, CFG_SDCARD_CDPIN) ? STRING(LOCALISATION_TEXT_True) : STRING(LOCALISATION_TEXT_False), CFG_PRINTF_NEWLINE);
    #if CFG_INTERFACE_LONGSYSINFO
      /* Indicate whether the filesystem is read-only or if write is also enabled */
      printf("%-25s : %s %s", STRING(LOCALISATION_TEXT_FAT_File_System), CFG_SDCARD_READONLY ? STRING(LOCALISATION_TEXT_Read_Only) : STRING(LOCALISATION_TEXT_Read_FORWARDSLASH_Write), CFG_PRINTF_NEWLINE);
    #endif
  #endif

  // TFT LCD Settings (if CFG_TFTLCD enabled)
  #ifdef CFG_TFTLCD
    /* LCD width in pixels */
    printf("%-25s : %d %s", STRING(LOCALISATION_TEXT_LCD_Width), (int)lcdGetWidth(), CFG_PRINTF_NEWLINE);
    /* LCD height in pixels */
    printf("%-25s : %d %s", STRING(LOCALISATION_TEXT_LCD_Height), (int)lcdGetHeight(), CFG_PRINTF_NEWLINE);
    #if CFG_INTERFACE_LONGSYSINFO
      /* LCD Controller ID */
      printf("%-25s : %04X %s", STRING(LOCALISATION_TEXT_LCD_Controller_ID), (unsigned short)lcdGetControllerID(), CFG_PRINTF_NEWLINE);
      /* Whether small fonts are enabled or not */
      printf("%-25s : %s %s", STRING(LOCALISATION_TEXT_LCD_Small_Fonts), CFG_TFTLCD_INCLUDESMALLFONTS == 1 ? STRING(LOCALISATION_TEXT_True) : STRING(LOCALISATION_TEXT_False), CFG_PRINTF_NEWLINE);
      /* Whether anti-aliased fonts are enabled or not */
      printf("%-25s : %s %s", STRING(LOCALISATION_TEXT_LCD_AA_Fonts), CFG_TFTLCD_USEAAFONTS == 1 ? STRING(LOCALISATION_TEXT_True) : STRING(LOCALISATION_TEXT_False), CFG_PRINTF_NEWLINE);
      lcdProperties_t lcdprops = lcdGetProperties();
      /* Whether a touch screen is enabled or not */
      printf("%-25s : %s %s", STRING(LOCALISATION_TEXT_Touch_Screen), lcdprops.touchscreen ? STRING(LOCALISATION_TEXT_True) : STRING(LOCALISATION_TEXT_False), CFG_PRINTF_NEWLINE);
      if (lcdprops.touchscreen)
      {
        // printf("%-25s : %s %s", "Touch Screen Calibrated", eepromReadU8(CFG_EEPROM_TOUCHSCREEN_CALIBRATED) == 1 ? "True" : "False", CFG_PRINTF_NEWLINE);
        /* ADC Threshold for a valid touch screen event to occur */
        printf("%-25s : %d %s", STRING(LOCALISATION_TEXT_Touch_Screen_Threshold), CFG_TFTLCD_TS_DEFAULTTHRESHOLD, CFG_PRINTF_NEWLINE);
      }
    #endif
  #endif

  // Debug LED
  #if CFG_INTERFACE_LONGSYSINFO
    /* Location of the LED pin */
    printf("%-25s : %d.%d %s", STRING(LOCALISATION_TEXT_LED_Location), CFG_LED_PORT, CFG_LED_PIN, CFG_PRINTF_NEWLINE);
  #endif
}
