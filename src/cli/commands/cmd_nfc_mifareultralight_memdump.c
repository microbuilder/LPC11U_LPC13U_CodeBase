/**************************************************************************/
/*!
    @file     cmd_nfc_mifareultralight_memdump.c
    @author   K. Townsend (microBuilder.eu)

    @brief    Tries to authenticate a Mifare Ultralight card using the default
              authentication key, and displays the raw memory contents if
              possible.
    @ingroup  CLI

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013 K. Townsend (microBuilder.eu)
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

#ifdef CFG_PN532

#include "core/gpio/gpio.h"
#include "cli/cli.h"
#include "cli/commands.h"       // Generic helper functions

#include "core/i2c/i2c.h"
#include "drivers/rf/nfc/pn532/pn532.h"
#include "drivers/rf/nfc/pn532/pn532_bus.h"
#include "drivers/rf/nfc/pn532/helpers/pn532_config.h"
#include "drivers/rf/nfc/pn532/helpers/pn532_mifare_classic.h"
#include "drivers/rf/nfc/pn532/helpers/pn532_mifare_ultralight.h"

/**************************************************************************/
/*!
    'nfc_mifareultralight_memdump' command handler
*/
/**************************************************************************/
void cmd_nfc_mifareultralight_memdump(uint8_t argc, char **argv)
{
  pn532_error_t error;
  byte_t        abtBuffer[8];
  size_t        szUIDLen;
  int32_t       timeout;
  bool          retriesChanged = false;

  // Set a timeout waiting for passive targets (default = 0xFF, wait forever)
  if (argc > 0)
  {
    getNumber (argv[0], &timeout);
    if (timeout > 0xFF || timeout < 0)
    {
      printf("Invalid timeout [0..255]%s", CFG_PRINTF_NEWLINE);
      return;
    }
    // We can safely ignore errors here since there is a default value anyway
    pn532_config_SetPassiveActivationRetries(timeout);
    retriesChanged = true;
  }

  printf("Please insert a Mifare Ultralight card%s%s", CFG_PRINTF_NEWLINE, CFG_PRINTF_NEWLINE);

  // Set a timeout waiting for passive targets (default = 0xFF, wait forever)
  // We can safely ignore errors here since there is a default value anyway
  getNumber (argv[0], &timeout);
  if ((timeout < 1) || (timeout > 0xFE))
    timeout = 0xFF;
  pn532_config_SetPassiveActivationRetries(timeout & 0xFF);

  // Try to do a memory dump of a Mifare Ultralight card
  // First wait for a card to arrive (will wake the PN532 if required)
  error = pn532_mifareultralight_WaitForPassiveTarget(abtBuffer, &szUIDLen);
  if (!error)
  {
    // Display the card's UID (normally 7 bytes long)
    printf("UID: ");
    pn532PrintHex(abtBuffer, szUIDLen);
    printf("%s", CFG_PRINTF_NEWLINE);
    printf("Page  Hex       Text%s", CFG_PRINTF_NEWLINE);
    printf("----  --------  ----%s", CFG_PRINTF_NEWLINE);
    // Dump the memory contents page by page
    uint8_t i;
    for (i = 0; i < 16; i++)
    {
      // Try to read the current page
      error = pn532_mifareultralight_ReadPage(i, abtBuffer);
      if (!error)
      {
        printf("0x%02x  ", i);
        pn532PrintHexChar(abtBuffer, 4);
      }
    }
  }
  else
  {
    switch (error)
    {
      case PN532_ERROR_WRONGCARDTYPE:
        printf("Wrong card type%s", CFG_PRINTF_NEWLINE);
        break;
      case PN532_ERROR_TIMEOUTWAITINGFORCARD:
        printf("Timed out waiting for a card%s", CFG_PRINTF_NEWLINE);
        break;
      default:
        printf("Error establishing passive connection (0x%02x)%s", error, CFG_PRINTF_NEWLINE);
        break;
    }
  }

  // Set retry count back to infinite if it was changed
  if (retriesChanged)
  {
    pn532_config_SetPassiveActivationRetries(0xFF);
  }
}

#endif  // #ifdef CFG_PN532
