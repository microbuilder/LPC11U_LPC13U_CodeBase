/**************************************************************************/
/*!
    @file     cmd_nfc_mifareclassic_memdump.c
    @author   K. Townsend (microBuilder.eu)

    @brief    Dumps the contents of a Mifare Classic card to the CLI
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
    'nfc_mifareclassic_memdump' command handler
*/
/**************************************************************************/
void cmd_nfc_mifareclassic_memdump(uint8_t argc, char **argv)
{
  pn532_error_t error;
  byte_t abtUID[8];
  byte_t abtBlock[32];
  size_t szUIDLen;
  int32_t timeout;
  bool retriesChanged = false;

  // To dump an NDEF formatted Mifare Classic Card (formatted using NXP TagWriter on Android
  // for example), you must use the following authentication keys:
  //
  //    Sector 0:       0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5
  //    Sectors 1..15:  0xd3, 0xf7, 0xd3, 0xf7, 0xd3, 0xf7
  //
  // For more information on NDEF see the following document:
  //
  //    AN1305 - MIFARE Classic as NFC Type MIFARE Classic Tag
  //    http://www.nxp.com/documents/application_note/AN1305.pdf

  // Set this to one for NDEF cards, or 0 for blank factory default cards
  #define CARDFORMAT_NDEF (0)

  #if CARDFORMAT_NDEF == 1
    byte_t abtAuthKey1[6] = { 0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5 };   // Sector 0 of NXP formatter NDEF cards
    byte_t abtAuthKey2[6] = { 0xd3, 0xf7, 0xd3, 0xf7, 0xd3, 0xf7 };   // All other sectors use standard key (AN1305 p.20, Table 6)
  #else
    byte_t abtAuthKey1[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    byte_t abtAuthKey2[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
  #endif

  // Set a timeout waiting for passive targets (default = 0xFF, wait forever)
  if (argc > 0)
  {
    getNumber (argv[0], &timeout);
    if (timeout > 0xFF || timeout < 0)
    {
      printf("Invalid timeout [0..255]%s", CFG_PRINTF_NEWLINE);
      return;
    }
    else if (timeout > 0 || timeout < 0xFF)
    {
      // We can safely ignore errors here since there is a default value anyway
      pn532_config_SetPassiveActivationRetries(timeout);
      retriesChanged = true;
    }
  }

  // Use the MIFARE Classic Helper to read/write to the tag's EEPROM storage
  printf("Please insert a Mifare Classic 1K or 4K card%s%s", CFG_PRINTF_NEWLINE, CFG_PRINTF_NEWLINE);

  // Wait for any ISO14443A card
  error = pn532_mifareclassic_WaitForPassiveTarget(abtUID, &szUIDLen);
  if (!error)
  {
    // Mifare classic card found ... cycle through each sector
    uint8_t block;
    bool authenticated = false;
    for (block = 0; block < 64; block++)
    {
      // Check if this is a new block so that we can reauthenticate
      if (pn532_mifareclassic_isFirstBlock(block)) authenticated = false;
      if (!authenticated)
      {
        // Start of a new sector ... try to to authenticate
        printf("-------------------------Sector %02d--------------------------%s", block / 4, CFG_PRINTF_NEWLINE);
        error = pn532_mifareclassic_AuthenticateBlock (abtUID, szUIDLen, block, PN532_MIFARE_CMD_AUTH_A, block / 4 ? abtAuthKey2 : abtAuthKey1);
        if (error)
        {
          switch(error)
          {
            default:
              printf("Authentication error (0x%02x)%s", error, CFG_PRINTF_NEWLINE);
              break;
          }
        }
        else
        {
          authenticated = true;
        }
      }
      // If we're still not authenticated just skip the block
      if (!authenticated)
      {
        printf("Block %02d: ", block);
        printf("Unable to authenticate%s", CFG_PRINTF_NEWLINE);
      }
      else
      {
        // Authenticated ... we should be able to read the block now
        error = pn532_mifareclassic_ReadDataBlock (block, abtBlock);
        if (error)
        {
          switch(error)
          {
            default:
              printf("Block %02d: ", block);
              printf("Unable to read this block%s", CFG_PRINTF_NEWLINE);
              break;
          }
        }
        else
        {
          // Read successful
          printf("Block %02d: ", block);
          pn532PrintHexChar(abtBlock, 16);
        }
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

#endif
