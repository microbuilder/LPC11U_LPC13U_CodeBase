/**************************************************************************/
/*!
    @file     cmd_nfc_mifareclassic_valueblock.c
    @author   K. Townsend (microBuilder.eu)

    @brief    Commands to work with 'Value Blocks' on Mifare Classic cards.
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

// Use default Mifare Classic authentication keys
static byte_t _cmd_mifare_valueblock_AuthKey1[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
static byte_t _cmd_mifare_valueblock_AuthKey2[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

// Handle common error cases here for pn532_mifareclassic_WaitForPassiveTarget()
#define VALUEBLOCK_PASSIVETARGET_ERRORHANDLER()  \
  do { switch (error) \
       { \
         case PN532_ERROR_WRONGCARDTYPE: \
           printf("Wrong card type%s", CFG_PRINTF_NEWLINE); \
           break; \
         case PN532_ERROR_TIMEOUTWAITINGFORCARD: \
           printf("Timed out waiting for a card%s", CFG_PRINTF_NEWLINE); \
           break; \
         default: \
           printf("Error establishing passive connection (0x%02x)%s", error, CFG_PRINTF_NEWLINE); \
           break; \
       } \
     } while (0);

// ToDo: The increment and decrement commands only vary by one command byte!
// Clean this code up with a single function that specifies if we are
// incrementing or decrementing!

/**************************************************************************/
/*!
    'cmd_nfc_mifareclassic_valueblock_create' command handler
*/
/**************************************************************************/
void cmd_nfc_mifareclassic_valueblock_create(uint8_t argc, char **argv)
{
  pn532_error_t error;
  byte_t abtUID[8];
  size_t szUIDLen;
  int32_t block;
  int32_t value;

  getNumber (argv[0], &block);
  getNumber (argv[1], &value);

  // Make sure the block address is valid, and we're not in the sector trailer
  if (block > 63 || block < 1 || pn532_mifareclassic_isTrailerBlock(block))
  {
    printf("Invalid block number%s", CFG_PRINTF_NEWLINE);
        return;
  }

  printf("Creating a value block at address %d%s%s", (int)block, CFG_PRINTF_NEWLINE, CFG_PRINTF_NEWLINE);

  // Set a timeout waiting for passive targets (default = 0xFF, wait forever)
  pn532_config_SetPassiveActivationRetries(0xFE);

  // Wait for any ISO14443A card
  error = pn532_mifareclassic_WaitForPassiveTarget(abtUID, &szUIDLen);
  if (!error)
  {
    // Mifare classic card found ... try to authenticate the sector
    if (pn532_mifareclassic_AuthenticateBlock (abtUID, szUIDLen, block, PN532_MIFARE_CMD_AUTH_A, (block / 4) ? _cmd_mifare_valueblock_AuthKey2 : _cmd_mifare_valueblock_AuthKey1))
    {
      printf("Authentication error %s", CFG_PRINTF_NEWLINE);
      return;
    }
    // We should be able to create the value block now
    if (pn532_mifareclassic_CreateValueBlock (block, value))
    {
      printf("Unable to write to block %d%s", (int)block, CFG_PRINTF_NEWLINE);
    }
    // Write successful
    printf("Updated block %02d with value '%d'%s", (int)block, (int)value, CFG_PRINTF_NEWLINE);
  }
  else
  {
    // Handled by macro since this is common to every function
    VALUEBLOCK_PASSIVETARGET_ERRORHANDLER()
  }

  // Set retry count back to infinite if it was changed
  pn532_config_SetPassiveActivationRetries(0xFF);
}

/**************************************************************************/
/*!
    'cmd_nfc_mifareclassic_valueblock_increment' command handler
*/
/**************************************************************************/
void cmd_nfc_mifareclassic_valueblock_increment(uint8_t argc, char **argv)
{
  pn532_error_t error;
  byte_t abtUID[8];
  size_t szUIDLen;
  int32_t block;
  int32_t value;

  getNumber (argv[0], &block);
  getNumber (argv[1], &value);

  // Make sure the block address is valid, and we're not in the sector trailer
  if (block > 63 || block < 1 || pn532_mifareclassic_isTrailerBlock(block))
  {
    printf("Invalid block number%s", CFG_PRINTF_NEWLINE);
        return;
  }

  printf("Trying to increment value block %d%s%s", (int)block, CFG_PRINTF_NEWLINE, CFG_PRINTF_NEWLINE);

  // Set a timeout waiting for passive targets (default = 0xFF, wait forever)
  pn532_config_SetPassiveActivationRetries(0xFE);

  // Wait for any ISO14443A card
  error = pn532_mifareclassic_WaitForPassiveTarget(abtUID, &szUIDLen);
  if (!error)
  {
    // Mifare classic card found ... try to authenticate the sector
    if (pn532_mifareclassic_AuthenticateBlock (abtUID, szUIDLen, block, PN532_MIFARE_CMD_AUTH_A, (block / 4) ? _cmd_mifare_valueblock_AuthKey2 : _cmd_mifare_valueblock_AuthKey1))
    {
      printf("Authentication error%s", CFG_PRINTF_NEWLINE);
      return;
    }
    // We should be able to increment the value block now
    error = pn532_mifareclassic_IncrementValueBlock (block, value);
    if (error)
    {
      switch (error)
      {
        case PN532_ERROR_INCORRECTBLOCKFORMAT:
          printf("Block %d is not a Mifare value block%s", (int)block, CFG_PRINTF_NEWLINE);
          break;
        default:
          printf("Unable to increment to block %d%s", (int)block, CFG_PRINTF_NEWLINE);
          break;
      }
    }
    // Increment successful
    printf("Incremented block %02d by '%d'%s", (int)block, (int)value, CFG_PRINTF_NEWLINE);
  }
  else
  {
    // Handled by macro since this is common to every function
    VALUEBLOCK_PASSIVETARGET_ERRORHANDLER()
  }

  // Set retry count back to infinite if it was changed
  pn532_config_SetPassiveActivationRetries(0xFF);
}

/**************************************************************************/
/*!
    'cmd_nfc_mifareclassic_valueblock_decrement' command handler
*/
/**************************************************************************/
void cmd_nfc_mifareclassic_valueblock_decrement(uint8_t argc, char **argv)
{
  pn532_error_t error;
  byte_t abtUID[8];
  size_t szUIDLen;
  int32_t block;
  int32_t value;

  getNumber (argv[0], &block);
  getNumber (argv[1], &value);

  // Make sure the block address is valid, and we're not in the sector trailer
  if (block > 63 || block < 1 || pn532_mifareclassic_isTrailerBlock(block))
  {
    printf("Invalid block number%s", CFG_PRINTF_NEWLINE);
        return;
  }

  printf("Trying to decrement value block %d%s%s", (int)block, CFG_PRINTF_NEWLINE, CFG_PRINTF_NEWLINE);

  // Set a timeout waiting for passive targets (default = 0xFF, wait forever)
  pn532_config_SetPassiveActivationRetries(0xFE);

  // Wait for any ISO14443A card
  error = pn532_mifareclassic_WaitForPassiveTarget(abtUID, &szUIDLen);
  if (!error)
  {
    // Mifare classic card found ... try to authenticate the sector
    if (pn532_mifareclassic_AuthenticateBlock (abtUID, szUIDLen, block, PN532_MIFARE_CMD_AUTH_A, (block / 4) ? _cmd_mifare_valueblock_AuthKey2 : _cmd_mifare_valueblock_AuthKey1))
    {
      printf("Authentication error%s", CFG_PRINTF_NEWLINE);
      return;
    }
    // We should be able to decrement the value block now
    error = pn532_mifareclassic_DecrementValueBlock (block, value);
    if (error)
    {
      switch (error)
      {
        case PN532_ERROR_INCORRECTBLOCKFORMAT:
          printf("Block %d is not a Mifare balue block%s", (int)block, CFG_PRINTF_NEWLINE);
          break;
        default:
          printf("Unable to decrement to block %d%s", (int)block, CFG_PRINTF_NEWLINE);
          break;
      }
    }
    // Decrement successful
    printf("Decremented block %02d by '%d'%s", (int)block, (int)value, CFG_PRINTF_NEWLINE);
  }
  else
  {
    // Handled by macro since this is common to every function
    VALUEBLOCK_PASSIVETARGET_ERRORHANDLER()
  }

  // Set retry count back to infinite if it was changed
  pn532_config_SetPassiveActivationRetries(0xFF);
}

/**************************************************************************/
/*!
    'cmd_nfc_mifareclassic_valueblock_read' command handler
*/
/**************************************************************************/
void cmd_nfc_mifareclassic_valueblock_read(uint8_t argc, char **argv)
{
  pn532_error_t error;
  byte_t abtUID[8];
  size_t szUIDLen;
  int32_t block;

  getNumber (argv[0], &block);

  // Set a timeout waiting for passive targets (default = 0xFF, wait forever)
  pn532_config_SetPassiveActivationRetries(0xFE);

  // Wait for any ISO14443A card
  error = pn532_mifareclassic_WaitForPassiveTarget(abtUID, &szUIDLen);
  if (!error)
  {
    // Try to authenticate the sector
    if (pn532_mifareclassic_AuthenticateBlock (abtUID, szUIDLen, block, PN532_MIFARE_CMD_AUTH_A, (block / 4) ? _cmd_mifare_valueblock_AuthKey2 : _cmd_mifare_valueblock_AuthKey1))
    {
      printf("Authentication error%s", CFG_PRINTF_NEWLINE);
      return;
    }
    // We should be able to read the value now
    int32_t value = 0;
    error = pn532_mifareclassic_ReadValueBlock(block, &value);
    if (error)
    {
      switch (error)
      {
        case PN532_ERROR_ADDRESSOUTOFRANGE:
          printf("Invalid block number%s", CFG_PRINTF_NEWLINE);
          break;
        case PN532_ERROR_INCORRECTBLOCKFORMAT:
          printf("Block %d is not a Mifare value block%s", (int)block, CFG_PRINTF_NEWLINE);
          break;
        default:
          printf("Unable to read block %d%s", (int)block, CFG_PRINTF_NEWLINE);
          break;
      }
    }
    // Read successful
    printf("Block %02d = %d (0x%08X)%s", (int)block, (int)value, (int)value, CFG_PRINTF_NEWLINE);
  }
  else
  {
    // Handled by macro since this is common to every function
    VALUEBLOCK_PASSIVETARGET_ERRORHANDLER()
  }

  // Set retry count back to infinite if it was changed
  pn532_config_SetPassiveActivationRetries(0xFF);
}

#endif  // #ifdef CFG_PN532
