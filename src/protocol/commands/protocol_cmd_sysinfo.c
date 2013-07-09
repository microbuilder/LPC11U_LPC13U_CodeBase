/**************************************************************************/
/*!
    @file     protocol_cmd_sysinfo.c
    @author   K. Townsend (microBuilder.eu)

    This command can be used to read system information based on a pre-
    defined key value. For example, sending the SYSINFO command with
    PROT_CMD_SYSINFO_KEY_EEPROMSIZE (0x0006) for the key will return the
    size of the onboard/onchip EEPROM if any is present.

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
#include "projectconfig.h"

#ifdef CFG_PROTOCOL

#include <stdio.h>
#include <string.h>
#include "boards/board.h"
#include "../protocol.h"
#include "core/iap/iap.h"

/**************************************************************************/
/*!
    SYSINFO Keys (indicates what specific system information we want)
*/
/**************************************************************************/
typedef enum
{
  PROT_CMD_SYSINFO_KEY_FIRST                = 0x0000,
  PROT_CMD_SYSINFO_KEY_CODEBASE_VERSION     = 0x0001,   /**< Code base version (3*U8) */
  PROT_CMD_SYSINFO_KEY_FIRMWARE_VERSION     = 0x0002,   /**< Firmware version (3*U8) */
  PROT_CMD_SYSINFO_KEY_MCU_STRING           = 0x0003,   /**< MCU model (string) */
  PROT_CMD_SYSINFO_KEY_SERIAL_NUMBER        = 0x0004,   /**< Unique on-chip serial number (4*U32) */
  PROT_CMD_SYSINFO_KEY_CLOCKSPEED           = 0x0005,   /**< Core clock speed in Hz (U32) */
  PROT_CMD_SYSINFO_KEY_EEPROMSIZE           = 0x0006,   /**< EEPROM size in bytes (U32) */
  PROT_CMD_SYSINFO_KEY_LAST
} prot_cmd_sysinfo_key_t;

/**************************************************************************/
/*!
    Returns system information for this board based on a 16-bit key
*/
/**************************************************************************/
error_t protcmd_sysinfo(uint8_t length, uint8_t const payload[], protMsgResponse_t* mess_response)
{
  uint16_t key = payload[4] << 8 | payload[5];

  /* Make sure we have a valid key */
  ASSERT( key > PROT_CMD_SYSINFO_KEY_FIRST, ERROR_INVALIDPARAMETER);
  ASSERT( key < PROT_CMD_SYSINFO_KEY_LAST, ERROR_INVALIDPARAMETER);

  switch(key)
  {
    case (PROT_CMD_SYSINFO_KEY_CODEBASE_VERSION):
    /* ====================================================================
        PROT_CMD_SYSINFO_KEY_CODEBASE_VERSION                   KEY: 0x0001
        -------------------------------------------------------------------
        Returns the parent code base version ID (see projectconfig.h)

        COMMAND:  [10] [02 00] [02] [02 00]
                  Optional Args       None
        RESPONSE: Payload Length      3
                  mess_response[4]    Major version number
                  mess_response[5]    Minor version number
                  mess_response[6]    Revision number
       ====================================================================*/
      mess_response->payload[3] = 3;
      mess_response->payload[4] = (uint8_t)CFG_CODEBASE_VERSION_MAJOR & 0xFF;
      mess_response->payload[5] = (uint8_t)CFG_CODEBASE_VERSION_MINOR & 0xFF;
      mess_response->payload[6] = (uint8_t)CFG_CODEBASE_VERSION_REVISION & 0xFF;
      break;

    case (PROT_CMD_SYSINFO_KEY_FIRMWARE_VERSION):
    /* ====================================================================
        PROT_CMD_SYSINFO_KEY_FIRMWARE_VERSION                   Key: 0x0002
        -------------------------------------------------------------------
        Returns the board-specific firmware version (defined in board_*.h)

        COMMAND:  [10] [02 00] [02] [02 00]
                  Optional Args       None
        RESPONSE: Payload Length      3
                  mess_response[4]    Major version number
                  mess_response[5]    Minor version number
                  mess_response[6]    Revision number
       ====================================================================*/
      mess_response->payload[3] = 3;
      mess_response->payload[4] = (uint8_t)CFG_FIRMWARE_VERSION_MAJOR & 0xFF;
      mess_response->payload[5] = (uint8_t)CFG_FIRMWARE_VERSION_MINOR & 0xFF;
      mess_response->payload[6] = (uint8_t)CFG_FIRMWARE_VERSION_REVISION & 0xFF;
      break;

    case (PROT_CMD_SYSINFO_KEY_MCU_STRING):
    /* ====================================================================
        PROT_CMD_SYSINFO_KEY_MCU_STRING                         Key: 0x0003
        -------------------------------------------------------------------
        Returns a string describing the MCU used on the board

        COMMAND:  [10] [02 00] [02] [03 00]
                  Optional Args       None
        RESPONSE: Payload Length      variable (0..60 bytes)
                  mess_response[4]    Start of MCU string
       ====================================================================*/
      #ifdef CFG_MCU_LPC11U24FBD48_401
        mess_response->payload[3] = (uint8_t)strlen("LPC11U24FBD48/401");
        memcpy(&mess_response->payload[4], "LPC11U24FBD48/401", strlen("LPC11U24FBD48/401"));
      #endif
      #ifdef CFG_MCU_LPC11U37FBD48_401
        mess_response->payload[3] = (uint8_t)strlen("LPC11U37FBD48/401");
        memcpy(&mess_response->payload[4], "LPC11U37FBD48/401", strlen("LPC11U37FBD48/401"));
      #endif
      #ifdef CFG_MCU_LPC1347FBD48
        mess_response->payload[3] = (uint8_t)strlen("LPC1347FBD48/401");
        memcpy(&mess_response->payload[4], "LPC1347FBD48/401", strlen("LPC1347FBD48/401"));
      #endif
      break;

    case (PROT_CMD_SYSINFO_KEY_SERIAL_NUMBER):
    /* ====================================================================
        PROT_CMD_SYSINFO_KEY_SERIAL_NUMBER                      Key: 0x0004
        -------------------------------------------------------------------
        Returns the uniaue four word serial number of the LPC chip

        COMMAND:  [10] [02 00] [02] [04 00]
                  Optional Args       None
        RESPONSE: Payload Length      16 bytes
                  mess_response[4]    ID 3 (uint32_t)
                  mess_response[8]    ID 2 (uint32_t)
                  mess_response[12]   ID 1 (uint32_t)
                  mess_response[16]   ID 0 (uint32_t)
       ====================================================================*/
      // mess_response->payload[3] = 16;
      // uint32_t uid[4];
      // iapReadUID(uid);
      // memcpy(&mess_response->payload[4], uid, 16);
      break;

    case (PROT_CMD_SYSINFO_KEY_CLOCKSPEED):
    /* ====================================================================
        PROT_CMD_SYSINFO_KEY_CLOCKSPEED                         Key: 0x0005
        -------------------------------------------------------------------
        Returns the core clock speed in Hz

        COMMAND:  [10] [02 00] [02] [05 00]
                  Optional Args       None
        RESPONSE: Payload Length      4 bytes
                  mess_response[4]    Clock speed in Hz (uint32_t)
       ====================================================================*/
      // mess_response->payload[3] = 4;
      // uint32_t speed = (uint32_t)SystemCoreClock;
      // memcpy(&mess_response->payload[4], &speed, sizeof(uint32_t));
      break;

    case (PROT_CMD_SYSINFO_KEY_EEPROMSIZE):
    /* ====================================================================
        PROT_CMD_SYSINFO_KEY_EEPROMSIZE                         Key: 0x0006
        -------------------------------------------------------------------
        Returns the size of the on-board or on-chip EEPROM, or 0 if no
        EEPROM is available

        COMMAND:  [10] [02 00] [02] [06 00]
                  Optional Args       None
        RESPONSE: Payload Length      4 bytes
                  mess_response[4]    EEPROM size in bytes (uint32_t)

       ====================================================================*/
      mess_response->payload[3] = 4;
      uint32_t eepromSize = (uint32_t)CFG_EEPROM_SIZE;
      memcpy(&mess_response->payload[4], &eepromSize, sizeof(uint32_t));
      break;

    default:
      return ERROR_INVALIDPARAMETER;
  }

  return ERROR_NONE;
}

#endif
