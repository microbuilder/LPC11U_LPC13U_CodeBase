/**************************************************************************/
/*!
    @file     protocol_cmd_sysinfo.c
    @author   K. Townsend (microBuilder.eu)

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

/**************************************************************************/
/*!
    SYSINFO Keys (indicates what specific system information we want)
*/
/**************************************************************************/
typedef enum
{
  PROT_CMD_SYSINFO_KEY_FIRST                    = 0x0000,
  PROT_CMD_SYSINFO_KEY_CODEBASE_VERSION         = 0x0001,
  PROT_CMD_SYSINFO_KEY_FIRMWARE_VERSION         = 0x0002,
  PROT_CMD_SYSINFO_KEY_MCU_STRING               = 0x0003,
  PROT_CMD_SYSINFO_KEY_SERIAL_NUMBER            = 0x0004,
  PROT_CMD_SYSINFO_KEY_CLOCKSPEED               = 0x0005,
  PROT_CMD_SYSINFO_KEY_EEPROMSIZE               = 0x0006,
  PROT_CMD_SYSINFO_KEY_LAST
} prot_cmd_sysinfo_key_t;

/**************************************************************************/
/*!
    Returns specific system information for this board, based on the
    specific 16-bit key
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
      mess_response->payload[3] = (uint8_t)3;
      mess_response->payload[4] = (uint8_t)CFG_CODEBASE_VERSION_MAJOR & 0xFF;
      mess_response->payload[5] = (uint8_t)CFG_CODEBASE_VERSION_MINOR & 0xFF;
      mess_response->payload[6] = (uint8_t)CFG_CODEBASE_VERSION_REVISION & 0xFF;
      break;

    case (PROT_CMD_SYSINFO_KEY_FIRMWARE_VERSION):
      mess_response->payload[3] = 3;
      mess_response->payload[4] = (uint8_t)CFG_FIRMWARE_VERSION_MAJOR & 0xFF;
      mess_response->payload[5] = (uint8_t)CFG_FIRMWARE_VERSION_MINOR & 0xFF;
      mess_response->payload[6] = (uint8_t)CFG_FIRMWARE_VERSION_REVISION & 0xFF;
      break;

    case (PROT_CMD_SYSINFO_KEY_MCU_STRING):
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
      break;

    case (PROT_CMD_SYSINFO_KEY_CLOCKSPEED):
      break;

    case (PROT_CMD_SYSINFO_KEY_EEPROMSIZE):
      mess_response->payload[3] = 4;
      uint32_t eepromSize= (uint32_t)CFG_EEPROM_SIZE;
      memcpy(&mess_response->payload[4], &eepromSize, sizeof(uint32_t));
      break;

    default:
      return ERROR_INVALIDPARAMETER;
  }

  return ERROR_NONE;
}

#endif
