/**************************************************************************/
/*!
    @file     protocol_cmd_eeprom.c
    @author   K. Townsend (microBuilder.eu)

    This command can be used to read/write to the on chip EEPROM memory

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

#include "core/eeprom/eeprom.h"

/**************************************************************************/
/*!
    Reads the specified number of bytes from EEPROM memory

    PAYLOAD:  Byte 0 : EEPROM Address (lower byte)
              Byte 1 : EEPROM Address (upper byte)
              Byte 2 : len (1..32)

    RESPONSE: EEPROM contents

    EXAMPLE:  Read four bytes of EEPROM memory from address 0x0100:
              [10 04 00 03] 00 01 04
*/
/**************************************************************************/
error_t protcmd_eeprom_read(uint8_t length, uint8_t const payload[], protMsgResponse_t* mess_response)
{
  uint32_t address = (payload[0] << 8) | payload[1];
  uint32_t rdlen = payload[2];

  ASSERT( 3 == length, ERROR_PROT_INVALIDPAYLOAD);

  /* Make sure we have a valid address */
  ASSERT (address < CFG_EEPROM_SIZE, ERROR_INVALIDPARAMETER);
  ASSERT (rdlen > 0, ERROR_INVALIDPARAMETER);
  ASSERT (address + rdlen < CFG_EEPROM_SIZE, ERROR_INVALIDPARAMETER);

  /* Try to read the EEPROM memory */
  ASSERT_STATUS(readEEPROM((uint8_t*)address, &mess_response->payload[0], rdlen));
  mess_response->length = rdlen;

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    Writes the specified number of bytes to EEPROM memory

    PAYLOAD:  Byte 0  : EEPROM Address (lower byte)
              Byte 1  : EEPROM Address (upper byte)
              Byte 2  : len (1..32)
              Byte 3+ : Data to write

    RESPONSE: None

    EXAMPLE:  Write two bytes (0x12, 0x34) to EEPROM at address 0x0100:
              [10 05 00 05] 00 01 02 12 34
*/
/**************************************************************************/
error_t protcmd_eeprom_write(uint8_t length, uint8_t const payload[], protMsgResponse_t* mess_response)
{
  uint32_t address = (payload[0] << 8) | payload[1];
  uint32_t wrlen = payload[2];

  ASSERT( length > wrlen, ERROR_PROT_INVALIDPAYLOAD);

  /* Make sure we have a valid address */
  ASSERT (address < CFG_EEPROM_SIZE, ERROR_INVALIDPARAMETER);
  ASSERT (wrlen > 0, ERROR_INVALIDPARAMETER);
  ASSERT (address + wrlen < CFG_EEPROM_SIZE, ERROR_INVALIDPARAMETER);

  /* Try to write to EEPROM memory */
  ASSERT_STATUS(writeEEPROM((uint8_t*)address, (uint8_t*)&payload[3], wrlen));

  return ERROR_NONE;
}

#endif
