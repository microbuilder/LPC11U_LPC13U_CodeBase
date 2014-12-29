/**************************************************************************/
/*!
    @file     eeprom.c
    @author   K. Townsend (microBuilder.eu)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2012, K. Townsend (microBuilder.eu)
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

#include <string.h>
#include "eeprom.h"

typedef void (*IAP)(unsigned int [],unsigned int[]);
static const IAP iap_entry = (IAP) 0x1FFF1FF1;

/**************************************************************************/
/*!
    @brief   Writes a byte array to the on-chip EEPROM memory

    @code
    uint8_t buffer[4];
    uint32_t address = 0x00000000;

    buffer[0] = 0x87;
    buffer[1] = 0x65;
    buffer[2] = 0x43;
    buffer[3] = 0x21;

    // Write four bytes starting at address
    writeEEPROM((uint8_t*)address, buffer, 4);
    @endcode
*/
/**************************************************************************/
RAMFUNC err_t writeEEPROM( uint8_t* eeAddress, uint8_t* buffAddress, uint32_t byteCount )
{
  unsigned int command[5], result[4];

  /* EEPROM Write     : IAP Command Code : 61
     Param 1          : EEPROM address
     Param 2          : RAM address of data/buffer to write
     Param 3          : Number of bytes to be written
     Param 4          : System clock frequency in kHz (SystemCoreClock/1000)
     Return Codes     : 0 - CMD_SUCCESS
                        4 - SRC_ADDR_NOT_MAPPED
                        5 - DST_ADDR_NOT_MAPPED */
  command[0] = 61;
  command[1] = (uint32_t) eeAddress;
  command[2] = (uint32_t) buffAddress;
  command[3] = byteCount;
  command[4] = (uint32_t) (SystemCoreClock/1000);

  /* Invoke IAP call (interrupts need to be disabled during IAP calls)...*/
  __disable_irq();
  iap_entry(command, result);
  __enable_irq();
  if (0 != result[0])
  {
    return ERROR_ADDRESSOUTOFRANGE;
  }
  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief   Reads a byte array from the on-chip EEPROM memory

    @code
    uint8_t buffer[4] = { 0x00, 0x00, 0x00, 0x00 };
    uint32_t address = 0x00000000;

    // Read four bytes starting at address
    readEEPROM((uint8_t*) address, buffer, 4);
    @endcode
*/
/**************************************************************************/
RAMFUNC err_t readEEPROM( uint8_t* eeAddress, uint8_t* buffAddress, uint32_t byteCount )
{
  unsigned int command[5], result[4];

  /* EEPROM Read      : IAP Command Code : 62
     Param 1          : EEPROM address
     Param 2          : RAM address to store data
     Param 3          : Number of bytes to read
     Param 4          : System clock frequency in kHz (SystemCoreClock/1000)
     Return Codes     : 0 - CMD_SUCCESS
                        4 - SRC_ADDR_NOT_MAPPED
                        5 - DST_ADDR_NOT_MAPPED */
  command[0] = 62;
  command[1] = (uint32_t) eeAddress;
  command[2] = (uint32_t) buffAddress;
  command[3] = byteCount;
  command[4] = (uint32_t) (SystemCoreClock/1000);

  /* Invoke IAP call (interrupts need to be disabled during IAP calls)...*/
  __disable_irq();
  iap_entry(command, result);
  __enable_irq();
  if (0 != result[0])
  {
    return ERROR_ADDRESSOUTOFRANGE;
  }
  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief   Dumps the entire contents of EEPROM via printf
*/
/**************************************************************************/
void eepromDump(void)
{
  uint32_t i,j;
  uint8_t z;

  char valBuff[12];
  char lineBuff[80];

  for (i=0; i<CFG_EEPROM_SIZE; i+=16)
  {
    sprintf(lineBuff,"0x%03X ", (unsigned int)i);
    for (j=0; j<16; j++)
    {
      readEEPROM( (uint8_t*) i+j, (uint8_t*) &z, 1);
      sprintf(valBuff,"%02X ", (unsigned int)z);
      strcat(lineBuff,valBuff);
    }
    strcat(lineBuff,"\r\n");
    printf((char*) lineBuff);
  }
}
