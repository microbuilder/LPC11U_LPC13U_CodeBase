/**************************************************************************/
/*!
    @file     cmd_i2c_read.c
    @author   K. Townsend (microBuilder.eu)

    @brief    Reads the specified number of bytes on the I2C bus.
    @ingroup  CLI

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, K. Townsend (microbuilder.eu)
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
#include "core/i2c/i2c.h"
#include "cli/commands.h"           // Generic helper functions

#if defined(CFG_ENABLE_I2C)

extern volatile uint8_t   I2CMasterBuffer[I2C_BUFSIZE];
extern volatile uint8_t   I2CSlaveBuffer[I2C_BUFSIZE];
extern volatile uint32_t  I2CReadLength, I2CWriteLength;

/**************************************************************************/
/*!
    @brief  Calls the underlying I2C driver (private function)
*/
/**************************************************************************/
err_t cmd_i2cReadWrapper(uint8_t addr, uint8_t len, uint8_t* values)
{
  uint8_t i;

  I2CWriteLength = 0;
  I2CReadLength = len;
  I2CMasterBuffer[0] = (addr << 1) | 0x01;

  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  /* Copy payload into the buffer */
  for (i=0;i<len;i++)
  {
    values[i] = I2CSlaveBuffer[i];
  }

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    'sysinfo' command handler
*/
/**************************************************************************/
void cmd_i2c_read(uint8_t argc, char **argv)
{
  err_t error;
  int32_t addr32;
  uint8_t addr;
  int32_t len32;
  uint8_t len;
  uint8_t values[I2C_BUFSIZE];

  getNumber(argv[0], &addr32);
  getNumber(argv[1], &len32);

  /* Make sure addr is valid */
  if ((addr32 < 0x03) || (addr32 > 0x78))
  {
    printf("%s%s", STRING(LOCALISATION_TEXT_Invalid_I2C_Address), CFG_PRINTF_NEWLINE);
    return;
  }
  addr = (uint8_t)addr32 & 0xFF;

  /* Make sure len is valid */
  if ((len32 > I2C_BUFSIZE) || (len32 < 1))
  {
    printf("%s %d%s", STRING(LOCALISATION_TEXT_len_must_be_less_than_equal), I2C_BUFSIZE, CFG_PRINTF_NEWLINE);
    return;
  }
  len = (uint8_t)len32 & 0xFF;

  /* Send read command */
  error = cmd_i2cReadWrapper(addr, len, values);

  /* Handle error cases */
  if (error)
  {
    switch(error)
    {
      case ERROR_I2C_NOACK:
        printf("%s%s", STRING(LOCALISATION_TEXT_No_ACK_received), CFG_PRINTF_NEWLINE);
        break;
      case ERROR_I2C_TIMEOUT:
        printf("%s%s", STRING(LOCALISATION_TEXT_Timeout), CFG_PRINTF_NEWLINE);
        break;
      default:
        printf("%s: %04X%s", STRING(LOCALISATION_TEXT_Unknown_Error), error, CFG_PRINTF_NEWLINE);
        break;
    }
    return;
  }

  /* Print values */
  uint8_t i;
  for (i=0;i<len;i++)
  {
    printf("%02X ", values[i]);
  }
  printf("%s", CFG_PRINTF_NEWLINE);
}

#endif /* CFG_ENABLE_I2C */
