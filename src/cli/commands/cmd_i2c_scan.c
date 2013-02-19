/**************************************************************************/
/*!
    @file     cmd_i2c_scan.c
    @author   K. Townsend (microBuilder.eu)

    @brief    Scans the I2C bus and displays a list of any addresses where
	          a valid I2C response occured.
    @ingroup  CLI

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2012, K. Townsend (microbuilder.eu)
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

#if defined(CFG_ENABLE_I2C)

/**************************************************************************/
/*!
    'sysinfo' command handler
*/
/**************************************************************************/
void cmd_i2c_scan(uint8_t argc, char **argv)
{
  uint8_t addr;
  bool match;

  i2cInit(I2CMASTER);

  // "Scanning I2C bus for devices"
  printf("%s%s%s", STRING(LOCALISATION_TEXT_Scanning_I2C_bus_for_devices), CFG_PRINTF_NEWLINE, CFG_PRINTF_NEWLINE);
  printf("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F%s", CFG_PRINTF_NEWLINE);
  printf("00:          ");

  // Scan through I2C address range (0x03-0x77)
  for (addr = 0x03; addr < 0x78; addr++)
  {
    if (!(addr % 0x10))
    {
      printf("%s", CFG_PRINTF_NEWLINE);
      printf("%X: ", addr);
    }

    // Check if we get an ACK at this address
    match = i2cCheckAddress(addr << 1);
    if (match)
    {
      // I2C device found
      printf("%02x ", addr);
    }
    else
    {
      // I2C slave didn't acknowledge the master transfer
      // The device probably isn't connected
      printf("-- ");
    }
  }

  printf("%s%s", CFG_PRINTF_NEWLINE, CFG_PRINTF_NEWLINE);
  // "All addresses are in 7-bit format and are not"
  printf("%s%s", STRING(LOCALISATION_TEXT_All_addresses_are_in_7bit_L1), CFG_PRINTF_NEWLINE);
  // "shifted to include the read bit."
  printf("%s%s", STRING(LOCALISATION_TEXT_All_addresses_are_in_7bit_L2), CFG_PRINTF_NEWLINE);
}

#endif /* CFG_ENABLE_I2C */
