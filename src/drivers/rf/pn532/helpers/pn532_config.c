/**************************************************************************/
/*!
    @file pn532_config.c

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

#ifdef CFG_PN532

#include <string.h>

#include "../pn532.h"
#include "../pn532_bus.h"
#include "pn532_config.h"

#include "core/delay/delay.h"

/**************************************************************************/
/*!
    @brief     Sets the MxRtyPassiveActivation byte of the
               RFConfiguration register

    @param     maxRetries    0xFF to wait forever, 0x00..0xFE to timeout
                             after maxRetries
*/
/**************************************************************************/
pn532_error_t pn532_config_SetPassiveActivationRetries(uint8_t maxRetries)
{
  pn532_error_t error;
  byte_t abtCommand[5];
  byte_t abtResponse[16];
  size_t szLen;

  #ifdef PN532_DEBUGMODE
    PN532_DEBUG("Updating Passive Activation Max Retries: 0x%02X (%d)%s", maxRetries, maxRetries, CFG_PRINTF_NEWLINE);
  #endif

  /* Send RFConfiguration command to adjust config item 5 (MaxRetries)    */
  abtCommand[0] = PN532_COMMAND_RFCONFIGURATION;
  abtCommand[1] = 5;                        /* Config item 5 (MaxRetries) */
  abtCommand[2] = 0xFF;                     /* MxRtyATR (default = 0xFF)  */
  abtCommand[3] = 0x01;                     /* MxRtyPSL (default = 0x01)  */
  abtCommand[4] = maxRetries;
  error = pn532Write(abtCommand, sizeof(abtCommand));
  if (error)
    return error;

  /* Wait until we get a valid response or a timeout                      */
  do
  {
    delay(25);
    error = pn532Read(abtResponse, &szLen);
  } while (error == PN532_ERROR_RESPONSEBUFFEREMPTY);
  if (error)
    return error;

  return PN532_ERROR_NONE;
}

#endif  // #ifdef CFG_PN532
