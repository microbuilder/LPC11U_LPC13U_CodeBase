/**************************************************************************/
/*!
    @file     protocol_cmd_adc.c
    @author   K. Townsend (microBuilder.eu)

    This command can be used to read an ADC channel on the MCU.

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

#include "core/adc/adc.h"

/**************************************************************************/
/*!
    Returns the results from a single read on the specified ADC channel

    PAYLOAD:  Byte 0 : ADC channel (0..7)

    RESPONSE: Payload Length : 2
              payload[0] : ADC results (lower byte)
              payload[1] : ADC result (upper byte)
*/
/**************************************************************************/
error_t protcmd_adc(uint8_t length, uint8_t const payload[], protMsgResponse_t* mess_response)
{
  uint8_t channel = payload[0];

  /* Make sure we have a valid channel */
  ASSERT (channel < 8, ERROR_INVALIDPARAMETER);

  /* Read the ADC channel and copy the results into mess_response */
  mess_response->length = 2;
  uint16_t results = (uint16_t)adcRead(channel);
  memcpy(&mess_response->payload[0], &results, sizeof(uint16_t));

  return ERROR_NONE;
}

#endif
