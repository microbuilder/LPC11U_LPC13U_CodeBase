/**************************************************************************/
/*! 
    @file     swspi.h
    @author   K. Townsend (microBuilder.eu)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2012, K. Townsend
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
#ifndef _SWSPI_H_
#define _SWSPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

typedef struct
{
  uint8_t  miso_port;
  uint8_t  miso_pin;
  uint8_t  mosi_port;
  uint8_t  mosi_pin;
  uint8_t  sck_port;
  uint8_t  sck_pin;
  uint8_t  cpol;            // Clock Polarity (1 = SCK high between frames, 0 = low)
  uint8_t  cpha;            // Clock Phase (1 = slave reads on rising clock edge, master on falling edge, 0 = inverse)
  uint32_t sck_delay;       // Delay in ticks between state changes on SCK
} swspi_config_t;

void    swspiInit(swspi_config_t *config);
uint8_t swspiTransferByte(swspi_config_t *config, uint8_t byte);

#ifdef __cplusplus
}
#endif 

#endif
