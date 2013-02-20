/**************************************************************************/
/*!
    @file pn532_mifare.h

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
#ifndef __PN532_MIFARE_H__
#define __PN532_MIFARE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

// These may need to be enlarged for multi card support
#define PN532_RESPONSELEN_INLISTPASSIVETARGET (64)
#define PN532_RESPONSELEN_INDATAEXCHANGE      (64)

typedef enum pn532_mifare_cmd_e
{
  PN532_MIFARE_CMD_AUTH_A     = 0x60,
  PN532_MIFARE_CMD_AUTH_B     = 0x61,
  PN532_MIFARE_CMD_READ       = 0x30,
  PN532_MIFARE_CMD_WRITE      = 0xA0,
  PN532_MIFARE_CMD_TRANSFER   = 0xB0,
  PN532_MIFARE_CMD_DECREMENT  = 0xC0,
  PN532_MIFARE_CMD_INCREMENT  = 0xC1,
  PN532_MIFARE_CMD_RESTORE    = 0xC2
}
pn532_mifare_cmd_t;

#ifdef __cplusplus
}
#endif 

#endif
