/**************************************************************************/
/*!
    @file pn532_mifare_classic.h

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
#ifndef __PN532_MIFARE_CLASSIC_H__
#define __PN532_MIFARE_CLASSIC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"
#include "pn532_mifare.h"

/* Generic helper funtions for any Mifare Classic activity */
bool          pn532_mifareclassic_isFirstBlock (uint32_t uiBlock);
bool          pn532_mifareclassic_isTrailerBlock (uint32_t uiBlock);
pn532_error_t pn532_mifareclassic_WaitForPassiveTarget (byte_t * pbtCUID, size_t * szCUIDLen);
pn532_error_t pn532_mifareclassic_WaitForTypeATags (byte_t * pSak, uint16_t * pAtqa, byte_t * pbtCUID, size_t * szCUIDLen);
pn532_error_t pn532_mifareclassic_AuthenticateBlock (byte_t * pbtCUID, size_t szCUIDLen, uint32_t uiBlockNumber, uint8_t uiKeyType, byte_t * pbtKeys);
pn532_error_t pn532_mifareclassic_ReadDataBlock (uint8_t uiBlockNumber, byte_t * pbtData);
pn532_error_t pn532_mifareclassic_WriteDataBlock (uint8_t uiBlockNumber, byte_t * pbtData);
pn532_error_t pn532_mifareclassic_RFfield(bool fieldOn);
/* Help functions for Mifare Value Blocks */
pn532_error_t pn532_mifareclassic_CreateValueBlock (uint8_t uiBlockNumber, int32_t value);
pn532_error_t pn532_mifareclassic_ReadValueBlock (uint8_t uiBlockNumber, int32_t * value);
pn532_error_t pn532_mifareclassic_IncrementValueBlock (uint8_t uiBlockNumber, int32_t value);
pn532_error_t pn532_mifareclassic_DecrementValueBlock (uint8_t uiBlockNumber, int32_t value);

#ifdef __cplusplus
}
#endif

#endif
