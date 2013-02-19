/**************************************************************************/
/*!
    @file     pn532_mifare_classic.h
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
pn532_error_t pn532_mifareclassic_AuthenticateBlock (byte_t * pbtCUID, size_t szCUIDLen, uint32_t uiBlockNumber, uint8_t uiKeyType, byte_t * pbtKeys);
pn532_error_t pn532_mifareclassic_ReadDataBlock (uint8_t uiBlockNumber, byte_t * pbtData);
pn532_error_t pn532_mifareclassic_WriteDataBlock (uint8_t uiBlockNumber, byte_t * pbtData);

/* Help functions for Mifare Value Blocks */
pn532_error_t pn532_mifareclassic_CreateValueBlock (uint8_t uiBlockNumber, int32_t value);
pn532_error_t pn532_mifareclassic_ReadValueBlock (uint8_t uiBlockNumber, int32_t * value);
pn532_error_t pn532_mifareclassic_IncrementValueBlock (uint8_t uiBlockNumber, int32_t value);
pn532_error_t pn532_mifareclassic_DecrementValueBlock (uint8_t uiBlockNumber, int32_t value);

#ifdef __cplusplus
}
#endif

#endif
