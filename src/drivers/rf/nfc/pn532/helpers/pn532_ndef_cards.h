/**************************************************************************/
/*!
    @file pn532_ndef_cards.h

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013 Adafruit Industries (www.adafruit.com)
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
#ifndef PN532_NDEF_CARDS_H_
#define PN532_NDEF_CARDS_H_
#ifdef __cplusplus
extern "C"
{
#endif

#include "../pn532.h"
//#include "../pn532_bus.h"
#include "pn532_mifare.h"
#include "pn532_ndef.h"

typedef unsigned char Bool, *pBool; /* Boolean (True/False) */
#define True            1
#define False      0
/** type to identify a Ndef Tag type*/
typedef enum ndefTagType_en
{
  /*a blank Card/Tag*/
  NDEF_TAG_TYPE_BLANK = 0,
  /*a blank card with default setting from manufacturer*/
  NDEF_TAG_TYPE_AFTER_PRODUCTION = 1,
  /*Ndef formated as a read-only tag*/
  NDEF_TAG_TYPE_NDEF_READ_ONLY = 2,
  /*Ndef formated as a Read-write Tag*/
  NDEF_TAG_TYPE_NDEF_WRITE_ENABLE = 3,
  /*Tag is propietary formated*/
  NDEF_TAG_PROPRIETARY = 4,
  /*Tag in unknown */
  NDEF_TAG_UNKNOWN = 5
} NdefTagType_t;

typedef enum tagType_en
{
  /*MF1 S50 tag*/
  TAG_TYPE_MFC_1K = 0,
  /*MF1 S70 tag*/
  TAG_TYPE_MFC_4K,
  /*MF1 PLUS 60*/
  TAG_TYPE_MFPLUS_X_2K,
  /*MF1 SPLUS 60*/
  TAG_TYPE_MFPLUS_S_2K,
  /*MF1 PLUS 80*/
  TAG_TYPE_MFPLUS_X_4K,
  /*MF1 SPLUS 80*/
  TAG_TYPE_MFPLUS_S_4K,
  /*MIFARE MINI*/
  TAG_TYPE_MF_MINI,
  /*Mifare UL*/
  TAG_TYPE_MF_ULTRALIGHT,
  /*todo: expand to other card type: UL, ULC, Topaz, Desfire, Felica, ...*/
  /*Unknown tag type*/
  TAG_TYPE_UNKNOWN
} TagType_t;

typedef struct MfcTag_str
{
  uint8_t nfcBlockStart;
  uint8_t nrNfcBlock;
} MfcTag_t;

/** tag data structure, hold info for the module to operate*/
typedef struct tag_str
{
  TagType_t type;
  NdefTagType_t ndefType;
  MfcTag_t mfcTagInfo;
  uint8_t uid[8];
  uint8_t lenUid;
  /*First Sector Of Ndef data, include TLV wrapper*/
  uint16_t ndefStartSector;
} Tag_t, *pTag_t;

/**************************************************************************/
/**
 * @brief Tries to write a Ndef record to a NFC formated tag
 * @param[in] pTag   pointer to the tag need to format
 * @param[in] rec    record to write to the tag;
 * @return    Possible error messages are:
 *     PN532_ERROR_BLOCKWRITEFAILED
 *     PN532_ERROR_NONE
 *     PN532_ERROR_INVALID_PARAM
 *    PN532_ERROR_INVALID_TAG
 */
/**************************************************************************/
pn532_error_t pn532_ndef_mfc_ndef_write(pTag_t pTag, pn532_ndef_record_t rec,
  uint8_t sector);

/**************************************************************************/
/**
 * @brief scan NDEF Message TLV
 * @param[in]  pTag   pointer to the tag need to format
 * @param[out] sector   return the sector containing Ndef Message TLV
 * @param[out] blockNumber    record to block sector containing Ndef Message TLV
 * @param[out] blockBufferIdx  record to block sector containing Ndef Message TLV
 * @return    Possible error messages are:
 *     PN532_ERROR_NOT_FOUND_NDEF_TLV
 *     PN532_ERROR_NONE
 *     PN532_ERROR_INVALID_PARAM
 *    PN532_ERROR_INVALID_TAG
 */
/**************************************************************************/
pn532_error_t pn532_ndef_mfc_scan_ndef_tlv(pTag_t pTag, uint16_t *ndefTlvAddrress);

/**************************************************************************/
/**
 *  @brief   Tries to format a Mifare Classic tag to become a NFC card
 *   @param[in] pTag   pointer to the tag need to format
 *   @param[in] isMad2Needed   mad2 input
 *   @return         pn532_error_t, Possible error messages are:
 *     PN532_ERROR_BLOCKWRITEFAILED
 *     PN532_ERROR_INVALID_PARAM
 */
/**************************************************************************/
pn532_error_t pn532_ndef_mfc_nfc_format(pTag_t pTag, Bool isMad2Needed);

/**************************************************************************/
/**
 *  @brief   Tries to format a Mifare Classic tag to become a Blank card
 *   @param[in] pTag   pointer to the tag need to format
 *   @return         pn532_error_t, Possible error messages are:
 *     PN532_ERROR_BLOCKWRITEFAILED
 *     PN532_ERROR_INVALID_PARAM
 */
/**************************************************************************/
pn532_error_t pn532_ndef_mfc_blank_format(pTag_t pTag);

/**************************************************************************/
/**
 * @brief check if a tag is ndef formated or Blank
 * @param[in] pTag   pointer to the tag need to get tag type.
 * @return pn532_error_t
 *    PN532_ERROR_NONE
 */
/**************************************************************************/
pn532_error_t pn532_ndef_mfc_get_tagType(pTag_t pTag);

/**************************************************************************/
/**
 * @brief  Tries to parse a Ndef record in a card an return the record out to
 * user if sucess
 * @param[in] pTag   the tag from what the parser will go thru
 * @param[out] rec  record to return to user.
 * @note: user need to destroy return records in case it is not null
 * @return pn532_error_t
 *     PN532_ERROR_INVALID_PARAM
 *     PN532_ERROR_INVALID_TAG
 *     PN532_ERROR_NONE
 *     PN532_ERROR_BLOCKWRITEFAILED
 */
/**************************************************************************/
pn532_error_t pn532_ndef_mfc_parseNtag(pTag_t pTag, pn532_ndef_record_t *rec);

/**************************************************************************/
/**
 * @brief  Identify a type A card based on its SAK and atqa infor
 * @param[in] sak   Select Acknowledge byte return by the cards
 * @param[in] atqa  answer to REQA data.
 * @param[out] pTAg tag information to be filled to
 * @return pn532_error_t
 *     PN532_ERROR_NONE
 */
/**************************************************************************/
pn532_error_t pn532_ndef_mfc_tagType_identify(uint8_t sak, uint16_t atqa,
  pTag_t pTag);
/**************************************************************************/
/**
 * @brief  reset card when authenticate fail
 * @return none
 */
/**************************************************************************/
void pn532_mifareclassic_reset();

#ifdef __cplusplus
}
#endif
#endif /* PN532_NDEF_CARDS_H_ */

/*@}*/

