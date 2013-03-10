/**************************************************************************/
/*!
    @file pn532_ndef_cards.c

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
#include "projectconfig.h"

#ifdef CFG_PN532

#include "pn532_ndef_cards.h"
#include "pn532_mifare_classic.h"
#include "pn532_ndef.h"
#include "../mem_allocator/pn532_mem.h"

/** used for padding purpose */
#define TLV_TAG_NULL            (uint8_t)0x00

/** the block contain a ndef message*/
#define TLV_TAG_NDEF            (uint8_t)0x03

/* proprieatary information*/
#define TLV_TAG_PROPRIETARY     (uint8_t)0xFD

/** last TLV block in the data area*/
#define TLV_TAG_TERMINATOR      (uint8_t)0xFE

/**Number of Short Sector of Mifare 1k/4k*/
#define NR_SHORTSECTOR          (32)

/**Number of Long Sector of Mifare 4K*/
#define NR_LONGSECTOR           (8)

/**Number of block of Short Sector, is first 16 sector of Mifare 1k/4k*/
#define NR_BLOCK_OF_SHORTSECTOR (4)

/** Number of block of Long Sector, is last 8 sector of Mifare 4K*/
#define NR_BLOCK_OF_LONGSECTOR  (16)

/** Total block of Mifare 1k/4k*/
#define TOTAL_BLOCK_MF1K        (16*4)
#define TOTAL_BLOCK_MF4K        (32*4+8*16)

/** Get Sector trailer block based on Sector                          */
#define BLOCK_NUMBER_OF_SECTOR_TRAILER(sector) (((sector)<NR_SHORTSECTOR)? \
  ((sector)*NR_BLOCK_OF_SHORTSECTOR + NR_BLOCK_OF_SHORTSECTOR-1):\
  (NR_SHORTSECTOR*NR_BLOCK_OF_SHORTSECTOR + (sector-NR_SHORTSECTOR)*NR_BLOCK_OF_LONGSECTOR + NR_BLOCK_OF_LONGSECTOR-1))

/** Get Sector 1st block based on Sector number                         */
#define BLOCK_NUMBER_OF_SECTOR_1ST_BLOCK(sector) (((sector)<NR_SHORTSECTOR)? \
  ((sector)*NR_BLOCK_OF_SHORTSECTOR):\
  (NR_SHORTSECTOR*NR_BLOCK_OF_SHORTSECTOR + (sector-NR_SHORTSECTOR)*NR_BLOCK_OF_LONGSECTOR))

/**Sector number where MAD1 is located*/
#define SECTOR_NUMBER_MAD1      (0)

/**Sector number where MAD2 is located*/
#define SECTOR_NUMBER_MAD2      (16)

/*** Block number of Sector-trailer block of Mad1*/
#define BLOCK_NUMBER_MAD1 BLOCK_NUMBER_OF_SECTOR_TRAILER(SECTOR_NUMBER_MAD1)

/*** Block number of Sector-trailer block of Mad2*/
#define BLOCK_NUMBER_MAD2 BLOCK_NUMBER_OF_SECTOR_TRAILER(SECTOR_NUMBER_MAD2)

/**GPB byte of a sector trailer block buffer*/
#define GPB(buff)               (uint8_t)(buff[9])

/**Access Condition 1st byte*/
#define ACB(buff)               (buff[6])

#define NFC_AID_CLUSTER_CODE    (uint8_t)0xE1
#define NFC_AID_APP_CODE        (uint8_t)0x03
#define NFC_AID                 ((uint16_t)NFC_AID_CLUSTER_CODE<<8 | (uint16_t)NFC_AID_APP_CODE)

/** MIFARE Classic and MIFARE Plus NFC Read/ Write Access Condition */
#define NFC_GPB_RW_ACCESS_RIGHT_GRANTED                    (0x00)
#define NFC_GPB_RW_ACCESS_RIGHT_PROHIBIT        (0x03)
#define NFC_GPB_MINOR_VERSION                   (0x00)  // mapping version 1.0
#define NFC_GPB_MAJOR_VERSION                   (0x01)

#define CAPACITY_MF1K            (1*2*16 + 15*3*16)
#define CAPACITY_MF1K_NDEF       (15*3*16)
#define CAPACITY_MF4K            ((1*2*16 + 31*3*16)+(8*16*16))
#define CAPACITY_MF4K_NDEF       ((31*3*16)+(8*15*16))

#define REMAIN_SIZE_MF1K(sector) ((TOTAL_BLOCK_MF1K  - BLOCK_NUMBER_OF_SECTOR_1ST_BLOCK(sector))/4*3)*16
#define REMAIN_SIZE_MF4K(sector) ((TOTAL_BLOCK_MF4K  - BLOCK_NUMBER_OF_SECTOR_1ST_BLOCK(sector))/4*3)*16

#define CHECK_INVALID_POINTER(p) {if (!p) return PN532_ERROR_INVALID_PARAM;}
#define CHECK_SUCCESS(status)    {if ((status) != PN532_ERROR_NONE) {return (status);}}
#define CHECK_N_BREAK(status)    {if ((status) != PN532_ERROR_NONE) {break;}}

#define CHECK_SUCCESS_FUNCTION(status,func)   {(status) = (func); CHECK_SUCCESS(status);}
#define CHECK_N_BREAK_FUNCTION(status, func)  {(status) = (func); CHECK_N_BREAK(status);}

#define MY_MIN(a,b) (a>b?b:a)

/* Keys used with Mifare Classic Ndef tags */
static const uint8_t KEY_DEFAULT_KEYAB[6]          = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const uint8_t KEY_MAD_PUBLIC_KEYA[6]        = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5};
static const uint8_t KEY_NFC_PUBLIC_KEYA[6]        = {0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7};

uint8_t gGPB;
uint16_t lengOfNdef = 0;

/**************************************************************************/
/*!
    @note   Possible error messages are:

            - PN532_ERROR_INVALID_PARAM
            - PN532_ERROR_WRONGCARDTYPE     (Not Mifare Classic 1K/4K)
            - PN532_ERROR_INVALID_TAG       (Not an NDEF formatted tag)
            - PN532_ERROR_MEM_INSUFFICIENT
*/
/**************************************************************************/
pn532_error_t pn532_ndef_mfc_ndef_write(pTag_t pTag, pn532_ndef_record_t rec,
  uint8_t sector)
{
  pn532_error_t errCode = PN532_ERROR_NONE;
  uint8_t   blockBuffer[16];
  uint16_t  ndefLength;
  uint8_t   *recordBuffer;
  uint8_t   blockBufferIdx;     /* Index while data is being processed in blockBuffer */
  uint16_t  recordBufferIdx;    /* Index while data is being processed in recordBuffer */
  uint8_t   processLength;      /* Number of byte to handle in recordBuffer */
  uint8_t   processSector;
  uint8_t   processBlockNr;
  uint8_t   miscDataLen;
  uint16_t  ndefTlvAdress;

  CHECK_INVALID_POINTER(pTag);
  CHECK_INVALID_POINTER(rec);

  /* Make sure this is a supported tag (Mifare Classic 1K or 4K) */
  if ((pTag->type != TAG_TYPE_MFC_1K) && (pTag->type != TAG_TYPE_MFC_4K))
  {
    return PN532_ERROR_WRONGCARDTYPE;
  }

  if (((pTag->type == TAG_TYPE_MFC_1K) && (sector > 15)) ||
    ((pTag->type == TAG_TYPE_MFC_4K) && (sector > 39)))
  {
    return PN532_ERROR_INVALID_PARAM;
  }

  if ((sector == SECTOR_NUMBER_MAD1) || (sector == SECTOR_NUMBER_MAD2))
  {
    return PN532_ERROR_INVALID_PARAM;
  }

  /* Check if the tag is in NFC format, if not return an error */
  errCode = pn532_ndef_mfc_get_tagType(pTag);

  if (pTag->ndefType != NDEF_TAG_TYPE_NDEF_WRITE_ENABLE)
  {
    return PN532_ERROR_INVALID_TAG;
  }

  /* Scan Ndef Message TLV block */
  errCode = pn532_ndef_mfc_scan_ndef_tlv(pTag, &ndefTlvAdress);
  if(errCode != PN532_ERROR_NONE)
  {
    return errCode;
  }

  /* Get ndefLength */
  ndefLength = pn532_ndef_getLength(rec);

  miscDataLen = ndefTlvAdress % 16;
  blockBufferIdx = (ndefTlvAdress % 16);

  if (ndefLength >= 255)
  {
    miscDataLen += 4; //03 XX XX XX DATA FE
  }
  else
  {
    miscDataLen += 2; //03 XX DATA FE
  }

  switch (pTag->type)
  {
    case TAG_TYPE_MFC_1K:
      if (REMAIN_SIZE_MF1K(sector) < miscDataLen + ndefLength)
      {
        return PN532_ERROR_MEM_INSUFFICIENT;
      }

      break;
    case TAG_TYPE_MFC_4K:
      if (REMAIN_SIZE_MF4K(sector) < miscDataLen + ndefLength)
      {
        return PN532_ERROR_MEM_INSUFFICIENT;
      }
      break;
    default:
      break;
  }

  processBlockNr = ndefTlvAdress / 16;
  errCode = pn532_mifareclassic_AuthenticateBlock(pTag->uid,
          pTag->lenUid,processBlockNr , PN532_MIFARE_CMD_AUTH_A,(byte_t *)KEY_NFC_PUBLIC_KEYA);
  if(errCode != PN532_ERROR_NONE)
  {
    return errCode;
  }

  errCode = pn532_mifareclassic_ReadDataBlock(processBlockNr, blockBuffer);

  if (ndefLength < 255) /* 1 byte length */
  {
    blockBuffer[blockBufferIdx + 1] = (uint8_t)ndefLength;
    blockBufferIdx++;
  }
  else
  {
    blockBuffer[blockBufferIdx + 1] = 0xff;
    blockBuffer[blockBufferIdx + 2] = (uint8_t)((ndefLength >> 8) & 0x00ff);
    blockBuffer[blockBufferIdx + 3] = (uint8_t)(ndefLength & 0x00ff);
    blockBufferIdx += 3;
  }
  blockBufferIdx = miscDataLen;

  recordBuffer = (uint8_t*) pn532_ndef_getAll(rec);
  recordBufferIdx = 0;

  processSector = (processBlockNr/4);  /* For 1st run */

  do
  {
    /* Fill the remaining bytes of the nfc block */
    processLength = MY_MIN(16-blockBufferIdx,ndefLength-recordBufferIdx);
    memcpy(blockBuffer + blockBufferIdx, recordBuffer + recordBufferIdx,
      processLength);

    recordBufferIdx += processLength;
    blockBufferIdx += processLength;

    /* Handle LAST BLOCK of Record */
    if (blockBufferIdx < 16)
    {
      blockBuffer[blockBufferIdx] = TLV_TAG_TERMINATOR;
      /* Fill the rest of block data as zero; */
      memset(blockBuffer + blockBufferIdx + 1, 0x00,
        16 - blockBufferIdx - 1);
    }

    if (processBlockNr == (uint8_t)(BLOCK_NUMBER_OF_SECTOR_1ST_BLOCK(processSector+1)))
    {
      processSector++;

      CHECK_N_BREAK_FUNCTION(errCode,pn532_mifareclassic_AuthenticateBlock(pTag->uid,
          pTag->lenUid,processBlockNr , PN532_MIFARE_CMD_AUTH_A,(byte_t *)KEY_NFC_PUBLIC_KEYA));
    }

    CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_WriteDataBlock(processBlockNr, blockBuffer));

    blockBufferIdx = 0;
    processBlockNr++;

    //skip sector-trailer block
    if (processBlockNr == (uint8_t)BLOCK_NUMBER_OF_SECTOR_TRAILER(processSector))
    {
      processBlockNr++;
    }

  } while (recordBufferIdx < ndefLength);

  if (errCode != PN532_ERROR_NONE)
  {
    return errCode;
  }

  /* Exception: note that when the last byte of a record fits nicely in the last byte
   * of a block, we need to fill TERMINATE TAG in the next block */
  if (((ndefLength + miscDataLen) % 16) == 0)
  {
    blockBuffer[0] = TLV_TAG_TERMINATOR;
    memset(&blockBuffer[1], 0, 15);
    do
    {
      if (processBlockNr == BLOCK_NUMBER_OF_SECTOR_1ST_BLOCK(processSector+1))
      {
        processSector++;

        CHECK_N_BREAK_FUNCTION(errCode,pn532_mifareclassic_AuthenticateBlock(pTag->uid,
            pTag->lenUid,processBlockNr , PN532_MIFARE_CMD_AUTH_A,(byte_t *)KEY_NFC_PUBLIC_KEYA));
      }

      CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_WriteDataBlock(processBlockNr, blockBuffer));
    } while (0);
  }

  return errCode;
}

/**************************************************************************/
/*!
    @brief  Formats a blank Mifare Classic card and an NFC/NDEF tag

    @note   Possible error messages are:

            - PN532_ERROR_INVALID_TAG       (Tag is not blank)
*/
/**************************************************************************/
pn532_error_t pn532_ndef_mfc_nfc_format(pTag_t pTag, Bool isMad2Needed)
{
  pn532_error_t errCode = PN532_ERROR_NONE;
  uint8_t blockBuffer[16];
  uint8_t nfcCrcInfo[2] = { 0x14, 0x01 };
  uint8_t nfcAids[16] = { 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1,
                          0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1 };
  uint8_t madAccessBits[3] = { 0x78, 0x77, 0x88 };
  uint8_t nfcAccessBits[3] = { 0x7F, 0x07, 0x88 };
  uint8_t numOfSector = 16;
  uint8_t idx;

  CHECK_INVALID_POINTER(pTag);

  /* Assumes that the tag is now blank. If tag is not blank, return error */
  if (pTag->ndefType != NDEF_TAG_TYPE_BLANK)
  {
    return PN532_ERROR_INVALID_TAG;
  }

  if ((pTag->type == TAG_TYPE_MFC_4K) && (isMad2Needed == True))
  {
    numOfSector = 40;
  }

  for (idx = 0; idx < numOfSector; idx++)
  {
    /* Change KEY A, accessBits, GPB of each sector */
    CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_AuthenticateBlock(pTag->uid,
        pTag->lenUid,BLOCK_NUMBER_OF_SECTOR_TRAILER(idx), PN532_MIFARE_CMD_AUTH_B,( byte_t *)KEY_DEFAULT_KEYAB));
    if ((idx == 0) || (idx == 16)) //format Mad sector(s)
    {
      /* Write 0x03E1 into MAD1, (MAD2) */
      memcpy(blockBuffer, nfcAids, sizeof(nfcAids));
      CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_WriteDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(idx)) - 1, blockBuffer)); // 2
      if (idx == 0)
      {
        memcpy(blockBuffer, nfcCrcInfo, sizeof(nfcCrcInfo));
        memcpy(blockBuffer + 2, nfcAids, sizeof(nfcAids) - 2);
        CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_WriteDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(idx)) - 2, blockBuffer)); // 1
      }
      else // idx = 16
      {
        memcpy(blockBuffer, nfcAids, sizeof(nfcAids));
        CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_WriteDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(idx)) - 2, blockBuffer)); // 1

        memcpy(blockBuffer, nfcCrcInfo, sizeof(nfcCrcInfo));
        memcpy(blockBuffer + 2, nfcAids, sizeof(nfcAids) - 2);
        CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_WriteDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(idx)) - 3, blockBuffer)); // 0
      }

      memcpy(blockBuffer, KEY_MAD_PUBLIC_KEYA, sizeof(KEY_MAD_PUBLIC_KEYA));
      memcpy(blockBuffer + 6, madAccessBits, sizeof(madAccessBits));
      if (idx == 0)
      {
        blockBuffer[9] = 0xC1;
      }
      else // idx = 16
      {
        blockBuffer[9] = 0xC2;
      }
      memcpy(blockBuffer + 10, KEY_DEFAULT_KEYAB, sizeof(KEY_DEFAULT_KEYAB));
      CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_WriteDataBlock(BLOCK_NUMBER_OF_SECTOR_TRAILER(idx), blockBuffer)); // 3
    }
    else /* Format nfc sector(s) */
    {
      if (idx == 1)
      {
        memset(blockBuffer, 0, sizeof(blockBuffer));
        blockBuffer[0] = 0x00;
        blockBuffer[1] = 0x00;
        blockBuffer[2] = 0x03; // TLV Ndef Message
        blockBuffer[3] = 0x00; // length = 0
        blockBuffer[4] = 0xfe; // end TLV
        CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_WriteDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(idx)) - 3, blockBuffer));
      }
      /* other nfc sector(s) */
      memcpy(blockBuffer, KEY_NFC_PUBLIC_KEYA, sizeof(KEY_NFC_PUBLIC_KEYA));
      memcpy(blockBuffer + 6, nfcAccessBits, sizeof(nfcAccessBits));
      blockBuffer[9] = 0x40;
      memcpy(blockBuffer + 10, KEY_DEFAULT_KEYAB, sizeof(KEY_DEFAULT_KEYAB));
      CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_WriteDataBlock(BLOCK_NUMBER_OF_SECTOR_TRAILER(idx), blockBuffer));
    }
  }
  if (errCode != PN532_ERROR_NONE)
  {
    return errCode;
  }

  pTag->ndefType = NDEF_TAG_TYPE_NDEF_WRITE_ENABLE;

  return PN532_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reformats an NFC/NDEF tag as a blank Mifare Classic card

    @note   Possible error messages are:

            - PN532_ERROR_TAG_UNWRITABLE
*/
/**************************************************************************/
pn532_error_t pn532_ndef_mfc_blank_format(pTag_t pTag)
{
  pn532_error_t errCode = PN532_ERROR_NONE;
  uint8_t blockBuffer[16];
  uint8_t blankAccessBits[3] = { 0xff, 0x07, 0x80 };
  uint8_t idx;
  uint8_t numOfSector = 0;

  CHECK_INVALID_POINTER(pTag);

  /* Make sure that this is an NDEF tag and writeable */
  if ((pTag->ndefType != NDEF_TAG_TYPE_NDEF_WRITE_ENABLE) && (pTag->ndefType
    != NDEF_TAG_TYPE_BLANK))
  {
    return PN532_ERROR_TAG_UNWRITABLE;
  }

  /* Just exit if the card is already blank */
  if (pTag->ndefType == NDEF_TAG_TYPE_BLANK)
  {
    return PN532_ERROR_NONE;
  }

  /* Figure out how many sectors we have */
  if (pTag->type == TAG_TYPE_MFC_1K)
  {
    numOfSector = 16;
  }
  else if (pTag->type == TAG_TYPE_MFC_4K)
  {
    errCode = pn532_mifareclassic_AuthenticateBlock(pTag->uid,
      pTag->lenUid, BLOCK_NUMBER_MAD2, PN532_MIFARE_CMD_AUTH_A,
      (byte_t *) KEY_MAD_PUBLIC_KEYA);
    if (errCode == PN532_ERROR_NONE)
    {
      numOfSector = 40;
    }
  }

  for (idx = 0; idx < numOfSector; idx++)
  {
    /* 1.1 Authenticate MAD sector */
    CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_AuthenticateBlock(pTag->uid,
        pTag->lenUid,BLOCK_NUMBER_OF_SECTOR_TRAILER(idx), PN532_MIFARE_CMD_AUTH_B,( byte_t *)KEY_DEFAULT_KEYAB));

    /* 1.2 Write to other blocks */
    if (idx == 16)
    {
      memset(blockBuffer, 0, sizeof(blockBuffer));
      CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_WriteDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(idx)) - 3, blockBuffer));
    }
    if ((idx == 0) || (idx == 16))
    {
      memset(blockBuffer, 0, sizeof(blockBuffer));
      CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_WriteDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(idx)) - 2, blockBuffer));
    }
    else
    {
      memset(blockBuffer, 0, sizeof(blockBuffer));
      CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_WriteDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(idx)) - 3, blockBuffer));
      CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_WriteDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(idx)) - 2, blockBuffer));
    }
    memset(blockBuffer, 0, sizeof(blockBuffer));
    CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_WriteDataBlock((BLOCK_NUMBER_OF_SECTOR_TRAILER(idx)) - 1, blockBuffer));

    /* 1.3 Change the key */
    memcpy(blockBuffer, KEY_DEFAULT_KEYAB, sizeof(KEY_DEFAULT_KEYAB));
    memcpy(blockBuffer + 6, blankAccessBits, sizeof(blankAccessBits));
    blockBuffer[9] = 0x69;
    memcpy(blockBuffer + 10, KEY_DEFAULT_KEYAB, sizeof(KEY_DEFAULT_KEYAB));

    /* 1.4 Write the trailer block */
    CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_WriteDataBlock(BLOCK_NUMBER_OF_SECTOR_TRAILER(idx), blockBuffer));
  }

  if (errCode != PN532_ERROR_NONE)
  {
    return errCode;
  }

  pTag->ndefType = NDEF_TAG_TYPE_BLANK;

  return PN532_ERROR_NONE;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
pn532_error_t pn532_ndef_mfc_get_tagType(pTag_t pTag)
{
  pn532_error_t errCode = PN532_ERROR_NONE;
  uint8_t nfcAidsTable[40] =  { 0 };
  uint8_t blockBuffer[16];
  uint8_t blockBufferIdx;
  uint16_t idx, blockIdx;

  CHECK_INVALID_POINTER(pTag)

  /* Using do {}while(0) to take the advantage of 'break' and avoid 'goto' */
  pTag->ndefType = NDEF_TAG_UNKNOWN;

  do
  {
    /* 1 Authenticate MAD1 sector with Mad public key A */
    CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_AuthenticateBlock(pTag->uid,
        pTag->lenUid,BLOCK_NUMBER_MAD1, PN532_MIFARE_CMD_AUTH_A,( byte_t *)KEY_MAD_PUBLIC_KEYA))

    CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_ReadDataBlock(BLOCK_NUMBER_MAD1, blockBuffer));

    /* 1.1 check DA bit in GPB, must be 1 indicate that MAD is available */
    if ((GPB(blockBuffer) & 0x01) == 0)
    {
      break;
    }
    /* 1.2 Check the access condition bits */
    if (memcmp(&ACB(blockBuffer), "\x78\x77\x88", 3) == 0)
    {
      pTag->ndefType = NDEF_TAG_TYPE_NDEF_WRITE_ENABLE;
    }
    else if (memcmp(&ACB(blockBuffer), "\x07\x8f\x0f", 3) == 0)
    {
      pTag->ndefType = NDEF_TAG_TYPE_NDEF_READ_ONLY;
    }

    /* 1.3. Fill NFC AIDs into NFC_AIDs table */
    CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_ReadDataBlock(1, blockBuffer));

    for (idx = 2; idx < 16; idx += 2)
    {
      if ((blockBuffer[idx] == NFC_AID_APP_CODE) && (blockBuffer[idx + 1]
        == NFC_AID_CLUSTER_CODE))
      {
        nfcAidsTable[idx / 2] = 1;
      }
    }

    CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_ReadDataBlock(2, blockBuffer));

    for (idx = 0; idx < 16; idx += 2)
    {
      if ((blockBuffer[idx] == NFC_AID_APP_CODE) && (blockBuffer[idx + 1]
        == NFC_AID_CLUSTER_CODE))
      {
        nfcAidsTable[(idx / 2) + 8] = 1;
      }
    }

    /* 2. Authenticate MAD2 sector with Mad public key A */
    if (pTag->type == TAG_TYPE_MFC_4K)
    {
      errCode = pn532_mifareclassic_AuthenticateBlock(pTag->uid, pTag->lenUid,
        BLOCK_NUMBER_MAD2, PN532_MIFARE_CMD_AUTH_A,
        (byte_t *) KEY_MAD_PUBLIC_KEYA);

      if (errCode == PN532_ERROR_NONE)
      {
        errCode = pn532_mifareclassic_ReadDataBlock(BLOCK_NUMBER_MAD2,
          blockBuffer);

        if (errCode == PN532_ERROR_NONE)
        {
          /* 2.1 Check the DA bit in GPB ... must be 1 to indicate that MAD is available */
          if ((GPB(blockBuffer) & 0x01) == 0)
          {
            break;
          }
          /* 2.2 Check the access condition bits */
          if (memcmp(&ACB(blockBuffer), "\x78\x77\x88", 3) == 0)
          {
            pTag->ndefType = NDEF_TAG_TYPE_NDEF_WRITE_ENABLE;
          }
          else if (memcmp(&ACB(blockBuffer), "\x07\x8f\x0f", 3) == 0)
          {
            pTag->ndefType = NDEF_TAG_TYPE_NDEF_READ_ONLY;
          }

          /* Fill NFC AIDs into NFC_AIDs table */
          CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_ReadDataBlock(1, blockBuffer));

          for (idx = 2; idx < 16; idx += 2)
          {
            if ((blockBuffer[idx] == NFC_AID_APP_CODE)
              && (blockBuffer[idx + 1] == NFC_AID_CLUSTER_CODE))
            {
              nfcAidsTable[(idx / 2) + 16] = 1;
            }
          }

          CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_ReadDataBlock(2, blockBuffer));

          for (idx = 0; idx < 16; idx += 2)
          {
            if ((blockBuffer[idx] == NFC_AID_APP_CODE)
              && (blockBuffer[idx + 1] == NFC_AID_CLUSTER_CODE))
            {
              nfcAidsTable[(idx / 2) + 23] = 1;
            }
          }

          CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_ReadDataBlock(2, blockBuffer));

          for (idx = 0; idx < 16; idx += 2)
          {
            if ((blockBuffer[idx] == NFC_AID_APP_CODE)
              && (blockBuffer[idx + 1] == NFC_AID_CLUSTER_CODE))
            {
              nfcAidsTable[(idx / 2) + 32] = 1;
            }
          }
        }
      }
    }

    /* 3. Check NFC sector */
    for (idx = 1; idx < sizeof(nfcAidsTable) / sizeof(nfcAidsTable[0]); idx++)
    {
      if (nfcAidsTable[idx] == 1)
      {
        // to do A
        errCode = pn532_mifareclassic_AuthenticateBlock(pTag->uid,
          pTag->lenUid, BLOCK_NUMBER_OF_SECTOR_TRAILER(idx),
          PN532_MIFARE_CMD_AUTH_A, (byte_t *) KEY_NFC_PUBLIC_KEYA);

        if (errCode != PN532_ERROR_NONE)
        {
          continue;
        }

        /* Check DA bit in GPB, must be 1 indicate that MAD is available */
        errCode = pn532_mifareclassic_ReadDataBlock(
          BLOCK_NUMBER_OF_SECTOR_TRAILER(idx), blockBuffer);

        if (errCode != PN532_ERROR_NONE)
        {
          continue;
        }

        gGPB = GPB(blockBuffer);

        /* Check version bits 4-7 */
        if ((((gGPB >> 4) & 0x03) != NFC_GPB_MINOR_VERSION) || (((gGPB
          >> 6) & 0x03) != NFC_GPB_MAJOR_VERSION))
        {
          break;
        }

        /* Check read access condition */
        if (((gGPB >> 2) & 0x03) != NFC_GPB_RW_ACCESS_RIGHT_GRANTED) /* read access condition support */
        {
          continue;
        }

        /* Check write access condition; R/W condition must be valid: either 00b or 11b */
        if ((((gGPB >> 0) & 0x03) != NFC_GPB_RW_ACCESS_RIGHT_GRANTED) &&
                        (((gGPB >> 0) & 0x03) != NFC_GPB_RW_ACCESS_RIGHT_PROHIBIT))
        {
          continue;
        }



        /* Check Access condition Bits */
        if ((memcmp(&ACB(blockBuffer), "\x7f\x07\x88", 3) == 0)
          && (pTag->ndefType = NDEF_TAG_TYPE_NDEF_WRITE_ENABLE))
        {
          pTag->ndefType = NDEF_TAG_TYPE_NDEF_WRITE_ENABLE;
        }
        else if ((memcmp(&ACB(blockBuffer), "\x07\x8f\x0f", 3) == 0)
          && (pTag->ndefType = NDEF_TAG_TYPE_NDEF_READ_ONLY))
        {
          pTag->ndefType = NDEF_TAG_TYPE_NDEF_READ_ONLY;
        }
        else
        {
          pTag->ndefType = NDEF_TAG_UNKNOWN;
        }

        for(blockIdx = 0; blockIdx < ((idx > 32)?15:3); blockIdx++)
        {
          errCode = pn532_mifareclassic_ReadDataBlock(
          (BLOCK_NUMBER_OF_SECTOR_1ST_BLOCK(idx)) + blockIdx, blockBuffer);

          if (errCode != PN532_ERROR_NONE)
          {
            continue;
          }

          for(blockBufferIdx = 0; blockBufferIdx < 16; blockBufferIdx++)
          {
            /* Check NDEF Message TLV */
            if (blockBuffer[blockBufferIdx] != TLV_TAG_NDEF)
            {
              continue;
            }

            pTag->ndefStartSector = idx;
            /* Check NDEF TLV Length 1 byte or 3 bytes format */
            // if (blockBuffer[3] == 0xff) /* 3 bytes format */
            // {
            //   tlvLength = (((uint16_t) blockBuffer[4] << 8) & 0xff00)
            //     | ((uint16_t) blockBuffer[5] & 0x00ff);
            // }
            // else /* 1 byte format */
            // {
            //   tlvLength = blockBuffer[4];
            // }

            return PN532_ERROR_NONE;
          }
        }
      }
    }
  } while (0);

  /* Detect blank tag */
  /* Authenticate MAD1 sector with the default Mad public key to check blank card */
  if ((pn532_mifareclassic_AuthenticateBlock(pTag->uid, pTag->lenUid, BLOCK_NUMBER_MAD1,
    PN532_MIFARE_CMD_AUTH_B, (byte_t *) KEY_DEFAULT_KEYAB))
    == PN532_ERROR_NONE)
  {
    errCode = pn532_mifareclassic_ReadDataBlock(BLOCK_NUMBER_MAD1,
      blockBuffer);
    if ((GPB(blockBuffer) & 0x01) == 0)
    {
      pTag->ndefType = NDEF_TAG_UNKNOWN;
      return PN532_ERROR_NONE;
    }
    /* Check access condition Bits */
    if (memcmp(&ACB(blockBuffer), "\xff\x07\x80", 3) == 0)
    {
      pTag->ndefType = NDEF_TAG_TYPE_BLANK;
    }
    else
    {
      pTag->ndefType = NDEF_TAG_UNKNOWN;
      return PN532_ERROR_NONE;
    }
    for (idx = 1; idx < 16; idx++)
    {
      if ((pn532_mifareclassic_AuthenticateBlock(pTag->uid, pTag->lenUid,
        BLOCK_NUMBER_OF_SECTOR_TRAILER(idx), PN532_MIFARE_CMD_AUTH_A,
        (byte_t *) KEY_DEFAULT_KEYAB)) != PN532_ERROR_NONE)
      {
        pTag->ndefType = NDEF_TAG_UNKNOWN;
        break;
      }
    }

    /* Check if the tag has MAD2 */
    if (((pn532_mifareclassic_AuthenticateBlock(pTag->uid, pTag->lenUid,
      BLOCK_NUMBER_MAD2, PN532_MIFARE_CMD_AUTH_A,
      (byte_t *) KEY_DEFAULT_KEYAB)) == PN532_ERROR_NONE) && (pTag->type
      = TAG_TYPE_MFC_4K))
    {
      if ( (GPB(blockBuffer) & 0x01) == 0)
      {
        pTag->ndefType = NDEF_TAG_UNKNOWN;
        return PN532_ERROR_NONE;
      }
      /* Check access condition bits */
      if (memcmp(&ACB(blockBuffer), "\xff\x07\x80", 3) == 0)
      {
        pTag->ndefType = NDEF_TAG_TYPE_BLANK;
      }
      else
      {
        pTag->ndefType = NDEF_TAG_UNKNOWN;
        return PN532_ERROR_NONE;
      }
      for (idx = 16; idx < 40; idx++)
      {
        if ((pn532_mifareclassic_AuthenticateBlock(pTag->uid, pTag->lenUid,
          BLOCK_NUMBER_OF_SECTOR_TRAILER(idx),
          PN532_MIFARE_CMD_AUTH_A, (byte_t *) KEY_DEFAULT_KEYAB))
          != PN532_ERROR_NONE)
        {
          pTag->ndefType = NDEF_TAG_UNKNOWN;
          return PN532_ERROR_NONE;
        }
      }
    }
    pTag->ndefType = NDEF_TAG_TYPE_BLANK;
  }

  return errCode;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
pn532_error_t pn532_ndef_mfc_parseNtag(pTag_t pTag, pn532_ndef_record_t *rec)
{
  pn532_error_t errCode = PN532_ERROR_NONE;
  uint8_t *pBuff = NULL;
  uint8_t processSector;
  uint8_t processBlockNr;
  uint8_t processDataLength;
  uint16_t bufferIdx = 0, blockBufferIdx;
  uint16_t bufferLength = 0;
  uint16_t nfcTlvAddress;
  uint8_t blockBuffer[16];

  CHECK_INVALID_POINTER(pTag)

  if ((pTag->type != TAG_TYPE_MFC_1K) && (pTag->type != TAG_TYPE_MFC_4K))
  {
    return PN532_ERROR_INVALID_TAG;
  }

  errCode = pn532_ndef_mfc_scan_ndef_tlv(pTag, &nfcTlvAddress);
  if(errCode != PN532_ERROR_NONE)
  {
    return errCode;
  }

  processSector = nfcTlvAddress/64;
  processBlockNr = nfcTlvAddress/16;
  blockBufferIdx = nfcTlvAddress%16;
  pTag->ndefStartSector = processSector;

  do
  {
    /* if (processBlockNr == BLOCK_NUMBER_OF_SECTOR_1ST_BLOCK(processSector)) */
    CHECK_N_BREAK_FUNCTION(errCode,pn532_mifareclassic_AuthenticateBlock(pTag->uid,
      pTag->lenUid,processBlockNr , PN532_MIFARE_CMD_AUTH_A,(byte_t *)KEY_NFC_PUBLIC_KEYA));

    CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_ReadDataBlock(processBlockNr, blockBuffer));
    if (processBlockNr == (nfcTlvAddress/16))  /* Only do this for the first block */
    {
      uint8_t miscDataLength;

      /* First block ... get Length and data */
      if (blockBuffer[blockBufferIdx + 1] == 0xff)
      {
        bufferLength = ((uint16_t) blockBuffer[blockBufferIdx + 2]) << 8 |
                ((uint16_t) blockBuffer[blockBufferIdx + 3]);
        miscDataLength = blockBufferIdx + 4;
      }
      else
      {
        bufferLength = blockBuffer[blockBufferIdx + 1];
        miscDataLength = blockBufferIdx + 2;
      }

      if(bufferLength)
      {
          pBuff = (uint8_t*) pn532_mem_alloc(bufferLength);
      }
      else
      {
          return PN532_ERROR_NOT_NDEF_CARD;
      }
      if (pBuff == NULL)
      {
        return PN532_ERROR_MEM_INSUFFICIENT;
      }

      processDataLength = MY_MIN(bufferLength-blockBufferIdx,16-miscDataLength);
      memcpy(pBuff, blockBuffer + miscDataLength, processDataLength);
      bufferIdx += processDataLength;
    }
    else
    {
      processDataLength = MY_MIN(bufferLength-bufferIdx,16);
      memcpy(pBuff + bufferIdx, blockBuffer, processDataLength);
      bufferIdx += processDataLength;
    }

    processBlockNr++;

    /* skip sector-trailer block */
    if (processBlockNr == (uint8_t)BLOCK_NUMBER_OF_SECTOR_TRAILER(processSector))
    {
      processSector++;
      processBlockNr++;
    }

  } while (bufferIdx < bufferLength);

  if (errCode == PN532_ERROR_NONE)
  {
    pn532_ndef_createFromRaw(rec, pBuff, bufferLength);
  }

  if (pBuff)
  {
    pn532_mem_free(pBuff);
  }

  return errCode;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
pn532_error_t pn532_ndef_mfc_tagType_identify(uint8_t sak, uint16_t atqa,
  pTag_t pTag)
{
  TagType_t tagType = TAG_TYPE_UNKNOWN;

  CHECK_INVALID_POINTER(pTag)

  /* Check if this is a type 4 tag */
  if (sak & 0x20)
  {
    tagType = TAG_TYPE_UNKNOWN;
    /* ToDo: We can identify more specific type 4 tags here, like desfire, jcop ...*/
  }
  else
  {
    switch (sak)
    {
      /* Infineon Mfc 1k */
      case 0x88:
        tagType = TAG_TYPE_MFC_1K;
        break;
      /* Classic 1K, Mifare Plus 2K cl2, ...*/
      case 0x08:
        if (atqa == (uint16_t)0x0004)
        {
          tagType = TAG_TYPE_MFC_1K;
        }
        break;
      /* Classic 4K,... */
      case 0x18:
        if (atqa == (uint16_t)0x0002)
        {
          tagType = TAG_TYPE_MFC_4K;
        }
        break;
      /* Mini group */
      case 0x09:
        tagType = TAG_TYPE_MF_MINI;       /* REQA==0X0400 */
        break;
      /* Ultralight/c group */
      case 0x00:
        tagType = TAG_TYPE_MF_ULTRALIGHT; /* ATQA = 0X4400 */
        break;
      default:
        break;
    }
  }

  pTag->type = tagType;

  return PN532_ERROR_NONE;
}

/**************************************************************************/
/*!

    @note   Possible error messages are:

            - PN532_ERROR_INVALID_TAG
            - PN532_ERROR_NOT_FOUND_NDEF_TLV
*/
/**************************************************************************/
pn532_error_t pn532_ndef_mfc_scan_ndef_tlv(pTag_t pTag, uint16_t *ndefTlvAddrress)
{
  pn532_error_t errCode = PN532_ERROR_NONE;
  uint8_t sectorSearchRange = 0;
  uint8_t blockSearchRange = 4;
  uint8_t blockBuffer[16];
  uint16_t sectorIdx, blockIdx, blockBufferIdx;
  /* Make sure pTag is an NDEF tag: NDEF tag and read/rw */
  if((pTag->ndefType != NDEF_TAG_TYPE_NDEF_WRITE_ENABLE) && (pTag->ndefType != NDEF_TAG_TYPE_NDEF_READ_ONLY))
  {
    return PN532_ERROR_INVALID_TAG;
  }
  if(pTag->type == TAG_TYPE_MFC_1K)
  {
    sectorSearchRange = 16;
  }
  else if(pTag->type == TAG_TYPE_MFC_4K)
  {
    sectorSearchRange = 40;
  }
  /* Scan all nfc sectors to find Ndef Message TLV */
  for(sectorIdx = 0; sectorIdx < sectorSearchRange; sectorIdx++)
  {
    if((sectorIdx == 0) || (sectorIdx == 16))  /* if idx is mad sector -> continue */
    {
      continue;
    }
    else
    {
      /* Authenticate all nfc sectors using keyA */
      CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_AuthenticateBlock(pTag->uid,
        pTag->lenUid,BLOCK_NUMBER_OF_SECTOR_TRAILER(sectorIdx), PN532_MIFARE_CMD_AUTH_A,(byte_t *)KEY_NFC_PUBLIC_KEYA));

      /* If authentication is OK -> read nfc authenticated sector block to find NDEF Message TLV */
      /* Read all blocks of a sector */
      if(sectorIdx > 31)
      {
        blockSearchRange = 16;
      }
      for(blockIdx = 0; blockIdx < blockSearchRange; blockIdx++)
      {
        CHECK_N_BREAK_FUNCTION(errCode, pn532_mifareclassic_ReadDataBlock
          ((BLOCK_NUMBER_OF_SECTOR_1ST_BLOCK(sectorIdx)) + blockIdx, blockBuffer));
        for(blockBufferIdx = 0; blockBufferIdx < sizeof(blockBuffer); blockBufferIdx++)
        {
          if(blockBuffer[blockBufferIdx] == TLV_TAG_NDEF)
          {
            *ndefTlvAddrress = ((BLOCK_NUMBER_OF_SECTOR_1ST_BLOCK(sectorIdx)) + blockIdx)*16 + blockBufferIdx;
            return PN532_ERROR_NONE;
          }
        } /* Loop to find Ndef messages TLV for blockBuffer */
      } /* loop to find Ndef messages TLV for all blocks of a sector */
    }
  } /* loop to find Ndef message TLV for all sectors */
  return PN532_ERROR_NOT_FOUND_NDEF_TLV;
}

#endif
