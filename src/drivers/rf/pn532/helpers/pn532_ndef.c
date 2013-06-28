/**************************************************************************/
/*!
    @file pn532_ndef.c

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

/**************************************************************************
 NDEF
 ====
 The NFC Data Exchange Format (NDEF) is a standardised data format that
 can be used to exchange information between any compatible NFC device
 and another NFC device or tag. The data format consists of NDEF
 Messages and NDEF Records. The standard is maintained by the NFC Forum.

 The NDEF format is used to store and exchange information like URIs,
 plain text, etc., using a commonly understood format. NFC tags like
 Mifare Classic cards can be configured as NDEF tags, and data written
 to them by one NFC device (NDEF Records) can be understood and
 accessed by any other NDEF compatible device. NDEF messages can also
 be used to exchange data between two active NFC devices in
 "peer-to-peer" mode. By adhering to the NDEF data exchange format
 during communication, devices that would otherwise have no meaningful
 knowledge of each other or common language are able to share data in
 an organised, mutually understandable manner.

 Some helpful white papers relating to NDEF are listed below:

 - NFC Data Exchange Format (NDEF) Technical Specification
 Available at: http://www.nfc-forum.org/specs/spec_list/

 - NFC Record Type Definition (RTD) Specification
 Available at: http://www.nfc-forum.org/specs/spec_list/

 - NXP White Paper - NFC Forum Type Tags
 http://www.nxp.com/acrobat_download2/other/identification/173110_NFC_Forum_Type_Tags_WhitePaper.pdf

 NDEF MESSAGES
 =============
 NDEF Messages are the basic "transportation" mechanism for NDEF
 records, with each message containing one or more NDEF Records.

 NDEF Records
 ============
 NDEF Records contain a specific payload, and have the
 following structure that identifies the contents and size of
 the record:

 Bit 7     6       5       4       3       2       1       0
 ------  ------  ------  ------  ------  ------  ------  ------
 [ MB ]  [ ME ]  [ CF ]  [ SR ]  [ IL ]  [        TNF         ]

 [                         TYPE LENGTH                        ]

 [                       PAYLOAD LENGTH                       ]

 [                          ID LENGTH                         ]

 [                         RECORD TYPE                        ]

 [                              ID                            ]

 [                           PAYLOAD                          ]


***************************************************************************/
#include "projectconfig.h"

//#ifdef CFG_PN532

#include "pn532_ndef.h"
#include "../mem_allocator/pn532_mem.h"

/**************************************************************************/
/*!
    Message Begin bit, used to mark the first record within the message
*/
/**************************************************************************/
#define NDEF_HEADER_MB_MASK             ((uint8_t)(0x01U<<7))

/**************************************************************************/
/*!
    Message End bit, used to mark the last record in the message
*/
/**************************************************************************/
#define NDEF_HEADER_ME_MASK             ((uint8_t)(0x01U<<6))

/**************************************************************************/
/*!
    Message Chunk flag, used to indicate if the record is an initial or
    middle chunked record
*/
/**************************************************************************/
#define NDEF_HEADER_CF_MASK             ((uint8_t)(0x01U<<5))

/**************************************************************************/
/*!
    Short Record flag, used to indicate if the record is a short record,
    hence it's length contain 1 octet
*/
/**************************************************************************/
#define NDEF_HEADER_SR_MASK             ((uint8_t)(0x01U<<4))

/**************************************************************************/
/*!
    ID Length Flag, indicates that the record contains an ID field if set
*/
/**************************************************************************/
#define NDEF_HEADER_IL_MASK             ((uint8_t)(0x01U<<3))

/**************************************************************************/
/*!
    Type name format, indicates the structure of the value of the TYPE
    field
*/
/**************************************************************************/
#define NDEF_HEADER_TNF_MASK            ((uint8_t)(0x07U<<0))

#define NDEF_FLAG_MASK                  (0xF8)
#define NDEF_TNF_MASK                   (0x07)
#define NDEF_HEADER_INDEX               (0)
#define NDEF_TYPE_LENGTH_INDEX          (1)
#define NDEF_PAYLOAD_LENGTH_INDEX       (2)
#define NDEF_EXTRACT_FLAGS(header)      ((header) & NDEF_FLAG_MASK)
#define NDEF_EXTRACT_TNF(header)        ((header) & NDEF_TNF_MASK)
#define NDEF_RECORD_HEADER(header, tnf) (((header) & NDEF_FLAG_MASK) | ((tnf) & NDEF_TNF_MASK))

/**************************************************************************/
/*!

*/
/**************************************************************************/
typedef struct NDefRecord_st
{
  uint8_t *pData;                   /** Raw data of entire NDEF record */
  uint32_t length;                  /** Length of entire NDEF record */
  uint32_t idLenIndex;
  uint32_t typeIndex;
  uint32_t idIndex;
  uint32_t payloadIndex;
  uint32_t payloadLen;
  uint8_t bufferAllocated;          /** Indicates whether the buffer *pData
                                        is allocated (and managed) by NDEF
                                        or managed by upper layer */
  struct NDefRecord_st *pNext;
} NdefRecord_t;

/**************************************************************************/
/*!

*/
/**************************************************************************/
pn532_error_t pn532_ndef_create_empty(pn532_ndef_record_t *pNdefRecord)
{
  NdefRecord_t *pRec;

  if (pNdefRecord == NULL)
  {
    return PN532_ERROR_INVALID_PARAM;
  }

  pRec = (NdefRecord_t*) pn532_mem_alloc(sizeof(NdefRecord_t));
  if (pRec == NULL)
  {
    return PN532_ERROR_MEM_INSUFFICIENT;
  }
  memset(pRec, 0, sizeof(NdefRecord_t));
  *pNdefRecord = pRec;

  return PN532_ERROR_NONE;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
pn532_error_t pn532_ndef_createFromValue(pn532_ndef_record_t *pNdefRecord,
  uint8_t tnf, uint8_t *pType, uint8_t typeLength, uint8_t *pId,
  uint8_t idLength, uint8_t *pPayload, uint32_t payloadLength)
{
  pn532_error_t errCode;
  NdefRecord_t *pRec;

  if ((pNdefRecord == NULL) || (pType == NULL) || (typeLength == 0))
  {
    return PN532_ERROR_INVALID_PARAM;
  }
  pRec = (NdefRecord_t*) pn532_mem_alloc(sizeof(NdefRecord_t));
  if (pRec == NULL)
  {
    return PN532_ERROR_MEM_INSUFFICIENT;
  }
  memset(pRec, 0, sizeof(NdefRecord_t));
  errCode = pn532_ndef_update(pRec, tnf, pType, typeLength, pId, idLength,
    pPayload, payloadLength);
  if (errCode != PN532_ERROR_NONE)
  {
    pn532_mem_free(pRec);
    return errCode;
  }
  pn532_ndef_setMB(pRec);
  pn532_ndef_setME(pRec);
  *pNdefRecord = pRec;

  return PN532_ERROR_NONE;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
pn532_error_t pn532_ndef_createFromRawEx(pn532_ndef_record_t *pNdefRecord,
  uint8_t *pBuffer, uint32_t length,
  pn532_ndef_create_buffer_mode_t bufferMode)
{
  pn532_error_t errCode;
  NdefRecord_t *pRec;

  if ((pNdefRecord == NULL) || (pBuffer == 0) || (bufferMode
    < NDEF_CREATE_BUFFER_MODE_DUPLICATE) || (bufferMode
    > NDEF_CREATE_BUFFER_MODE_DELEGATE))
  {
    return PN532_ERROR_INVALID_PARAM;
  }

  pRec = (NdefRecord_t*) pn532_mem_alloc(sizeof(NdefRecord_t));
  if (pRec == NULL)
  {
    return PN532_ERROR_MEM_INSUFFICIENT;
  }
  memset(pRec, 0, sizeof(NdefRecord_t));
  errCode = pn532_ndef_updateFromRawEx(pRec, pBuffer, length, bufferMode);
  if (errCode != PN532_ERROR_NONE)
  {
    pn532_mem_free(pRec);
    return errCode;
  }
  *pNdefRecord = pRec;

  return PN532_ERROR_NONE;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
pn532_error_t pn532_ndef_createFromRaw(pn532_ndef_record_t *pNdefRecord,
  uint8_t *pBuffer, uint32_t length)
{
  return pn532_ndef_createFromRawEx(pNdefRecord, pBuffer, length,
    NDEF_CREATE_BUFFER_MODE_DUPLICATE);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
pn532_error_t pn532_ndef_createFromStream(pn532_ndef_record_t *pNdefRecord,
  pn532_ndef_fetch_data_fn fnFetch, void* pUserData)
{
  pn532_error_t errCode;
  NdefRecord_t *pRec;

  if ((pNdefRecord == NULL) || (fnFetch == NULL))
  {
    return PN532_ERROR_INVALID_PARAM;
  }
  pRec = (NdefRecord_t*) pn532_mem_alloc(sizeof(NdefRecord_t));
  if (pRec == NULL)
  {
    return PN532_ERROR_MEM_INSUFFICIENT;
  }
  memset(pRec, 0, sizeof(NdefRecord_t));
  errCode = pn532_ndef_updateFromStream(pRec, fnFetch, pUserData);
  if (errCode != PN532_ERROR_NONE)
  {
    pn532_mem_free(pRec);
    return errCode;
  }
  *pNdefRecord = pRec;

  return PN532_ERROR_NONE;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void pn532_ndef_destroy(pn532_ndef_record_t ndefRecord)
{
  NdefRecord_t *pRec = (NdefRecord_t*) ndefRecord;
  if (pRec == NULL)
  {
    return;
  }
  if ((pRec->pData != NULL) && (pRec->bufferAllocated))
  {
    pn532_mem_free(pRec->pData);
  }
  pn532_mem_free(pRec);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
pn532_error_t pn532_ndef_update(pn532_ndef_record_t ndefRecord, uint8_t tnf,
  uint8_t *pType, uint8_t typeLength, uint8_t *pId, uint8_t idLength,
  uint8_t *pPayload, uint32_t payloadLength)
{
  NdefRecord_t *pRec = (NdefRecord_t*) ndefRecord;
  uint8_t flags = 0;

  if ((pRec == NULL) || (pType == NULL) || (typeLength == 0))
  {
    return PN532_ERROR_INVALID_PARAM;
  }
  if (pRec->pData != NULL)
  {
    /* Keep the current MB and ME flag */
    flags = pRec->pData[NDEF_HEADER_INDEX] & (NDEF_HEADER_MB_MASK
      | NDEF_HEADER_ME_MASK);
  }
  /* To keep memory peek low, we pn532_mem_free current data before allocating new data */
  if ((pRec->pData != NULL) && (pRec->bufferAllocated))
  {
    pn532_mem_free(pRec->pData);
    pRec->pData = NULL;
    pRec->bufferAllocated = 0;
  }

  if (pId == NULL)
  {
    idLength = 0;
  }
  if (pPayload == NULL)
  {
    payloadLength = 0;
  }

  pRec->length = 1                      /* FLAG + TNF */
  + 1                                   /* TYPE LENGTH */
  + (payloadLength > 0xFF ? 4 : 1)      /* PAYLOAD LENGTH */
  + (idLength > 0 ? 1 + idLength : 0)   /* ID LENGTH + ID */
  + typeLength                          /* TYPE */
  + payloadLength;                      /* PAYLOAD */
  pRec->pData = (uint8_t*) pn532_mem_alloc(pRec->length);
  if (pRec->pData == NULL)
  {
    pRec->bufferAllocated = 0;
    return PN532_ERROR_MEM_INSUFFICIENT;
  }
  pRec->bufferAllocated = 1;

  if (payloadLength < 0xFF)
  {
    flags |= NDEF_HEADER_SR_MASK;
  }
  if (idLength > 0)
  {
    flags |= NDEF_HEADER_IL_MASK;
  }
  pRec->pData[NDEF_HEADER_INDEX] = NDEF_RECORD_HEADER(flags, tnf);
  pRec->pData[NDEF_TYPE_LENGTH_INDEX] = typeLength;

  if (payloadLength < 0xFF)
  {
    pRec->pData[NDEF_PAYLOAD_LENGTH_INDEX] = (uint8_t) payloadLength;
    pRec->idLenIndex = NDEF_PAYLOAD_LENGTH_INDEX + 1;
  }
  else
  {
    pRec->pData[NDEF_PAYLOAD_LENGTH_INDEX] = (uint8_t)(
      (payloadLength >> 24) & 0xFF);
    pRec->pData[NDEF_PAYLOAD_LENGTH_INDEX + 1] = (uint8_t)(
      (payloadLength >> 16) & 0xFF);
    pRec->pData[NDEF_PAYLOAD_LENGTH_INDEX + 2] = (uint8_t)(
      (payloadLength >> 8) & 0xFF);
    pRec->pData[NDEF_PAYLOAD_LENGTH_INDEX + 3] = (uint8_t)(
      payloadLength & 0xFF);
    pRec->idLenIndex = NDEF_PAYLOAD_LENGTH_INDEX + 4;
  }

  if (idLength > 0)
  {
    pRec->pData[pRec->idLenIndex] = idLength;
    pRec->typeIndex = pRec->idLenIndex + 1;
  }
  else
  {
    pRec->typeIndex = pRec->idLenIndex;
  }

  memcpy(pRec->pData + pRec->typeIndex, pType, typeLength);
  pRec->idIndex = pRec->typeIndex + typeLength;

  if (idLength > 0)
  {
    memcpy(pRec->pData + pRec->idIndex, pId, idLength);
    pRec->payloadIndex = pRec->idIndex + idLength;
  }
  else
  {
    pRec->payloadIndex = pRec->idIndex;
  }
  memcpy(pRec->pData + pRec->payloadIndex, pPayload, payloadLength);
  pRec->payloadLen = payloadLength;

  return PN532_ERROR_NONE;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
pn532_error_t pn532_ndef_updateFromRawEx(pn532_ndef_record_t ndefRecord,
  uint8_t *pBuffer, uint32_t length,
  pn532_ndef_create_buffer_mode_t bufferMode)
{
  NdefRecord_t *pRec = (NdefRecord_t*) ndefRecord;
  uint32_t totalLen = 2;
  uint32_t idLenIndex;
  uint32_t typeIndex;
  uint32_t idIndex;
  uint32_t payloadIndex;
  uint32_t payloadLen;

  if ((pRec == NULL) || (pBuffer == NULL) || (bufferMode
    < NDEF_CREATE_BUFFER_MODE_DUPLICATE) || (bufferMode
    > NDEF_CREATE_BUFFER_MODE_DELEGATE))
  {
    return PN532_ERROR_INVALID_PARAM;
  }
  /************Parse input data ********************/
  /* Header + Type */
  totalLen = 2;
  if (length < totalLen)
  {
    return PN532_ERROR_INVALID_PARAM;
  }

  /* Payload Len */
  if (pBuffer[NDEF_HEADER_INDEX] & NDEF_HEADER_SR_MASK)
  {
    if (totalLen + 1 > length)
    {
      return PN532_ERROR_INVALID_PARAM;
    }
    payloadLen = pBuffer[NDEF_PAYLOAD_LENGTH_INDEX];
    totalLen += 1;
  }
  else
  {
    if (totalLen + 4 > length)
    {
      return PN532_ERROR_INVALID_PARAM;
    }
    payloadLen = ((pBuffer[NDEF_PAYLOAD_LENGTH_INDEX] << 24) & 0xFF000000)
      | ((pBuffer[NDEF_PAYLOAD_LENGTH_INDEX + 1] << 16) & 0x00FF0000)
      | ((pBuffer[NDEF_PAYLOAD_LENGTH_INDEX + 2] << 8) & 0x0000FF00)
      | (pBuffer[NDEF_PAYLOAD_LENGTH_INDEX + 3] & 0x000000FF);
    totalLen += 4;
  }
  idLenIndex = totalLen;

  /* ID Len */
  if (pBuffer[NDEF_HEADER_INDEX] & NDEF_HEADER_IL_MASK)
  {
    if (totalLen + 1 > length)
    {
      return PN532_ERROR_INVALID_PARAM;
    }
    totalLen++;
  }
  typeIndex = totalLen;

  /* Type */
  if (totalLen + pBuffer[NDEF_TYPE_LENGTH_INDEX] > length)
  {
    return PN532_ERROR_INVALID_PARAM;
  }
  totalLen += pBuffer[NDEF_TYPE_LENGTH_INDEX];
  idIndex = totalLen;

  /* ID */
  if (pBuffer[NDEF_HEADER_INDEX] & NDEF_HEADER_IL_MASK)
  {
    if (totalLen + pBuffer[idLenIndex] > length)
    {
      return PN532_ERROR_INVALID_PARAM;
    }
    totalLen += pBuffer[idLenIndex];
  }
  payloadIndex = totalLen;

  /* Payload */
  if (totalLen + payloadLen > length)
  {
    return PN532_ERROR_INVALID_PARAM;
  }
  totalLen += payloadLen;

  /************Assign to target record ********************/
  /* To keep memory peek low, we pn532_mem_free current data before allocating new */
  if ((pRec->pData != NULL) && (pRec->bufferAllocated))
  {
    pn532_mem_free(pRec->pData);
    pRec->pData = NULL;
    pRec->bufferAllocated = 0;
  }
  switch (bufferMode)
  {
    case NDEF_CREATE_BUFFER_MODE_DUPLICATE:
      pRec->pData = (uint8_t*) pn532_mem_alloc(length);
      if (pRec->pData == NULL)
      {
        pRec->bufferAllocated = 0;
        pRec->length = 0;
        pRec->idLenIndex = 0;
        pRec->typeIndex = 0;
        pRec->idIndex = 0;
        pRec->payloadIndex = 0;
        pRec->payloadLen = 0;
        return PN532_ERROR_MEM_INSUFFICIENT;
      }
      pRec->bufferAllocated = 1;
      memcpy(pRec->pData, pBuffer, length);

      pRec->length = totalLen;
      pRec->idLenIndex = idLenIndex;
      pRec->typeIndex = typeIndex;
      pRec->idIndex = idIndex;
      pRec->payloadIndex = payloadIndex;
      pRec->payloadLen = payloadLen;

      break;

    case NDEF_CREATE_BUFFER_MODE_REFERENCE:
      pRec->pData = pBuffer;
      pRec->bufferAllocated = 0;

      pRec->length = totalLen;
      pRec->idLenIndex = idLenIndex;
      pRec->typeIndex = typeIndex;
      pRec->idIndex = idIndex;
      pRec->payloadIndex = payloadIndex;
      pRec->payloadLen = payloadLen;
      break;

    case NDEF_CREATE_BUFFER_MODE_DELEGATE:
      pRec->pData = pBuffer;
      pRec->bufferAllocated = 1;

      pRec->length = totalLen;
      pRec->idLenIndex = idLenIndex;
      pRec->typeIndex = typeIndex;
      pRec->idIndex = idIndex;
      pRec->payloadIndex = payloadIndex;
      pRec->payloadLen = payloadLen;
      break;

    default:
      return PN532_ERROR_INVALID_PARAM;
  }

  return PN532_ERROR_NONE;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
pn532_error_t pn532_ndef_updateFromRaw(pn532_ndef_record_t ndefRecord,
  uint8_t *pBuffer, uint32_t length)
{
  return pn532_ndef_updateFromRawEx(ndefRecord, pBuffer, length,
    NDEF_CREATE_BUFFER_MODE_DUPLICATE);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
pn532_error_t pn532_ndef_updateFromStream(pn532_ndef_record_t ndefRecord,
  pn532_ndef_fetch_data_fn fnFetch, void* pUserData)
{
  NdefRecord_t *pRec = (NdefRecord_t*) ndefRecord;
  uint8_t header;
  uint8_t typeLen = 0;
  uint8_t payloadLen = 0;
  uint8_t idLen = 0;

  uint32_t totalLen;
  uint32_t typeIndex;
  uint32_t idIndex;
  uint32_t payloadIndex;

  pn532_error_t err;

  if ((pRec == NULL) || (fnFetch == NULL))
  {
    return PN532_ERROR_INVALID_PARAM;
  }
  if ((pRec->pData != NULL) && (pRec->bufferAllocated))
  {
    pn532_mem_free(pRec->pData);
    pRec->pData = NULL;
    pRec->bufferAllocated = 0;
  }

  err = fnFetch(pUserData, &header, sizeof(header));
  if (err != PN532_ERROR_NONE)
  {
    return err;
  }
  err = fnFetch(pUserData, &typeLen, sizeof(typeLen));
  if (err != PN532_ERROR_NONE)
  {
    return err;
  }
  if (header & NDEF_HEADER_SR_MASK)
  {
    //	typedef pn532_error_t (*pn532_ndef_fetch_data_fn)(void* pUserData,
    //	    uint8_t *out_buff, uint32_t length);
    err = fnFetch(pUserData, &payloadLen, 1);
    totalLen = 3;
  }
  else
  {
    err = fnFetch(pUserData, &payloadLen, 4);
    totalLen = 6;
  }
  if (err != PN532_ERROR_NONE)
  {
    return err;
  }

  if (header & NDEF_HEADER_IL_MASK)
  {
    err = fnFetch(pUserData, &idLen, sizeof(idLen));
  }
  else
  {
    idLen = 0;
  }
  totalLen++;
  typeIndex = totalLen;
  totalLen += typeLen;
  idIndex = totalLen;
  totalLen += idLen;
  payloadIndex = totalLen;
  totalLen += payloadLen;

  /* To keep memory peek low, we pn532_mem_free current data before allocating new data */
  if ((pRec->pData != NULL) && (pRec->bufferAllocated))
  {
    pn532_mem_free(pRec->pData);
    pRec->pData = NULL;
    pRec->bufferAllocated = 0;
  }

  pRec->pData = (uint8_t*) pn532_mem_alloc(totalLen);
  if (pRec->pData != NULL)
  {
    pRec->bufferAllocated = 1;
    pRec->length = totalLen;

    pRec->pData[NDEF_HEADER_INDEX] = header;
    pRec->pData[NDEF_TYPE_LENGTH_INDEX] = typeLen;

    if (header & NDEF_HEADER_SR_MASK)
    {
      pRec->pData[NDEF_PAYLOAD_LENGTH_INDEX] = payloadLen;
      pRec->idLenIndex = NDEF_PAYLOAD_LENGTH_INDEX + 1;
    }
    else
    {
      pRec->pData[NDEF_PAYLOAD_LENGTH_INDEX] = (uint8_t)(
        (payloadLen >> 24) & 0xFF);
      pRec->pData[NDEF_PAYLOAD_LENGTH_INDEX + 1] = (uint8_t)(
        (payloadLen >> 16) & 0xFF);
      pRec->pData[NDEF_PAYLOAD_LENGTH_INDEX + 2] = (uint8_t)(
        (payloadLen >> 8) & 0xFF);
      pRec->pData[NDEF_PAYLOAD_LENGTH_INDEX + 3] = (uint8_t)(
        payloadLen & 0xFF);
      pRec->idLenIndex = NDEF_PAYLOAD_LENGTH_INDEX + 4;
    }
    pRec->pData[pRec->idLenIndex] = idLen;
    pRec->typeIndex = typeIndex;
    pRec->idIndex = idIndex;
    pRec->payloadIndex = payloadIndex;
    pRec->payloadLen = payloadLen;

    err = fnFetch(pUserData, pRec->pData + typeIndex,
      typeLen + idLen + payloadLen);
    if (err != PN532_ERROR_NONE)
    {
      pn532_mem_free(pRec->pData);
      pRec->pData = NULL;
      pRec->bufferAllocated = 0;
      pRec->length = 0;
      pRec->idLenIndex = 0;
      pRec->typeIndex = 0;
      pRec->idIndex = 0;
      pRec->payloadIndex = 0;
      pRec->payloadLen = 0;
      return err;
    }
    return PN532_ERROR_NONE;
  }
  else
  {
    pRec->bufferAllocated = 0;
    pRec->length = 0;
    pRec->idLenIndex = 0;
    pRec->typeIndex = 0;
    pRec->idIndex = 0;
    pRec->payloadIndex = 0;
    pRec->payloadLen = 0;
    return PN532_ERROR_MEM_INSUFFICIENT;
  }
}

/**************************************************************************/
/*                 Getter and Setter functions                            */
/**************************************************************************/

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t pn532_ndef_getMB(pn532_ndef_record_t ndefRecord)
{
  NdefRecord_t *pRec = (NdefRecord_t*) ndefRecord;
  if ((pRec == NULL) || (pRec->pData == NULL))
  {
    return 0;
  }
  if ((pRec->pData[NDEF_HEADER_INDEX] & NDEF_HEADER_MB_MASK)
    == NDEF_HEADER_MB_MASK)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
pn532_error_t pn532_ndef_setMB(pn532_ndef_record_t ndefRecord)
{
  NdefRecord_t *pRec = (NdefRecord_t*) ndefRecord;
  if ((pRec == NULL) || (pRec->pData == NULL))
  {
    return PN532_ERROR_INVALID_PARAM;
  }
  pRec->pData[NDEF_HEADER_INDEX] |= NDEF_HEADER_MB_MASK;

  return PN532_ERROR_NONE;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
pn532_error_t pn532_ndef_clearMB(pn532_ndef_record_t ndefRecord)
{
  NdefRecord_t *pRec = (NdefRecord_t*) ndefRecord;
  if ((pRec == NULL) || (pRec->pData == NULL))
  {
    return PN532_ERROR_INVALID_PARAM;
  }
  pRec->pData[NDEF_HEADER_INDEX] &= (~NDEF_HEADER_MB_MASK);

  return PN532_ERROR_NONE;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t pn532_ndef_getME(pn532_ndef_record_t ndefRecord)
{
  NdefRecord_t *pRec = (NdefRecord_t*) ndefRecord;
  if ((pRec == NULL) || (pRec->pData == NULL))
  {
    return 0;
  }
  if ((pRec->pData[NDEF_HEADER_INDEX] & NDEF_HEADER_ME_MASK)
    == NDEF_HEADER_ME_MASK)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
pn532_error_t pn532_ndef_setME(pn532_ndef_record_t ndefRecord)
{
  NdefRecord_t *pRec = (NdefRecord_t*) ndefRecord;
  if ((pRec == NULL) || (pRec->pData == NULL))
  {
    return PN532_ERROR_INVALID_PARAM;
  }
  pRec->pData[NDEF_HEADER_INDEX] |= NDEF_HEADER_ME_MASK;

  return PN532_ERROR_NONE;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
pn532_error_t pn532_ndef_clearME(pn532_ndef_record_t ndefRecord)
{
  NdefRecord_t *pRec = (NdefRecord_t*) ndefRecord;
  if ((pRec == NULL) || (pRec->pData == NULL))
  {
    return PN532_ERROR_INVALID_PARAM;
  }
  pRec->pData[NDEF_HEADER_INDEX] &= (~NDEF_HEADER_ME_MASK);

  return PN532_ERROR_NONE;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t pn532_ndef_getTNF(pn532_ndef_record_t ndefRecord)
{
  NdefRecord_t *pRec = (NdefRecord_t*) ndefRecord;
  if ((pRec == NULL) || (pRec->pData == NULL))
  {
    return 0;
  }
  return NDEF_EXTRACT_TNF(pRec->pData[NDEF_HEADER_INDEX]);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
const uint8_t* pn532_ndef_getType(pn532_ndef_record_t ndefRecord)
{
  NdefRecord_t *pRec = (NdefRecord_t*) ndefRecord;
  if ((pRec == NULL) || (pRec->pData == NULL))
  {
    return NULL;
  }
  return pRec->pData + pRec->typeIndex;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint32_t pn532_ndef_getTypeLength(pn532_ndef_record_t ndefRecord)
{
  NdefRecord_t *pRec = (NdefRecord_t*) ndefRecord;
  if ((pRec == NULL) || (pRec->pData == NULL))
  {
    return 0;
  }
  return pRec->pData[NDEF_TYPE_LENGTH_INDEX];
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
const uint8_t* pn532_ndef_getId(pn532_ndef_record_t ndefRecord)
{
  NdefRecord_t *pRec = (NdefRecord_t*) ndefRecord;
  if ((pRec == NULL) || (pRec->pData == NULL))
  {
    return NULL;
  }
  if (pRec->pData[NDEF_HEADER_INDEX] & NDEF_HEADER_IL_MASK)
  {
    return pRec->pData + pRec->idIndex;
  }
  else
  {
    return NULL;
  }
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint32_t pn532_ndef_getIdLength(pn532_ndef_record_t ndefRecord)
{
  NdefRecord_t *pRec = (NdefRecord_t*) ndefRecord;
  if ((pRec == NULL) || (pRec->pData == NULL))
  {
    return 0;
  }
  if (pRec->pData[NDEF_HEADER_INDEX] & NDEF_HEADER_IL_MASK)
  {
    return pRec->pData[pRec->idLenIndex];
  }
  else
  {
    return 0;
  }
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
const uint8_t* pn532_ndef_getPayload(pn532_ndef_record_t ndefRecord)
{
  NdefRecord_t *pRec = (NdefRecord_t*) ndefRecord;
  if ((pRec == NULL) || (pRec->pData == NULL))
  {
    return NULL;
  }
  return pRec->pData + pRec->payloadIndex;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint32_t pn532_ndef_getPayloadLength(pn532_ndef_record_t ndefRecord)
{
  NdefRecord_t *pRec = (NdefRecord_t*) ndefRecord;
  if ((pRec == NULL) || (pRec->pData == NULL))
  {
    return 0;
  }
  return pRec->payloadLen;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
const uint8_t* pn532_ndef_getAll(pn532_ndef_record_t ndefRecord)
{
  NdefRecord_t *pRec = (NdefRecord_t*) ndefRecord;
  if ((pRec == NULL) || (pRec->pData == NULL))
  {
    return NULL;
  }
  return pRec->pData;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint32_t pn532_ndef_getLength(pn532_ndef_record_t ndefRecord)
{
  NdefRecord_t *pRec = (NdefRecord_t*) ndefRecord;
  if ((pRec == NULL) || (pRec->pData == NULL))
  {
    return 0;
  }
  return pRec->length;
}

/************************************************************************/
/*                      NDEF Message functions                          */
/************************************************************************/

/**************************************************************************/
/*!

*/
/**************************************************************************/
pn532_ndef_record_t pn532_ndef_getNextRecord(pn532_ndef_record_t ndefRecord)
{
  NdefRecord_t *pRec = (NdefRecord_t*) ndefRecord;
  if (pRec == NULL)
  {
    return NULL;
  }
  return (pn532_ndef_record_t) pRec->pNext;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
pn532_error_t pn532_ndef_linkNextRecord(pn532_ndef_record_t ndefRecord,
  pn532_ndef_record_t nextRecord)
{
  NdefRecord_t *pRec = (NdefRecord_t*) ndefRecord;
  if (pRec == NULL)
  {
    return PN532_ERROR_INVALID_PARAM;
  }
  pRec->pNext = (NdefRecord_t*) nextRecord;
  return PN532_ERROR_NONE;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
pn532_error_t pn532_ndef_appendRecord(pn532_ndef_message_t ndefMesg,
  pn532_ndef_record_t ndefRecord)
{
  NdefRecord_t *pRec = (NdefRecord_t*) ndefMesg;
  if (pRec == NULL)
  {
    return PN532_ERROR_INVALID_PARAM;
  }
  while (pRec->pNext != NULL)
  {
    pRec = pRec->pNext;
  }
  pRec->pNext = (NdefRecord_t*) ndefRecord;
  return PN532_ERROR_NONE;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
pn532_ndef_record_t pn532_ndef_getFirstRecord(pn532_ndef_message_t ndefMesg)
{
  return (pn532_ndef_record_t) ndefMesg;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
pn532_ndef_record_t pn532_ndef_getRecord(pn532_ndef_message_t ndefMesg,
  uint32_t index)
{
  NdefRecord_t *pRec = (NdefRecord_t*) ndefMesg;
  uint32_t i;
  if (pRec == NULL)
  {
    return NULL;
  }

  for (i = 0; i < index; i++)
  {
    pRec = pRec->pNext;
    if (pRec == NULL)
    {
      return NULL;
    }
  }
  return pRec;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint32_t pn532_ndef_getNumberOfRecords(pn532_ndef_message_t ndefMesg)
{
  NdefRecord_t *pRec = (NdefRecord_t*) ndefMesg;
  uint32_t c = 0;
  if (pRec == NULL)
  {
    return 0;
  }
  c = 1;
  while (pRec->pNext != NULL)
  {
    pRec = pRec->pNext;
    c++;
  }
  return c;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
pn532_error_t pn532_ndef_createTextPayload(char *isoCode, char *text,
    char *payloadBuffer, int32_t *bufferLen)
{
  int32_t i = 0;
  *bufferLen = 0;

  if ((isoCode == NULL) || (text == NULL) || (payloadBuffer == NULL))
  {
    return PN532_ERROR_INVALID_PARAM;
  }

  if (strlen(isoCode) > 5)
  {
    /* ISO code is maximum 5 chars, ex. "en-GB" */
    return PN532_ERROR_INVALID_PARAM;
  }

  if (strlen(text) > 256)
  {
    /* ToDo: Throw a more precise error message? */
    return PN532_ERROR_INVALID_PARAM;
  }

  /* Insert the status byte (assumes UTF-8 encoding) */
  i = strlen(isoCode);
  payloadBuffer[0] = i;   /* ISO code length, 0..5 */
  *bufferLen += 1;

  /* Copy isoCode into the payload buffer */
  memcpy(payloadBuffer+1, isoCode, i);
  *bufferLen += i;

  /* Copy text in the payload buffer */
  memcpy(payloadBuffer+i+1, text, strlen(text));
  *bufferLen += strlen(text);

  return PN532_ERROR_NONE;
}

//#endif  // #ifdef CFG_PN532
