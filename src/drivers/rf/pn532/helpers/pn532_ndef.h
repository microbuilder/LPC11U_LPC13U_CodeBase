/**************************************************************************/
/*!
    @file pn532_ndef.h

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
#ifndef __PN532_NDEF_H__
#define __PN532_NDEF_H__

#include "projectconfig.h"
#include "../pn532.h"

/**************************************************************************/
/*!
    Indicates no type, id, or payload is associated with this NDEF Record.

    Type, id and payload fields must all be empty to be a valid
    NDEF_TNF_EMPTY record.
*/
/**************************************************************************/
#define NDEF_TNF_EMPTY                (0x00)

/**************************************************************************/
/*!
    Indicates the type field uses the RTD type name format.

    Use this TNF with RTD types such as NDEF_RTD_TEXT, NDEF_RTD_URI.
*/
/**************************************************************************/
#define NDEF_TNF_WELL_KNOWN            (0x01)

/**************************************************************************/
/*!
    Indicates the type field contains a value that follows the media-type
    BNF construct defined by RFC 2046.
*/
/**************************************************************************/
#define NDEF_TNF_MIME_MEDIA            (0x02)

/**************************************************************************/
/*!
    Indicates the type field contains a value that follows the absolute-URI
    BNF construct defined by RFC 3986.
*/
/**************************************************************************/
#define NDEF_TNF_ABSOLUTE_URI          (0x03)

/**************************************************************************/
/*!
    Indicates the type field contains a value that follows the RTD external
    name specification.

    Note this TNF should not be used with NDEF_RTD_TEXT or NDEF_RTD_URI
    constants.  Those are well known RTD constants, not external RTD
    constants.
*/
/**************************************************************************/
#define NDEF_TNF_EXTERNAL_TYPE         (0x04)

/**************************************************************************/
/*!
    Indicates the payload type is unknown.

    This is similar to the "application/octet-stream" MIME type. The
    payload type is not explicitly encoded within the NDEF Message.

    The type field must be empty to be a valid NDEF_TNF_UNKNOWN record.
*/
/**************************************************************************/
#define NDEF_TNF_UNKNOWN               (0x05)

/**************************************************************************/
/*!
    Indicates the payload is an intermediate or final chunk of a chunked
    NDEF Record.

    The payload type is specified in the first chunk, and subsequent chunks
    must use NDEF_TNF_UNCHANGED with an empty type field.
    NDEF_TNF_UNCHANGED must not be used in any other situation.
*/
/**************************************************************************/
#define NDEF_TNF_UNCHANGED             (0x06)

/**************************************************************************/
/*!
    Reserved TNF type.

    The NFC Forum NDEF Specification v1.0 suggests for NDEF parsers to
    treat this value like NDEF_TNF_UNKNOWN.

    @hide
*/
/**************************************************************************/
#define NDEF_TNF_RESERVED              (0x07)

/**************************************************************************/
/*!
    RTD Text type. For use with NDEF_TNF_WELL_KNOWN.
*/
/**************************************************************************/
#define NDEF_RTD_TEXT                  ("T")

/**************************************************************************/
/*!
    RTD URI type. For use with NDEF_TNF_WELL_KNOWN.
*/
/**************************************************************************/
#define NDEF_RTD_URI                   ("U")

/**************************************************************************/
/*!
    RTD Smart Poster type. For use with NDEF_TNF_WELL_KNOWN.
*/
/**************************************************************************/
#define NDEF_RTD_SMART_POSTER          ("Sp")

/**************************************************************************/
/*!
    RTD Alternative Carrier type. For use with NDEF_TNF_WELL_KNOWN.
*/
/**************************************************************************/
#define NDEF_RTD_ALTERNATIVE_CARRIER   ("ac")

/**************************************************************************/
/*!
    RTD Handover Carrier type. For use with NDEF_TNF_WELL_KNOWN.
*/
/**************************************************************************/
#define NDEF_RTD_HANDOVER_CARRIER      ("Hc")

/**************************************************************************/
/*!
    RTD Handover Request type. For use with NDEF_TNF_WELL_KNOWN.
*/
/**************************************************************************/
#define NDEF_RTD_HANDOVER_REQUEST      ("Hr")

/**************************************************************************/
/*!
    RTD Handover Select type. For use with NDEF_TNF_WELL_KNOWN.
*/
/**************************************************************************/
#define NDEF_RTD_HANDOVER_SELECT       ("Hs")

/**************************************************************************/
/*!
    RTD Android app type. For use with TNF_EXTERNAL.

    The payload of a record with type NDEF_RTD_ANDROID_APP should be the
    package name identifying an application. Multiple NDEF_RTD_ANDROID_APP
    records may be included in a single {@link NdefMessage}.

    Use {@link #createApplicationRecord(String)} to create
    NDEF_RTD_ANDROID_APP records.
*/
/**************************************************************************/
#define NDEF_RTD_ANDROID_APP           ("android.com:pkg")

/**************************************************************************/
/*!
    URI Idenfifier Codes used by URI Records with Well-Known Records (TNF
    Record type 0x01).  These code are inserted before the URI payload to
    help keep the records as short as possible.
*/
/**************************************************************************/
typedef enum
{
  NDEF_WNR_URIPREFIX_NONE           = 0x00,
  NDEF_WNR_URIPREFIX_HTTP_WWWDOT    = 0x01,
  NDEF_WNR_URIPREFIX_HTTPS_WWWDOT   = 0x02,
  NDEF_WNR_URIPREFIX_HTTP           = 0x03,
  NDEF_WNR_URIPREFIX_HTTPS          = 0x04,
  NDEF_WNR_URIPREFIX_TEL            = 0x05,
  NDEF_WNR_URIPREFIX_MAILTO         = 0x06,
  NDEF_WNR_URIPREFIX_FTP_ANONAT     = 0x07,
  NDEF_WNR_URIPREFIX_FTP_FTPDOT     = 0x08,
  NDEF_WNR_URIPREFIX_FTPS           = 0x09,
  NDEF_WNR_URIPREFIX_SFTP           = 0x0A,
  NDEF_WNR_URIPREFIX_SMB            = 0x0B,
  NDEF_WNR_URIPREFIX_NFS            = 0x0C,
  NDEF_WNR_URIPREFIX_FTP            = 0x0D,
  NDEF_WNR_URIPREFIX_DAV            = 0x0E,
  NDEF_WNR_URIPREFIX_NEWS           = 0x0F,
  NDEF_WNR_URIPREFIX_TELNET         = 0x10,
  NDEF_WNR_URIPREFIX_IMAP           = 0x11,
  NDEF_WNR_URIPREFIX_RTSP           = 0x12,
  NDEF_WNR_URIPREFIX_URN            = 0x13,
  NDEF_WNR_URIPREFIX_POP            = 0x14,
  NDEF_WNR_URIPREFIX_SIP            = 0x15,
  NDEF_WNR_URIPREFIX_SIPS           = 0x16,
  NDEF_WNR_URIPREFIX_TFTP           = 0x17,
  NDEF_WNR_URIPREFIX_BTSPP          = 0x18,
  NDEF_WNR_URIPREFIX_BTL2CAP        = 0x19,
  NDEF_WNR_URIPREFIX_BTGOEP         = 0x1A,
  NDEF_WNR_URIPREFIX_TCPOBEX        = 0x1B,
  NDEF_WNR_URIPREFIX_IRDAOBEX       = 0x1C,
  NDEF_WNR_URIPREFIX_FILE           = 0x1D,
  NDEF_WNR_URIPREFIX_URN_EPC_ID     = 0x1E,
  NDEF_WNR_URIPREFIX_URN_EPC_TAG    = 0x1F,
  NDEF_WNR_URIPREFIX_URN_EPC_PAT    = 0x20,
  NDEF_WNR_URIPREFIX_URN_EPC_RAW    = 0x21,
  NDEF_WNR_URIPREFIX_URN_EPC        = 0x22,
  NDEF_WNR_URIPREFIX_URN_NFC        = 0x23
} pn532_ndef_wnr_urirec_ident_t;

/**************************************************************************/
/*!
    PN532 buffer modes (controls the way dynamic memory is managed)
*/
/**************************************************************************/
typedef enum pn532_ndef_create_buffer_mode_en
{
  /** pn532_ndef_create allocate new buffer to store NDEF record and manage
   * it, after calling pn532_ndef_create function, upper layer can change
   * buffer content and free it after using.
   * This is the safest mode, but require more memory*/
  NDEF_CREATE_BUFFER_MODE_DUPLICATE = 0,

  /** pn532_ndef_create refer to input buffer, upper layer should not modify
   * buffer content and has to free it after using buffer */
  NDEF_CREATE_BUFFER_MODE_REFERENCE,

  /** pn532_ndef_create refer and manage input buffer */
  NDEF_CREATE_BUFFER_MODE_DELEGATE
} pn532_ndef_create_buffer_mode_t;

/**************************************************************************/
/*!
    NDEF Structures use to parse NDEF message reading from media
 */
/**************************************************************************/
typedef void* pn532_ndef_record_t;

typedef pn532_ndef_record_t pn532_ndef_message_t;

/**************************************************************************/
/*!
    @brief Call back to fetch data for NDEF parsing

    @see pn532_ndef_parser_stream
 */
/**************************************************************************/
typedef pn532_error_t (*pn532_ndef_fetch_data_fn)(void* pUserData,
  uint8_t *out_buff, uint32_t length);

/**************************************************************************/
/*!
    @brief          Creates an empty NDEF record

    @param[out]     pNdefRecord     pointer to the new record

    @return         pn532_error_t, possible error messages are:

                    PN532_ERROR_INVALID_PARAM
                    PN532_ERROR_MEM_INSUFFICIENT
*/
/**************************************************************************/
pn532_error_t pn532_ndef_create_empty(pn532_ndef_record_t *pNdefRecord);

/**************************************************************************/
/*!
    @brief Create an NDEF record from supplied details

    @param[out]     pNdefRecord     pointer to the new record
    @param[in]      tnf             TNF ID
    @param[in]      pType           record type
    @param[in]      typeLength      length of pType
    @param[in,opt]  pId             record ID
    @param[in]      idLength        length of pId
    @param[in]      pPayload        record payload
    @param[in]      payloadLength   length of pPayload

    @return         pn532_error_t, possible error messages are:

                    PN532_ERROR_INVALID_PARAM
                    PN532_ERROR_MEM_INSUFFICIENT
*/
/**************************************************************************/
pn532_error_t pn532_ndef_createFromValue(pn532_ndef_record_t *pNdefRecord,
  uint8_t tnf, uint8_t *pType, uint8_t typeLength, uint8_t *pId,
  uint8_t idLength, uint8_t *pPayload, uint32_t payloadLength);

/**************************************************************************/
/*!
    @brief  Creates an NDEF record from raw buffer data

    @param[out]     pNdefRecord     pointer to the new record
    @param[in]      pBuffer         buffer containing the raw record data
    @param[in]      length          length of the raw NDEF record data
    @param[in]      bufferMode      buffer management mode

    @return         pn532_error_t, possible error messages are:

                    PN532_ERROR_INVALID_PARAM
                    PN532_ERROR_MEM_INSUFFICIENT

    @see pn532_ndef_create_buffer_mode_t
*/
/**************************************************************************/
pn532_error_t pn532_ndef_createFromRawEx(pn532_ndef_record_t *pNdefRecord,
  uint8_t *pBuffer, uint32_t length,
  pn532_ndef_create_buffer_mode_t bufferMode);

/**************************************************************************/
/*!
    @brief Creates an NDEF record from raw buffer data using the default
           buffer management mode (NDEF_CREATE_BUFFER_MODE_DUPLICATE)

    @param[out]     pNdefRecord     pointer to the new record
    @param[in]      pBuffer         buffer containing the raw record data
    @param[in]      length          length of raw NDEF record data

    @return         pn532_error_t, possible error messages are:

                    PN532_ERROR_INVALID_PARAM
                    PN532_ERROR_MEM_INSUFFICIENT

    @see pn532_ndef_create_buffer_mode_t
*/
/**************************************************************************/
pn532_error_t pn532_ndef_createFromRaw(pn532_ndef_record_t *pNdefRecord,
  uint8_t *pBuffer, uint32_t length);

/**************************************************************************/
/*!
    @brief Creates an NDEF record from a stream (NDEF_CREATE_BUFFER_MODE_REFERENCE)

    @param[out]     pNdefRecord     pointer to the new record
    @param[in]      fnFetch         function to fetch data
    @param[in]      pUserData       User data which will be passed to
                                    the fnFetch callback

    @return         pn532_error_t, possible error messages are:

                    PN532_ERROR_INVALID_PARAM
                    PN532_ERROR_MEM_INSUFFICIENT

    @see pn532_ndef_create_buffer_mode_t
*/
/**************************************************************************/
pn532_error_t pn532_ndef_createFromStream(pn532_ndef_record_t *pNdefRecord,
  pn532_ndef_fetch_data_fn fnFetch, void *pUserData);

/**************************************************************************/
/*!
    @brief Destroys the record, freeing the memory

    @param          ndefRec         NDEF record to destroy
*/
/**************************************************************************/
void pn532_ndef_destroy(pn532_ndef_record_t ndefRecord);

/**************************************************************************/
/*!
    @brief    Updates an NDEF record with new data/values

    @note     Please note that this function frees allocated resources
              before updating any values. If the function fails to allocate
              memory for new record, previous values are discarded
              and the record becomes empty.

    @param[in,out]  ndefRecord      Ndef record to update
    @param[in]      tnf             TNF ID
    @param[in]      pType           record type
    @param[in]      typeLength      length of pType
    @param[in,opt]  pId             record Id
    @param[in]      idLength        length of pId
    @param[in]      pPayload        record payload
    @param[in]      payloadLength   length of pPayload

    @return         pn532_error_t, possible error messages are:

                    PN532_ERROR_INVALID_PARAM
                    PN532_ERROR_MEM_INSUFFICIENT
*/
/**************************************************************************/
pn532_error_t pn532_ndef_update(pn532_ndef_record_t ndefRecord, uint8_t tnf,
  uint8_t *pType, uint8_t typeLength, uint8_t *pId, uint8_t idLength,
  uint8_t *pPayload, uint32_t payloadLength);

/**************************************************************************/
/*!
    @brief    Updates an NDEF record from a raw buffer

    @note     Please note that this function frees allocated resources
              before updating any values. If the function fails to allocate
              memory for new record, previous values are discarded
              and the record becomes empty.

    @param[in,out]  ndefRecord      Ndef record to update
    @param[in]      pBuffer         Buffer containing raw record data
    @param[in]      length          length of data in buffer
    @param[in]      bufferMode      buffer management mode

    @return         pn532_error_t, possible error messages are:

                    PN532_ERROR_INVALID_PARAM
                    PN532_ERROR_MEM_INSUFFICIENT
*/
/**************************************************************************/
pn532_error_t pn532_ndef_updateFromRawEx(pn532_ndef_record_t ndefRecord,
  uint8_t *pBuffer, uint32_t length,
  pn532_ndef_create_buffer_mode_t bufferMode);

/**************************************************************************/
/*!
    @brief    Parse an NDEF record from a buffer using the default buffer
              management mode (NDEF_CREATE_BUFFER_MODE_DUPLICATE)

    @note     Please note that this function frees allocated resources
              before updating any values. If the function fails to allocate
              memory for new record, previous values are discarded
              and the record becomes empty.

    @param[in,out]  ndefRecord      Ndef record to update
    @param[in]      pBuffer         Buffer containing raw record data
    @param[in]      length          length of data in buffer

    @return         pn532_error_t, possible error messages are:

                    PN532_ERROR_INVALID_PARAM
                    PN532_ERROR_MEM_INSUFFICIENT
*/
/**************************************************************************/
pn532_error_t pn532_ndef_updateFromRaw(pn532_ndef_record_t ndefRecord,
  uint8_t *pBuffer, uint32_t length);

/**************************************************************************/
/*!
    @brief Updates an Ndef record from a runtime stream

    @note     Please note that this function frees allocated resources
              before updating any values. If the function fails to allocate
              memory for new record, previous values are discarded
              and the record becomes empty.

    @param[in,out]  ndefRecord      Ndef record to store data
    @param[in]      fnFetch         Function to get data on demand during the
                                    parsing process
    @param[in]      pUserData       User data which will passed to fnFetch
                                    callback

    @return         pn532_error_t, possible error messages are:

                    PN532_ERROR_INVALID_PARAM
                    PN532_ERROR_MEM_INSUFFICIENT
*/
/**************************************************************************/
pn532_error_t pn532_ndef_updateFromStream(pn532_ndef_record_t ndefRecord,
  pn532_ndef_fetch_data_fn fnFetch, void* pUserData);

/**************************************************************************/
/*                 Getter and Setter functions                            */
/**************************************************************************/

/**************************************************************************/
/*!
    @brief Get MB (Message Begin) flag

    @param[in]      ndefRecord      NDEF record to get MB from

    @return         MB state (1 or 0)

    @see pn532_ndef_setMB, pn532_ndef_clearMB
*/
/**************************************************************************/
uint8_t pn532_ndef_getMB(pn532_ndef_record_t ndefRecord);

/**************************************************************************/
/*!
    @brief  Set MB (Message Begin) flag

    @param[in]      ndefRecord      Ndef record to set MB flag in

    @return         pn532_error_t, possible error messages are:

                    PN532_ERROR_INVALID_PARAM

    @see pn532_ndef_getMB, pn532_ndef_clearMB
*/
/**************************************************************************/
pn532_error_t pn532_ndef_setMB(pn532_ndef_record_t ndefRecord);

/**************************************************************************/
/*!
    @brief  Clear MB (Message Begin) flag

    @param[in]      ndefRecord      Ndef record to clear MB flag in

    @return         pn532_error_t, possible error messages are:

                    PN532_ERROR_INVALID_PARAM

    @see pn532_ndef_getMB, pn532_ndef_setMB
*/
/**************************************************************************/
pn532_error_t pn532_ndef_clearMB(pn532_ndef_record_t ndefRecord);

/**************************************************************************/
/*!
    @brief  Gets the ME (Message End) flag

    @param[in]      ndefRecord      NDEF record to get MB from

    @return         ME flag

    @see pn532_ndef_setME, pn532_ndef_clearME
*/
/**************************************************************************/
uint8_t pn532_ndef_getME(pn532_ndef_record_t ndefRecord);

/**************************************************************************/
/*!
    @brief Sets the ME (Message End) flag

    @param[in]      ndefRecord      Ndef record to set ME flag in

    @return         pn532_error_t, possible error messages are:

                    PN532_ERROR_INVALID_PARAM

    @see pn532_ndef_getME, pn532_ndef_clearME
*/
/**************************************************************************/
pn532_error_t pn532_ndef_setME(pn532_ndef_record_t ndefRecord);

/**************************************************************************/
/*!
    @brief Clears the ME (Message End) flag

    @param[in]      ndefRecord      Ndef record to clear ME flag in

    @return         pn532_error_t, possible error messages are:

                    PN532_ERROR_INVALID_PARAM

    @see pn532_ndef_getME, pn532_ndef_setME
*/
/**************************************************************************/
pn532_error_t pn532_ndef_clearME(pn532_ndef_record_t ndefRecord);

/**************************************************************************/
/*!
    @brief  Gets the TNF (Type Name Format)

    @param[in]      ndefRecord      NDEF record to get TNF from

    @return         The TNF value
*/
/**************************************************************************/
uint8_t pn532_ndef_getTNF(pn532_ndef_record_t ndefRecord);

/**************************************************************************/
/*!
    @brief Gets the record type

    @param[in]      ndefRecord      NDEF record to get Type from

    @return         A constant pointer to Type or NULL if no valid Type
                    value was found

    @see pn532_ndef_getTypeLength
*/
/**************************************************************************/
const uint8_t* pn532_ndef_getType(pn532_ndef_record_t ndefRecord);

/**************************************************************************/
/*!
    @brief  Gets the type length

    @param[in]      ndefRecord      NDEF record to get Type length from

    @return         type length or 0 if no valid type value

    @see pn532_ndef_getType
*/
/**************************************************************************/
uint32_t pn532_ndef_getTypeLength(pn532_ndef_record_t ndefRecord);

/**************************************************************************/
/*!
    @brief  Gets the record's ID

    @param[in]      ndefRecord      NDEF record to get ID from

    @return         A constant pointer to ID or NULL if the input record
                    has no ID

    @see pn532_ndef_getIdLength
*/
/**************************************************************************/
const uint8_t* pn532_ndef_getId(pn532_ndef_record_t ndefRecord);

/**
 */
/**************************************************************************/
/*!
    @brief Get the ID length

    @param[in]        ndefRecord    NDEF record to get Id length from

    @return           ID length or 0 if the input record has no ID

    @see pn532_ndef_getId
*/
/**************************************************************************/
uint32_t pn532_ndef_getIdLength(pn532_ndef_record_t ndefRecord);

/**************************************************************************/
/*!
    @brief Get the record payload

    @param[in]        ndefRecord    NDEF record to get payload from

    @return           A constant pointer to payload or NULL if the input
                      record has no valid payload.

    @see pn532_ndef_getPayloadLength
*/
/**************************************************************************/
const uint8_t* pn532_ndef_getPayload(pn532_ndef_record_t ndefRecord);

/**************************************************************************/
/*!
    @brief Get the payload length

    @param[in]        ndefRecord    NDEF record to get Payload length

    @return           Payload length or 0 if the input record has no
                      valid payload

    @see pn532_ndef_getPayload
*/
/**************************************************************************/
uint32_t pn532_ndef_getPayloadLength(pn532_ndef_record_t ndefRecord);

/**************************************************************************/
/*!
    @brief Get the entire NDEF record

    @param[in]      ndefRecord      NDEF record

    @return         constant pointer to the record data
*/
/**************************************************************************/
const uint8_t* pn532_ndef_getAll(pn532_ndef_record_t ndefRecord);

/**************************************************************************/
/*!
    @brief  Get length of entire NDEF record

    @param[in]      ndefRecord      NDEF record to get length

    @return         length of entire NDEF record
*/
/**************************************************************************/
uint32_t pn532_ndef_getLength(pn532_ndef_record_t ndefRecord);

/************************************************************************/
/*                      NDEF Message functions                          */
/************************************************************************/

/**************************************************************************/
/*!
    @brief Gets the next NDEF record in the NDEF message

    @param[in]      ndefRecord      Current NDEF record

    @return         The next NDEF record, or NULL if no record is present

    @see pn532_ndef_linkNextRecord
*/
/**************************************************************************/
pn532_ndef_record_t pn532_ndef_getNextRecord(pn532_ndef_record_t ndefRecord);

/**************************************************************************/
/*!
    @brief  Appends an NDEF record after current NDEF record.<br/>
            The current record should be the last record of the NDEF
            message.

    @param[in]      ndefRecord      Current NDEF record
    @param[in]      nextRecord      Next NDEF record

    @return         pn532_error_t, possible error messages are:

                    PN532_ERROR_INVALID_PARAM

    @see pn532_ndef_linkNextRecord
*/
/**************************************************************************/
pn532_error_t pn532_ndef_linkNextRecord(pn532_ndef_record_t ndefRecord,
  pn532_ndef_record_t nextRecord);

/**************************************************************************/
/*!
    @brief  Appends a new NDEF record to NDEF message.

    @param[in]      ndefMesg        NDEF message
    @param[in]      ndefRecord      new NDEF record, which will be appended
                                    to ndefMesg

    @return         pn532_error_t, possible error messages are:

                    PN532_ERROR_INVALID_PARAM
*/
/**************************************************************************/
pn532_error_t pn532_ndef_appendRecord(pn532_ndef_message_t ndefMesg,
  pn532_ndef_record_t ndefRecord);

/**************************************************************************/
/*!
    @brief  Gest the first record of the NDEF message

    @param[in]      ndefMesg        NDEF message

    @return The first NDEF record or NULL if NDEF message is empty
*/
/**************************************************************************/
pn532_ndef_record_t pn532_ndef_getFirstRecord(pn532_ndef_message_t ndefMesg);

/**************************************************************************/
/*!
    @brief  Gets the record identified by index

    @param[in]      ndefMesg        NDEF message
    @param[in]      index           index of requested record (start by 0)

    @return         NDEF record at index or NULL if NDEF message has less
                    than (index+1) record

    @see pn532_ndef_getNumberOfRecord
*/
/**************************************************************************/
pn532_ndef_record_t pn532_ndef_getRecord(pn532_ndef_message_t ndefMesg,
  uint32_t index);

/**************************************************************************/
/*!
    @brief  Gets the number of records in the NDEF message

    @param[in]      ndefMesg        NDEF message

    @return         Number of NDEF record
*/
/**************************************************************************/
uint32_t pn532_ndef_getNumberOfRecords(pn532_ndef_message_t ndefMesg);

/**************************************************************************/
/*!
    @brief  Creates a payload for a Text 'Well-Known Record Type'

            BYTE    Description
            ----    -------------------------------------------------
            0       Status byte
                    Bit 7     - 0 - UTF-8 encoding
                                1 - UTF-16 encoding
                    Bit 6     - RFU (must be 0)
                    BIT 5..0  - ISO/IANA code length
            1       ISO/IANA language code (ex. 'en' or 'en-GB').
                    Encoding is always in US-ASCII.
            n+1     Actual text encoded in either UTF-8 or UTF-16

    @param[in]      isoCode        char array containing the ISO/IANA code
    @param[in]      text           char array containg the text payload
    @param[in]      payloadBuffer  buffer to hold the text payload
    @param[out]     bufferLen      length of the payloadBuffer

    @return         pn532_error_t, possible error messages are:

                    PN532_ERROR_INVALID_PARAM
*/
/**************************************************************************/
pn532_error_t pn532_ndef_createTextPayload(char *isoCode, char *text,
    char *payloadBuffer, int32_t *bufferLen);

#endif

