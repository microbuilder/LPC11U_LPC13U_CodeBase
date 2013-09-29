/**************************************************************************/
/*!
 @file test_ndef.c
 */
/**************************************************************************/
#define CFG_PN532

#include <string.h>
#include "unity.h"
#include "bget.h"
#include "pn532_ndef.h"
#include "pn532_mem.h"
#include "pn532.h"

uint8_t g_buffer[128];
uint32_t idx = 0;

const char payloadNull[] = "0x00";
const uint8_t referenceNull[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* Standard NDEF message with id 'id0', WELL KNOWN - TEXT type, payload0 */
const char payload0[] = "\02en Lost time is never found again \r\n";
const uint8_t reference0[] =
{
  0xD9, 0x01, 0x25, 0x03, 0x54, 0x69, 0x64, 0x30, 0x02, 0x65, 0x6E, 0x20,
  0x4C, 0x6F, 0x73, 0x74, 0x20, 0x74, 0x69, 0x6D, 0x65, 0x20, 0x69, 0x73,
  0x20, 0x6E, 0x65, 0x76, 0x65, 0x72, 0x20, 0x66, 0x6F, 0x75, 0x6E, 0x64,
  0x20, 0x61, 0x67, 0x61, 0x69, 0x6E, 0x20, 0x0D, 0x0A
};

/* Standard NDEF message with no id, WELL KNOWN - UIR type, payload1 */
const char payload1[] = "\02en I believe that how you feel is very important to how you look - that healthy equals beautiful \r\n";
const uint8_t reference1[] =
{
  0xD1, 0x01, 0x64, 0x55, 0x02, 0x65, 0x6E, 0x20, 0x49, 0x20, 0x62, 0x65,
  0x6C, 0x69, 0x65, 0x76, 0x65, 0x20, 0x74, 0x68, 0x61, 0x74, 0x20, 0x68,
  0x6F, 0x77, 0x20, 0x79, 0x6F, 0x75, 0x20, 0x66, 0x65, 0x65, 0x6C, 0x20,
  0x69, 0x73, 0x20, 0x76, 0x65, 0x72, 0x79, 0x20, 0x69, 0x6D, 0x70, 0x6F,
  0x72, 0x74, 0x61, 0x6E, 0x74, 0x20, 0x74, 0x6F, 0x20, 0x68, 0x6F, 0x77,
  0x20, 0x79, 0x6F, 0x75, 0x20, 0x6C, 0x6F, 0x6F, 0x6B, 0x20, 0x2D, 0x20,
  0x74, 0x68, 0x61, 0x74, 0x20, 0x68, 0x65, 0x61, 0x6C, 0x74, 0x68, 0x79,
  0x20, 0x65, 0x71, 0x75, 0x61, 0x6C, 0x73, 0x20, 0x62, 0x65, 0x61, 0x75,
  0x74, 0x69, 0x66, 0x75, 0x6C, 0x20, 0x0D, 0x0A
};

/* Standard NDEF message with id = 'id0', WELL KNOWN - UIR type, payload2 */
const char payload2[] = "\02en The true sign of intelligence is not knowledge but imagination \r\n";
const uint8_t reference3[] =
{
  0xD9, 0x01, 0x45, 0x03, 0x55, 0x69, 0x64, 0x30, 0x02, 0x65, 0x6E, 0x20,
  0x54, 0x68, 0x65, 0x20, 0x74, 0x72, 0x75, 0x65, 0x20, 0x73, 0x69, 0x67,
  0x6E, 0x20, 0x6F, 0x66, 0x20, 0x69, 0x6E, 0x74, 0x65, 0x6C, 0x6C, 0x69,
  0x67, 0x65, 0x6E, 0x63, 0x65, 0x20, 0x69, 0x73, 0x20, 0x6E, 0x6F, 0x74,
  0x20, 0x6B, 0x6E, 0x6F, 0x77, 0x6C, 0x65, 0x64, 0x67, 0x65, 0x20, 0x62,
  0x75, 0x74, 0x20, 0x69, 0x6D, 0x61, 0x67, 0x69, 0x6E, 0x61, 0x74, 0x69,
  0x6F, 0x6E, 0x20, 0x0D, 0x0A
};

/* Used when creating records from a stream */
pn532_error_t fn_fetch(void* pUserData, uint8_t *out_buff, uint32_t length);

void setUp(void)
{
}

void tearDown(void)
{
}

/**************************************************************************/
/*!
    Create a TEXT RECORD with no ID field following the example in the
    NDEF Text spec

    See: 'Example UTF-8 Encoding' in Text Record Type Definition
         (NFCForum-TS-RTD_Text_1.0 2006-07-24)

    Offset   Content           Explanation
    0        N/A               IL = 0 (no ID), SD = 1 (short frame)
    1        0x01              Length of the record name
    2        0x10              Length of the payload data (16 bytes)
    3        "T"               Binary encoded record type
    4        0x02              Status byte: UTF-8 with two byte IDO code
    5        "en"              ISO code for text
    7        "Hello, World!"   Payload
*/
/**************************************************************************/
void test_ndef_createTextRecord(void)
{
  pn532_ndef_record_t rec;
  pn532_error_t error;
  char payload[16];
  int32_t len;

  /* Create the text payload */
  error = pn532_ndef_createTextPayload("en", "Hello, World!", payload, &len);

  /* Make sure we didn't throw an error */
  TEST_ASSERT_TRUE(error == PN532_ERROR_NONE);
  /* Check payload length (should be 16 bytes) */
  TEST_ASSERT_EQUAL_UINT32(16, len);
  /* Check the ISO/IANA code */
  TEST_ASSERT_EQUAL_MEMORY(payload+1, "en", 2);
  /* Check the text */
  TEST_ASSERT_EQUAL_MEMORY(payload+3, "Hello, World!", 13);

  /* Create a new Text NDEF message with the payload */
  error = pn532_ndef_createFromValue(&rec, NDEF_TNF_WELL_KNOWN,
      (uint8_t*)NDEF_RTD_TEXT, strlen(NDEF_RTD_TEXT), NULL, 0,
      (uint8_t*)payload, sizeof(payload));

  TEST_ASSERT_TRUE(rec != NULL);
  /* Check record length (should be 20) */
  TEST_ASSERT_EQUAL_UINT32(20, pn532_ndef_getLength(rec));
  /* Check message begin bit (should be 1) */
  TEST_ASSERT_EQUAL_UINT8(1, pn532_ndef_getMB(rec));
  /* Check message end bit (should be 1) */
  TEST_ASSERT_EQUAL_UINT8(1, pn532_ndef_getME(rec));
  /* Check TNF bits (should be 001) */
  TEST_ASSERT_EQUAL_UINT8(NDEF_TNF_WELL_KNOWN, pn532_ndef_getTNF(rec));
  /* Check type length (should be 1) */
  TEST_ASSERT_EQUAL_UINT32(1, pn532_ndef_getTypeLength(rec));
  /* Check record type (should be "T") */
  TEST_ASSERT_EQUAL_MEMORY(NDEF_RTD_TEXT, pn532_ndef_getType(rec), 1);
  /* Check ID length (should be 0) */
  TEST_ASSERT_EQUAL_UINT32(0, pn532_ndef_getIdLength(rec));
  /* Check payload length */
  TEST_ASSERT_EQUAL_UINT32(sizeof(payload), pn532_ndef_getPayloadLength(rec));
  /* Check payload */
  TEST_ASSERT_EQUAL_MEMORY(payload, pn532_ndef_getPayload(rec),
      pn532_ndef_getPayloadLength(rec));

  /* Destroy the record */
  pn532_ndef_destroy(rec);
}

/**************************************************************************/
/*!
    Create an NDEF record by passing in some specific values
*/
/**************************************************************************/
void test_ndef_createFromValue(void)
{
  pn532_error_t       errorCode;
  pn532_ndef_record_t rec;

  /* Create a Well Known Text record that should match reference0 */
  errorCode = pn532_ndef_createFromValue(&rec, NDEF_TNF_WELL_KNOWN,
          (uint8_t*)"T", strlen("T"), (uint8_t*)"id0", strlen("id0"),
          (uint8_t*)payload0, strlen(payload0));
  /* Make sure we didn't throw any errors creating the message */
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_NONE, errorCode);
  /* Make sure we have a valid record */
  TEST_ASSERT_TRUE(rec != NULL);
  /* Compare the created message with the reference buffer */
  TEST_ASSERT_EQUAL_MEMORY(reference0, pn532_ndef_getAll(rec),
          pn532_ndef_getLength(rec));
  /* Check record length (should be 45) */
  TEST_ASSERT_EQUAL_UINT32(45, pn532_ndef_getLength(rec));
  /* Check message begin bit (should be 1) */
  TEST_ASSERT_EQUAL_UINT8(1, pn532_ndef_getMB(rec));
  /* Check message end bit (should be 1) */
  TEST_ASSERT_EQUAL_UINT8(1, pn532_ndef_getME(rec));
  /* Check TNF bits (should be 001) */
  TEST_ASSERT_EQUAL_UINT8(NDEF_TNF_WELL_KNOWN, pn532_ndef_getTNF(rec));
  /* Check type length (should be 1) */
  TEST_ASSERT_EQUAL_UINT32(1, pn532_ndef_getTypeLength(rec));
  /* Check ID length (should be 3) */
  TEST_ASSERT_EQUAL_UINT32(3, pn532_ndef_getIdLength(rec));
  /* Check ID , should be 'id0' */
  TEST_ASSERT_EQUAL_MEMORY("id0", pn532_ndef_getId(rec),
          pn532_ndef_getIdLength(rec));
  /* Check payload length */
  TEST_ASSERT_EQUAL_UINT32(strlen(payload0), pn532_ndef_getPayloadLength(rec));
  /* Check payload */
  TEST_ASSERT_EQUAL_MEMORY(payload0, pn532_ndef_getPayload(rec),
          pn532_ndef_getPayloadLength(rec));
  /* Clear up the memory used by this record */
  pn532_ndef_destroy(rec);

  /* Create a Well Known UIR record, no ID field, payload1 */
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_NONE,
          pn532_ndef_createFromValue(&rec,
            NDEF_TNF_WELL_KNOWN, (uint8_t*)"U", strlen("U"),
            NULL, 0, (uint8_t*)payload1, strlen(payload1)));
  /* Make sure we have a record */
  TEST_ASSERT_TRUE(rec != NULL);
  /* Make sure the record and reference buffer match */
  TEST_ASSERT_EQUAL_MEMORY(reference1, pn532_ndef_getAll(rec),
          pn532_ndef_getLength(rec));
  /* Check record length (should be 104) */
  TEST_ASSERT_EQUAL_UINT32(104, pn532_ndef_getLength(rec));
  /* Check message begin bit (should be 1) */
  TEST_ASSERT_EQUAL_UINT8(1, pn532_ndef_getMB(rec));
  /* Check message end bit (should be 1) */
  TEST_ASSERT_EQUAL_UINT8(1, pn532_ndef_getME(rec));
  /* Check TNF bits (should be 001) */
  TEST_ASSERT_EQUAL_UINT8(NDEF_TNF_WELL_KNOWN, pn532_ndef_getTNF(rec));
  /* Check type length (should be 1) */
  TEST_ASSERT_EQUAL_UINT32(1, pn532_ndef_getTypeLength(rec));
  /* Check ID length (should be 0 ... no ID present!) */
  TEST_ASSERT_EQUAL_UINT32(0, pn532_ndef_getIdLength(rec));
  /* Check ID (should be NULL)  */
  TEST_ASSERT_EQUAL_STRING(NULL, pn532_ndef_getId(rec));
  /* Check payload length */
  TEST_ASSERT_EQUAL_UINT32(strlen(payload1), pn532_ndef_getPayloadLength(rec));
  /* Check payload */
  TEST_ASSERT_EQUAL_MEMORY(payload1, pn532_ndef_getPayload(rec),
          pn532_ndef_getPayloadLength(rec));

  /* Intentionally pass in erroneous values to test error checking */
  /* Ndef record = NULL, should be PN532_ERROR_INVALID_PARAM */
  TEST_ASSERT_EQUAL_UINT32(
          PN532_ERROR_INVALID_PARAM,
          pn532_ndef_createFromValue(NULL, NDEF_TNF_WELL_KNOWN,
                  (uint8_t*)"U", strlen("U"), NULL, 0, (uint8_t*)payload1,
                  strlen(payload1)));
  /* Type length = 0, should be PN532_ERROR_INVALID_PARAM */
  TEST_ASSERT_EQUAL_UINT32(
          PN532_ERROR_INVALID_PARAM,
          pn532_ndef_createFromValue(&rec, NDEF_TNF_WELL_KNOWN, (uint8_t*)"U",
                  0, NULL, 0, (uint8_t*)payload1, strlen(payload1)));
  /* Type = NULL, should be PN532_ERROR_INVALID_PARAM */
  TEST_ASSERT_EQUAL_UINT32(
          PN532_ERROR_INVALID_PARAM,
          pn532_ndef_createFromValue(&rec, NDEF_TNF_WELL_KNOWN, NULL,
                  strlen("U"), NULL, 0, (uint8_t*)payload1, strlen(payload1)));
  /* Out of alloc memory, should be PN532_ERROR_MEM_INSUFFICIENT  */
  TEST_ASSERT_EQUAL_UINT32(
          PN532_ERROR_MEM_INSUFFICIENT,
          pn532_ndef_createFromValue(&rec, NDEF_TNF_WELL_KNOWN, (uint8_t*)"U",
                  strlen("U"), NULL, 0, (uint8_t*)payload1, 2000));

  /* Free up any memory used by the record */
  pn532_ndef_destroy(rec);
}

/**************************************************************************/
/*!
    Updates an existing NDEF record
*/
/**************************************************************************/
void test_ndef_update(void)
{
  pn532_ndef_record_t rec;
  pn532_error_t errorCode;

  /* Create a new record */
  errorCode = pn532_ndef_createFromValue(&rec, NDEF_TNF_WELL_KNOWN,
          (uint8_t*)"T", strlen("T"), (uint8_t*)"id0", strlen("id0"),
          (uint8_t*)payload0, strlen(payload0));
  /* Make sure the record is OK */
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_NONE, errorCode);

  /* Update the record with a new payload and well-known record type */
  errorCode = pn532_ndef_update(rec, NDEF_TNF_WELL_KNOWN, (uint8_t*)"U",
          strlen("U"), NULL, 0, (uint8_t*)payload1, strlen(payload1));
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_NONE, errorCode);
  /* Make sure we have a record! */
  TEST_ASSERT_TRUE(rec != NULL);
  /* Compare the update buffer with the reference buffer */
  TEST_ASSERT_EQUAL_MEMORY(reference1, pn532_ndef_getAll(rec),
          pn532_ndef_getLength(rec));
  /* Check record length (should be 104) */
  TEST_ASSERT_EQUAL_UINT32(104, pn532_ndef_getLength(rec));
  /* Check message begin bit (should be 1) */
  TEST_ASSERT_EQUAL_UINT8(1, pn532_ndef_getMB(rec));
  /* Check message end bit (should be 1) */
  TEST_ASSERT_EQUAL_UINT8(1, pn532_ndef_getME(rec));
  /* Check TNF bits (should be 001) */
  TEST_ASSERT_EQUAL_UINT8(NDEF_TNF_WELL_KNOWN, pn532_ndef_getTNF(rec));
  /* Check type length (should be 1) */
  TEST_ASSERT_EQUAL_UINT32(1, pn532_ndef_getTypeLength(rec));
  /* Check ID length (should be 0 ... no id!) */
  TEST_ASSERT_EQUAL_UINT32(0, pn532_ndef_getIdLength(rec));
  /* Check ID (should be NULL) */
  TEST_ASSERT_EQUAL_STRING(NULL, pn532_ndef_getId(rec));
  /* Check payload length */
  TEST_ASSERT_EQUAL_UINT32(strlen(payload1), pn532_ndef_getPayloadLength(rec));
  /* Check payload */
  TEST_ASSERT_EQUAL_MEMORY(payload1, pn532_ndef_getPayload(rec),
          pn532_ndef_getPayloadLength(rec));

  /* Intentionally throw some errors to test error checking */
  /* Missing record reference, should be PN532_ERROR_INVALID_PARAM */
  TEST_ASSERT_EQUAL_UINT32(
          PN532_ERROR_INVALID_PARAM,
          pn532_ndef_update(NULL, NDEF_TNF_WELL_KNOWN, (uint8_t*)"U", strlen("U"),
                  NULL, 0, (uint8_t*)payload1, strlen(payload1)));
  /* Update an ndef rec (no error here!), should be PN532_ERROR_NONE */
  TEST_ASSERT_EQUAL_UINT32(
          PN532_ERROR_NONE,
          pn532_ndef_update(rec, NDEF_TNF_WELL_KNOWN, (uint8_t*)"U", strlen("U"),
                  NULL, 0, (uint8_t*)payload1, strlen(payload1)));
  /* Memory allocation error, should be  PN532_ERROR_MEM_INSUFFICIENT */
  TEST_ASSERT_EQUAL_UINT32(
          PN532_ERROR_MEM_INSUFFICIENT,
          pn532_ndef_update(rec, NDEF_TNF_WELL_KNOWN, (uint8_t*)"U", strlen("U"),
                  NULL, 0, (uint8_t*)payload1, 2000));

  /* Kill the record to free up some memory */
  pn532_ndef_destroy(rec);
}

/**************************************************************************/
/*!
    Create an NDEF message from a raw byte array
*/
/**************************************************************************/
void test_ndef_createFromRaw(void)
{
  pn532_ndef_record_t rec, rec1;
  uint32_t            length;
  pn532_error_t       errorCode;

  /* Try to create a record from the raw data above */
  length = sizeof(reference1) / sizeof(reference1[0]);
  errorCode = pn532_ndef_createFromRaw(&rec, (uint8_t*)reference1, length);
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_NONE, errorCode);
  /* Make sure we have a record! */
  TEST_ASSERT_TRUE(rec != NULL);
  /* Compare the record with the reference buffer */
  TEST_ASSERT_EQUAL_MEMORY(reference1, pn532_ndef_getAll(rec),
          pn532_ndef_getLength(rec));
  /* Make sure we have a record! */
  TEST_ASSERT_TRUE(rec != NULL);
  /* Check record length (should be 104) */
  TEST_ASSERT_EQUAL_UINT32(104, pn532_ndef_getLength(rec));
  /* Check message begin bit (should be 1) */
  TEST_ASSERT_EQUAL_UINT8(1, pn532_ndef_getMB(rec));
  /* Check message end bit (should be 1) */
  TEST_ASSERT_EQUAL_UINT8(1, pn532_ndef_getME(rec));
  /* Check TNF bits (should be 001) */
  TEST_ASSERT_EQUAL_UINT8(NDEF_TNF_WELL_KNOWN, pn532_ndef_getTNF(rec));
  /* Check type length (should be 1) */
  TEST_ASSERT_EQUAL_UINT32(1, pn532_ndef_getTypeLength(rec));
  /* Check ID length (should be 0 ... no id!) */
  TEST_ASSERT_EQUAL_UINT32(0, pn532_ndef_getIdLength(rec));
  /* Check ID ( should be NULL) */
  TEST_ASSERT_EQUAL_STRING(NULL, pn532_ndef_getId(rec));
  /* Check payload length */
  TEST_ASSERT_EQUAL_UINT32(strlen(payload1), pn532_ndef_getPayloadLength(rec));
  /* Check payload */
  TEST_ASSERT_EQUAL_MEMORY(payload1, pn532_ndef_getPayload(rec),
          pn532_ndef_getPayloadLength(rec));
  /* Free up some memory */
  pn532_ndef_destroy(rec);

  /* Intentionally throw some errors */
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_INVALID_PARAM,
          pn532_ndef_createFromRaw(NULL, (uint8_t*)reference1, length));
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_INVALID_PARAM,
          pn532_ndef_createFromRaw(&rec1, NULL, length));
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_MEM_INSUFFICIENT,
          pn532_ndef_createFromRaw(&rec1, (uint8_t*)reference1, 2000));
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void test_ndef_createFromRawEx(void)
{
  pn532_ndef_record_t rec, rec1;
  uint32_t            length;

  length = sizeof(reference1) / sizeof(reference1[0]);
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_NONE,
          pn532_ndef_createFromRawEx(&rec, (uint8_t*)reference1, length,
                  NDEF_CREATE_BUFFER_MODE_REFERENCE));
  /* Compare created record and reference buffer */
  TEST_ASSERT_EQUAL_MEMORY(reference1, pn532_ndef_getAll(rec),
          pn532_ndef_getLength(rec));
  /* Make sure we have a record! */
  TEST_ASSERT_TRUE(rec != NULL);
  /* Check record length (should be 104) */
  TEST_ASSERT_EQUAL_UINT32(104, pn532_ndef_getLength(rec));
  /* Check message begin bit (should be 1) */
  TEST_ASSERT_EQUAL_UINT8(1, pn532_ndef_getMB(rec));
  /* Check message end bit (should be 1) */
  TEST_ASSERT_EQUAL_UINT8(1, pn532_ndef_getME(rec));
  /* Check TNF bits (should be 001) */
  TEST_ASSERT_EQUAL_UINT8(NDEF_TNF_WELL_KNOWN, pn532_ndef_getTNF(rec));
  /* Check type length (should be 1) */
  TEST_ASSERT_EQUAL_UINT32(1, pn532_ndef_getTypeLength(rec));
  /* Check ID length (should be 0 ... no ID!) */
  TEST_ASSERT_EQUAL_UINT32(0, pn532_ndef_getIdLength(rec));
  /* Check ID ( should be NULL) */
  TEST_ASSERT_EQUAL_STRING(NULL, pn532_ndef_getId(rec));
  /* Check payload length */
  TEST_ASSERT_EQUAL_UINT32(strlen(payload1), pn532_ndef_getPayloadLength(rec));
  /* Check payload */
  TEST_ASSERT_EQUAL_MEMORY(payload1, pn532_ndef_getPayload(rec),
          pn532_ndef_getPayloadLength(rec));
  /* Free up some memory */
  pn532_ndef_destroy(rec);

  /* Intentionally throw some errors! */
  /* Memory allocation error, should be PN532_ERROR_MEM_INSUFFICIENT */
  TEST_ASSERT_EQUAL_UINT32(
          PN532_ERROR_MEM_INSUFFICIENT,
          pn532_ndef_createFromRawEx(&rec1, (uint8_t*)reference1, 1000,
                  NDEF_CREATE_BUFFER_MODE_DUPLICATE));
  /* pNdefRecord = NULL, should be PN532_ERROR_INVALID_PARAM */
  TEST_ASSERT_EQUAL_UINT32(
          PN532_ERROR_INVALID_PARAM,
          pn532_ndef_createFromRawEx(NULL, (uint8_t*)reference1, length,
                  NDEF_CREATE_BUFFER_MODE_DUPLICATE));
  /* pBuffer = NULL, should be PN532_ERROR_INVALID_PARAM */
  TEST_ASSERT_EQUAL_UINT32(
          PN532_ERROR_INVALID_PARAM,
          pn532_ndef_createFromRawEx(&rec1, NULL, length,
                  NDEF_CREATE_BUFFER_MODE_DUPLICATE));
  /* Wrong mode, should be PN532_ERROR_INVALID_PARAM */
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_INVALID_PARAM,
          pn532_ndef_createFromRawEx(&rec1, (uint8_t*)reference1, length, 5));
}

/**************************************************************************/
/*!
    Update a message from a raw byte array
*/
/**************************************************************************/
void test_ndef_updateFromRaw(void)
{
  pn532_error_t       errorCode;
  uint32_t            length;
  pn532_ndef_record_t rec;
  pn532_ndef_record_t rec1;

  /* First create a text record in rec */
  errorCode = pn532_ndef_createFromValue(&rec, NDEF_TNF_WELL_KNOWN,
          (uint8_t*)"T", strlen("T"), (uint8_t*)"id0", strlen("id0"),
          (uint8_t*)payload0, strlen(payload0));
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_NONE, errorCode);
  /* Also create a record in rec1 */
  errorCode = pn532_ndef_createFromValue(&rec1, NDEF_TNF_WELL_KNOWN,
          (uint8_t*)"T", strlen("T"), (uint8_t*)"id0", strlen("id0"),
          (uint8_t*)payload0, strlen(payload0));
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_NONE, errorCode);

  /* Try to update rec with the raw byte array reference1 */
  length = sizeof(reference1) / sizeof(reference1[0]);
  errorCode = pn532_ndef_updateFromRawEx(rec, (uint8_t*)reference1,
          length, NDEF_CREATE_BUFFER_MODE_REFERENCE);
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_NONE, errorCode);
  /* Make sure we have a record! */
  TEST_ASSERT_TRUE(rec != NULL);
  /* Compare the contents with the raw byte array */
  TEST_ASSERT_EQUAL_MEMORY(reference1, pn532_ndef_getAll(rec),
          pn532_ndef_getLength(rec));
  /* Check record length (should be 104) */
  TEST_ASSERT_EQUAL_UINT32(104, pn532_ndef_getLength(rec));
  /* Check message begin bit (should be 1) */
  TEST_ASSERT_EQUAL_UINT8(1, pn532_ndef_getMB(rec));
  /* Check message end bit (should be 1) */
  TEST_ASSERT_EQUAL_UINT8(1, pn532_ndef_getME(rec));
  /* Check TNF bits (should be 001) */
  TEST_ASSERT_EQUAL_UINT8(NDEF_TNF_WELL_KNOWN, pn532_ndef_getTNF(rec));
  /* Check type length (should be 1) */
  TEST_ASSERT_EQUAL_UINT32(1, pn532_ndef_getTypeLength(rec));
  /* Check ID length (should be 0 ... no id!) */
  TEST_ASSERT_EQUAL_UINT32(0, pn532_ndef_getIdLength(rec));
  /* Check ID ( should be NULL) */
  TEST_ASSERT_EQUAL_STRING(NULL, pn532_ndef_getId(rec));
  /* Check payload length */
  TEST_ASSERT_EQUAL_UINT32(strlen(payload1), pn532_ndef_getPayloadLength(rec));
  /* Check payload */
  TEST_ASSERT_EQUAL_MEMORY(payload1, pn532_ndef_getPayload(rec),
          pn532_ndef_getPayloadLength(rec));

  /* Intentionally throw some errors! */
  /* Missing record reference, should be PN532_ERROR_INVALID_PARAM  */
  TEST_ASSERT_EQUAL_UINT32(
          PN532_ERROR_INVALID_PARAM,
          pn532_ndef_updateFromRawEx(NULL, (uint8_t*)reference1, length,
                  NDEF_CREATE_BUFFER_MODE_REFERENCE));
  /* Length = 1, should be PN532_ERROR_INVALID_PARAM  */
  TEST_ASSERT_EQUAL_UINT32(
          PN532_ERROR_INVALID_PARAM,
          pn532_ndef_updateFromRawEx(rec, (uint8_t*)reference1, 1,
                  NDEF_CREATE_BUFFER_MODE_REFERENCE));
  /* Length = 2, should be PN532_ERROR_INVALID_PARAM  */
  /* reference1 is a short NDEF record, so length must be larger than 3 */
  TEST_ASSERT_EQUAL_UINT32(
          PN532_ERROR_INVALID_PARAM,
          pn532_ndef_updateFromRawEx(rec, (uint8_t*)reference1, 2,
                  NDEF_CREATE_BUFFER_MODE_REFERENCE));
  /* Length = 5, should be PN532_ERROR_INVALID_PARAM  */
  /* reference0 is a short record with an ID field, length must be larger than 5 */
  TEST_ASSERT_EQUAL_UINT32(
          PN532_ERROR_INVALID_PARAM,
          pn532_ndef_updateFromRawEx(rec, (uint8_t*)reference0, 5,
                  NDEF_CREATE_BUFFER_MODE_REFERENCE));
  /* Length = 10, should be PN532_ERROR_INVALID_PARAM  */
  /* Assign an invalid length value ... */
  /* length = 10 < length of payload = 0x13 => should be PN532_ERROR_INVALID_PARAM  */
  TEST_ASSERT_EQUAL_UINT32(
          PN532_ERROR_INVALID_PARAM,
          pn532_ndef_updateFromRawEx(rec, (uint8_t*)reference0, 10,
                  NDEF_CREATE_BUFFER_MODE_REFERENCE));
  /* Memory allocation error, should be PN532_ERROR_MEM_INSUFFICIENT */
  TEST_ASSERT_EQUAL_UINT32(
          PN532_ERROR_MEM_INSUFFICIENT,
          pn532_ndef_updateFromRawEx(rec1, (uint8_t*)reference0, 2000,
                  NDEF_CREATE_BUFFER_MODE_DUPLICATE));
}

/**************************************************************************/
/*!
    Create messages from a stream
*/
/**************************************************************************/
void test_ndef_createFromStream(void)
{
  pn532_error_t       errorCode;
  pn532_ndef_record_t rec;

  errorCode = pn532_ndef_createFromStream(&rec, fn_fetch, NULL);
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_NONE, errorCode);
  TEST_ASSERT_EQUAL_MEMORY(referenceNull, pn532_ndef_getAll(rec),
          pn532_ndef_getLength(rec));
  TEST_ASSERT_TRUE(rec != NULL);
  /* Check record length (should be 7) */
  TEST_ASSERT_EQUAL_UINT32(7, pn532_ndef_getLength(rec));
  /* Check message begin bit (should be 0 ... no message) */
  TEST_ASSERT_EQUAL_UINT8(0, pn532_ndef_getMB(rec));
  /* Check message end bit (should be 0 ... no message) */
  TEST_ASSERT_EQUAL_UINT8(0, pn532_ndef_getME(rec));
  /* Check TNF bits (should be 000) */
  TEST_ASSERT_EQUAL_UINT8(0, pn532_ndef_getTNF(rec));
  /* Check type length (should be 0) */
  TEST_ASSERT_EQUAL_UINT32(0, pn532_ndef_getTypeLength(rec));
  /* Check ID length (should be 0) */
  TEST_ASSERT_EQUAL_UINT32(0, pn532_ndef_getIdLength(rec));
  /* Check ID ( should be NULL) */
  TEST_ASSERT_EQUAL_STRING(NULL, pn532_ndef_getId(rec));
  /* Check payload length, no payload to test! */
  TEST_ASSERT_EQUAL_UINT32(0, pn532_ndef_getPayloadLength(rec));
  /* Free up some memory */
  pn532_ndef_destroy(rec);

  /* Throw some errors! */
  /* fnFetch = NULL, should be PN532_ERROR_INVALID_PARAM  */
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_INVALID_PARAM,
          pn532_ndef_createFromStream(&rec, NULL, NULL));
  /* pNdefRecord = NULL, should be PN532_ERROR_INVALID_PARAM  */
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_INVALID_PARAM,
          pn532_ndef_createFromStream(NULL, fn_fetch, NULL));
}

#if 0

/**************************************************************************/
/*!
    Link record tests
*/
/**************************************************************************/
void test_ndef_linkNextRecord(void)
{
  pn532_error_t errorCode;
  pn532_ndef_record_t rec, rec1;

  /* Create a first record */
  errorCode = pn532_ndef_createFromValue(&rec, NDEF_TNF_WELL_KNOWN,
          (uint8_t*)"T", strlen("T"), (uint8_t*)"id0", strlen("id0"),
          (uint8_t*)payload0, strlen(payload0));
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_NONE, errorCode);
  TEST_ASSERT_TRUE(rec != NULL);
  TEST_ASSERT_EQUAL_MEMORY(reference0, pn532_ndef_getAll(rec),
          pn532_ndef_getLength(rec));

  /* Create a second record */
  errorCode = pn532_ndef_createFromValue(&rec1, NDEF_TNF_WELL_KNOWN,
          (uint8_t*)"U", strlen("U"), NULL, 0, (uint8_t*)payload1,
          strlen(payload1));
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_NONE, errorCode);
  TEST_ASSERT_TRUE(rec1 != NULL);
  TEST_ASSERT_EQUAL_MEMORY(reference1, pn532_ndef_getAll(rec1),
          pn532_ndef_getLength(rec1));

  /* Try to link rec1 to rec */
  /* should be ERROR_NONE */
  TEST_ASSERT_EQUAL_UINT32(ERROR_NONE, pn532_ndef_linkNextRecord(rec, rec1));

  /* Intentionally throw an error */
  /* Should be PN532_ERROR_INVALID_PARAM */
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_INVALID_PARAM,
          pn532_ndef_linkNextRecord(NULL, rec1));

  /* Free up some memory */
  pn532_ndef_destroy(rec);
  pn532_ndef_destroy(rec1);
}

/**************************************************************************/
/*!
    Get next record tests
*/
/**************************************************************************/
void test_ndef_getNextRecord(void)
{
  pn532_error_t errorCode;
  pn532_ndef_record_t rec, rec1, recTemp;

  /* Create a first record */
  errorCode = pn532_ndef_createFromValue(&rec, NDEF_TNF_WELL_KNOWN,
          (uint8_t*)"T", strlen("T"), (uint8_t*)"id0", strlen("id0"),
          (uint8_t*)payload0, strlen(payload0));
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_NONE, errorCode);

  /* Create a second record */
  errorCode = pn532_ndef_createFromValue(&rec1, NDEF_TNF_WELL_KNOWN,
          (uint8_t*)"U", strlen("U"), NULL, 0, (uint8_t*)payload1,
          strlen(payload1));
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_NONE, errorCode);

  /* Try to link rec1 to rec */
  /* Check error code return , should be ERROR_NONE */
  TEST_ASSERT_EQUAL_UINT32(ERROR_NONE, pn532_ndef_linkNextRecord(rec, rec1));
  /* Fill recTemp with next Record in rec */
  recTemp = pn532_ndef_getNextRecord(rec);
  /* Make sure we have a record! */
  TEST_ASSERT_TRUE(recTemp != NULL);
  /* Parse recTemp and compare it to the expected data */
  TEST_ASSERT_EQUAL_MEMORY(reference1, pn532_ndef_getAll(recTemp),
          pn532_ndef_getLength(recTemp));
  /* Check record length (should be 104) */
  TEST_ASSERT_EQUAL_UINT32(104, pn532_ndef_getLength(recTemp));
  /* Check message begin bit (should be 1) */
  TEST_ASSERT_EQUAL_UINT8(1, pn532_ndef_getMB(recTemp));
  /* Check message end bit (should be 1) */
  TEST_ASSERT_EQUAL_UINT8(1, pn532_ndef_getME(recTemp));
  /* Check TNF bits (should be 001) */
  TEST_ASSERT_EQUAL_UINT8(NDEF_TNF_WELL_KNOWN, pn532_ndef_getTNF(recTemp));
  /* Check type length (should be 1) */
  TEST_ASSERT_EQUAL_UINT32(1, pn532_ndef_getTypeLength(recTemp));
  /* Check ID length (should be 0 ... no ID!) */
  TEST_ASSERT_EQUAL_UINT32(0, pn532_ndef_getIdLength(recTemp));
  /* Check ID ( should be NULL) */
  TEST_ASSERT_EQUAL_STRING(NULL, pn532_ndef_getId(recTemp));
  /* Check payload length */
  TEST_ASSERT_EQUAL_UINT32(strlen(payload1),
          pn532_ndef_getPayloadLength(recTemp));
  /* Check payload */
  TEST_ASSERT_EQUAL_MEMORY(payload1, pn532_ndef_getPayload(recTemp),
          pn532_ndef_getPayloadLength(recTemp));

  /* Intentionally throw some errors! */
  /* Check return of pn532_ndef_getNextRecord(NULL), should be NULL  */
  TEST_ASSERT_EQUAL(NULL, pn532_ndef_getNextRecord(NULL));

  /* Free up some memory */
  pn532_ndef_destroy(rec);
  pn532_ndef_destroy(rec1);
}

/**************************************************************************/
/*!
    Get record tests
*/
/**************************************************************************/
void text_ndef_getRecord(void)
{
  pn532_ndef_record_t rec, rec1, recTemp;

  /* Create a first record */
  TEST_ASSERT_EQUAL_UINT32(
          PN532_ERROR_NONE,
          pn532_ndef_createFromValue(&rec, NDEF_TNF_WELL_KNOWN, (uint8_t*)"T",
                  strlen("T"), (uint8_t*)"id0", strlen("id0"),
                  (uint8_t*)payload0, strlen(payload0)));
  /* Create a second record */
  TEST_ASSERT_EQUAL_UINT32(
          PN532_ERROR_NONE,
          pn532_ndef_createFromValue(&rec1, NDEF_TNF_WELL_KNOWN,
                  (uint8_t*)"U", strlen("U"), NULL, 0, (uint8_t*)payload1,
                  strlen(payload1)));
  /* Link rec1 to rec */
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_NONE,
          pn532_ndef_linkNextRecord(rec, rec1));
  /* Get record 0 from rec */
  recTemp = pn532_ndef_getRecord(rec, 0);
  /* Make sure we have a record */
  TEST_ASSERT_TRUE(recTemp != NULL);
  /* Check contents of record 0 */
  TEST_ASSERT_EQUAL_MEMORY(reference0, pn532_ndef_getAll(recTemp),
          pn532_ndef_getLength(recTemp));
  /* Check record length (should be 45) */
  TEST_ASSERT_EQUAL_UINT32(45, pn532_ndef_getLength(recTemp));
  /* Check message begin bit (should be 1) */
  TEST_ASSERT_EQUAL_UINT8(1, pn532_ndef_getMB(recTemp));
  /* Check message end bit (should be 1) */
  TEST_ASSERT_EQUAL_UINT8(1, pn532_ndef_getME(recTemp));
  /* Check TNF bits (should be 001) */
  TEST_ASSERT_EQUAL_UINT8(NDEF_TNF_WELL_KNOWN, pn532_ndef_getTNF(recTemp));
  /* Check type length (should be 1) */
  TEST_ASSERT_EQUAL_UINT32(1, pn532_ndef_getTypeLength(recTemp));
  /* Check ID length (should be 3) */
  TEST_ASSERT_EQUAL_UINT32(3, pn532_ndef_getIdLength(recTemp));
  /* Check ID , should be 'id0' */
  TEST_ASSERT_EQUAL_MEMORY("id0", pn532_ndef_getId(recTemp),
          pn532_ndef_getIdLength(recTemp));
  /* Check payload length */
  TEST_ASSERT_EQUAL_UINT32(strlen(payload0),
          pn532_ndef_getPayloadLength(recTemp));
  /* Check payload */
  TEST_ASSERT_EQUAL_MEMORY(payload0, pn532_ndef_getPayload(recTemp),
          pn532_ndef_getPayloadLength(recTemp));
  /* Get record 1 from rec */
  recTemp = pn532_ndef_getRecord(rec, 1);
  /* Make sure we have a record */
  TEST_ASSERT_TRUE(recTemp != NULL);
  /* Check RAW data in record 1, should be equal to reference1 */
  TEST_ASSERT_EQUAL_MEMORY(reference1, pn532_ndef_getAll(recTemp),
          pn532_ndef_getLength(recTemp));
  /* Check record length (should be 104) */
  TEST_ASSERT_EQUAL_UINT32(104, pn532_ndef_getLength(recTemp));
  /* Check message begin bit (should be 1) */
  TEST_ASSERT_EQUAL_UINT8(1, pn532_ndef_getMB(recTemp));
  /* Check message end bit (should be 1) */
  TEST_ASSERT_EQUAL_UINT8(1, pn532_ndef_getME(recTemp));
  /* Check TNF bits (should be 001) */
  TEST_ASSERT_EQUAL_UINT8(NDEF_TNF_WELL_KNOWN, pn532_ndef_getTNF(recTemp));
  /* Check type length (should be 1) */
  TEST_ASSERT_EQUAL_UINT32(1, pn532_ndef_getTypeLength(recTemp));
  /* Check ID length (should be 0 ... no ID!) */
  TEST_ASSERT_EQUAL_UINT32(0, pn532_ndef_getIdLength(recTemp));
  /* Check ID ( should be NULL) */
  TEST_ASSERT_EQUAL_STRING(NULL, pn532_ndef_getId(recTemp));
  /* Check payload length */
  TEST_ASSERT_EQUAL_UINT32(strlen(payload1),
          pn532_ndef_getPayloadLength(recTemp));
  /* Check payload */
  TEST_ASSERT_EQUAL_MEMORY(payload1, pn532_ndef_getPayload(recTemp),
          pn532_ndef_getPayloadLength(recTemp));

  /* Intentionally throw some errors! */
  /* Check get third record ... should be NULL! */
  TEST_ASSERT_EQUAL(NULL, pn532_ndef_getRecord(rec, 2));

  /* Free up some memory */
  pn532_ndef_destroy(rec);
  pn532_ndef_destroy(rec1);
}

/**************************************************************************/
/*!
    Append record tests
*/
/**************************************************************************/
void test_ndef_appendRecord(void)
{
  pn532_error_t       errorCode;
  pn532_ndef_record_t rec, rec1, rec2, recTemp;

  /* First create a new text record */
  errorCode = pn532_ndef_createFromValue(&rec, NDEF_TNF_WELL_KNOWN,
          (uint8_t*)"T", strlen("T"), (uint8_t*)"id0", strlen("id0"),
          (uint8_t*)payload0, strlen(payload0));
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_NONE, errorCode);
  TEST_ASSERT_TRUE(rec != NULL);

  /* Next create a new UIR record with no ID */
  errorCode = pn532_ndef_createFromValue(&rec1, NDEF_TNF_WELL_KNOWN,
          (uint8_t*)"U", strlen("U"), NULL, 0, (uint8_t*)payload1,
          strlen(payload1));
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_NONE, errorCode);
  TEST_ASSERT_TRUE(rec1 != NULL);

  /* Link rec1 to rec */
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_NONE,
          pn532_ndef_linkNextRecord(rec, rec1));

  /* Create a third UIR record with an ID */
  TEST_ASSERT_EQUAL_UINT32(
          PN532_ERROR_NONE,
          pn532_ndef_createFromValue(&rec2, NDEF_TNF_WELL_KNOWN,
                  (uint8_t*)"U", strlen("U"), (uint8_t*)"id0",
                  strlen("id0"), (uint8_t*)payload2, strlen(payload2)));
  TEST_ASSERT_TRUE(rec2 != NULL);

  /* Append rec2 onto rec */
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_NONE,
          pn532_ndef_appendRecord(rec, rec2));
  /* Checking number of records, should be 3 */
  TEST_ASSERT_EQUAL_UINT32(3, pn532_ndef_getNumberOfRecords(rec));
  /* Get a copy of the appended record */
  recTemp = pn532_ndef_getRecord(rec, 2);
  /* Make sure we have a record! */
  TEST_ASSERT_TRUE(recTemp != NULL);
  /* Make sure the memory contents matched the ref buffer */
  TEST_ASSERT_EQUAL_MEMORY(reference3, pn532_ndef_getAll(recTemp),
          pn532_ndef_getLength(recTemp));
  /* Check record length (should be 77) */
  TEST_ASSERT_EQUAL_UINT32(77, pn532_ndef_getLength(recTemp));
  /* Check message begin bit (should be 1) */
  TEST_ASSERT_EQUAL_UINT8(1, pn532_ndef_getMB(recTemp));
  /* Check message end bit (should be 1) */
  TEST_ASSERT_EQUAL_UINT8(1, pn532_ndef_getME(recTemp));
  /* Check TNF bits (should be 001) */
  TEST_ASSERT_EQUAL_UINT8(NDEF_TNF_WELL_KNOWN, pn532_ndef_getTNF(recTemp));
  /* Check type length (should be 1) */
  TEST_ASSERT_EQUAL_UINT32(1, pn532_ndef_getTypeLength(recTemp));
  /* Check ID length (should be 3) */
  TEST_ASSERT_EQUAL_UINT32(3, pn532_ndef_getIdLength(recTemp));
  /* Check ID ( should be "id0") */
  TEST_ASSERT_EQUAL_MEMORY("id0", pn532_ndef_getId(recTemp),
          pn532_ndef_getIdLength(recTemp));
  /* Check payload length, should be length of payload2 */
  TEST_ASSERT_EQUAL_UINT32(strlen(payload2),
          pn532_ndef_getPayloadLength(recTemp));
  /* Compare the payload with the original byte array */
  TEST_ASSERT_EQUAL_MEMORY(payload2, pn532_ndef_getPayload(recTemp),
          pn532_ndef_getPayloadLength(recTemp));

  /* Intentionally throw some errors! */
  /* Pass in NULL reference for record count */
  TEST_ASSERT_EQUAL_UINT32(0, pn532_ndef_getNumberOfRecords(NULL));
  /* Pass NULL reference to appendRecord, should be PN532_ERROR_INVALID_PARAM  */
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_INVALID_PARAM,
          pn532_ndef_appendRecord(NULL, rec2));
  /* This is fine ... we should get PN532_ERROR_NONE  */
  TEST_ASSERT_EQUAL_UINT32(PN532_ERROR_NONE,
          pn532_ndef_appendRecord(rec, rec2));

  /* Free up some memory! */
  pn532_ndef_destroy(rec);
  pn532_ndef_destroy(rec1);
  pn532_ndef_destroy(rec2);
}
#endif

/**************************************************************************/
/*!
    Used when creating records from a stream
*/
/**************************************************************************/
pn532_error_t fn_fetch(void* pUserData, uint8_t *out_buff, uint32_t length)
{
  memcpy(out_buff, g_buffer + idx, length);
  idx += length;
  return PN532_ERROR_NONE;
}
