/**************************************************************************/
/*!
    @file pn532_mifare_classic.c

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

/*  MIFARE CLASSIC DESCRIPTION
    ==========================

    MIFARE Classic cards come in 1K and 4K varieties.  While several
    varieties of chips exist, the two main chipsets used are described
    in the following publicly accessible documents:

        MF1S503x Mifare Classic 1K data sheet:
        http://www.nxp.com/documents/data_sheet/MF1S503x.pdf

        MF1S70yyX MIFARE Classic 4K data sheet:
        http://www.nxp.com/documents/data_sheet/MF1S70YYX.pdf

    Mifare Classic cards typically have a a 4-byte NUID, though you may
    find cards with 7 byte IDs as well

    EEPROM MEMORY
    =============
    Mifare Classic cards have either 1K or 4K of EEPROM memory. Each
    memory block can be configured with different access conditions,
    with two seperate authentication keys present in each block.

    The two main Mifare Classic card types are organised as follows:

        1K Cards: 16 sectors of 4 blocks (0..15)
        4K Cards: 32 sectors of 4 blocks (0..31) and
                  8 sectors of 16 blocks (32..39)

    4 block sectors
    ===============
    Sector  Block   Bytes                                                           Description
    ------  -----   -----                                                           -----------
                    0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15

    1       3       [-------KEY A-------]   [Access Bits]   [-------KEY B-------]   Sector Trailer 1
            2       [                            Data                           ]   Data
            1       [                            Data                           ]   Data
            0       [                            Data                           ]   Data

    0       3       [-------KEY A-------]   [Access Bits]   [-------KEY B-------]   Sector Trailer 1
            2       [                            Data                           ]   Data
            1       [                            Data                           ]   Data
            0       [                     Manufacturer Data                     ]   Manufacturer Block

    Sector Trailer (Block 3)
    ------------------------
    The sector trailer block contains the two secret keys (Key A and Key B), as well
    as the access conditions for the four blocks.  It has the following structure:

        Sector Trailer Bytes
        --------------------------------------------------------------
        0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15
        [       Key A       ]   [Access Bits]   [       Key B       ]

    For more information in using Keys to access the clock contents, see
    Accessing Data Blocks further below.

    Data Blocks (Blocks 0..2)
    -------------------------
    Data blocks are 16 bytes wide and, depending on the permissions set in the
    access bits, can be read from and written to. You are free to use the 16 data
    bytes in any way you wish.  You can easily store text input, store four 32-bit
    integer values, a 16 character uri, etc.

    Data Blocks as "Value Blocks"
    -----------------------------
    An alternative to storing random data in the 16 byte-wide blocks is to
    configure them as "Value Blocks".  Value blocks allow performing electronic
    purse functions (valid commands are: read, write, increment, decrement,
    restore, transfer).

    Each Value block contains a single signed 32-bit value, and this value is
    stored 3 times for data integrity and security reasons.  It is stored twice
    non-inverted, and once inverted.  The last 4 bytes are used for a 1-byte
    address, which is stored 4 times (twice non-inverted, and twice inverted).

    Data blocks configured as "Value Blocks" have the following structure:

        Value Block Bytes
        --------------------------------------------------------------
        0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15
        [   Value   ]   [   ~Value  ]   [   Value   ]   [A  ~A  A   ~A]

    For more information on value blocks see 8.6.2 and chapter 10 in:
    http://www.nxp.com/documents/data_sheet/MF1S503x.pdf

    Manufacturer Block (Sector 0, Block 0)
    --------------------------------------
    Sector 0 is special since it contains the Manufacturer Block. This block
    contains the manufacturer data, and is read-only.  It should be avoided
    unless you know what you are doing.

    16 block sectors
    ================
    16 block sectors are identical to 4 block sectors, but with more data blocks.  The same
    structure described in the 4 block sectors above applies.

    Sector  Block   Bytes                                                           Description
    ------  -----   -----                                                           ----------
                    0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15

    32      15      [-------KEY A-------]   [Access Bits]   [-------KEY B-------]   Sector Trailer 32
            14      [                            Data                           ]   Data
            13      [                            Data                           ]   Data
            ...
            2       [                            Data                           ]   Data
            1       [                            Data                           ]   Data
            0       [                            Data                           ]   Data

    ACCESSING DATA BLOCKS
    =====================

    In order to access the Mifare Classic card, you must follow these steps:

    1.) You must retrieve the 7 byte UID or the 4-byte NUID of the card.
        This can be done using pn532_mifareclassic_WaitForPassiveTarget()
        below, which will return the appropriate ID.

    2.) You must authenticate the sector you wish to access according to the
        access rules defined in the Sector Trailer block for that sector.
        This can be done using pn532_mifareclassic_AuthenticateBlock(),
        passing in the appropriate key value.

        Most new cards have a default Key A of 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF,
        but some common values worth trying are:

            0XFF 0XFF 0XFF 0XFF 0XFF 0XFF
            0XD3 0XF7 0XD3 0XF7 0XD3 0XF7
            0XA0 0XA1 0XA2 0XA3 0XA4 0XA5
            0XB0 0XB1 0XB2 0XB3 0XB4 0XB5
            0X4D 0X3A 0X99 0XC3 0X51 0XDD
            0X1A 0X98 0X2C 0X7E 0X45 0X9A
            0XAA 0XBB 0XCC 0XDD 0XEE 0XFF
            0X00 0X00 0X00 0X00 0X00 0X00
            0XAB 0XCD 0XEF 0X12 0X34 0X56

    3.) Once authentication has succeeded, and depending on the sector
        permissions, you can then read/write/increment/decrement the
        contents of the specific block, using one of the helper functions
        included in this module.

*/

#include "projectconfig.h"

#ifdef CFG_PN532

#include <string.h>

#include "../pn532.h"
#include "../pn532_bus.h"
#include "pn532_mifare.h"
#include "pn532_mifare_classic.h"

#include "core/delay/delay.h"

/**************************************************************************/
/*!
    Resets the magnetic field and waits for a new Type A tag
*/
/**************************************************************************/
static pn532_error_t pn532_mifareclassic_reset()
{
  uint8_t sak;
  uint16_t atqa;
  uint8_t uid[8];
  size_t leng;
  pn532_error_t error;

  pn532_mifareclassic_RFfield(FALSE);
  delay(50);
  pn532_mifareclassic_RFfield(TRUE);
  error = pn532_mifareclassic_WaitForTypeATags(&sak, &atqa, &uid[0], &leng);

  return error;
}

/**************************************************************************/
/*!
    Indicates whether the specified block number is the first block
    in the sector (block 0 relative to the current sector)
*/
/**************************************************************************/
bool pn532_mifareclassic_isFirstBlock (uint32_t uiBlock)
{
  // Test if we are in the small or big sectors
  if (uiBlock < 128)
    return ((uiBlock) % 4 == 0);
  else
    return ((uiBlock) % 16 == 0);
}

/**************************************************************************/
/*!
    Indicates whether the specified block number is the sector trailer
*/
/**************************************************************************/
bool pn532_mifareclassic_isTrailerBlock (uint32_t uiBlock)
{
  // Test if we are in the small or big sectors
  if (uiBlock < 128)
    return ((uiBlock + 1) % 4 == 0);
  else
    return ((uiBlock + 1) % 16 == 0);
}

/**************************************************************************/
/*!
    Tries to detect MIFARE targets in passive mode.  This needs to be done
    before anything useful can be accomplished with a tag since you need
    the tag's unique UID to communicate with it.

    @param  pbtCUID     Pointer to the byte array where the card's UID
                        will be stored once a card is detected
    @param  pszCUIDLen  Pointer to the size of the card UID in bytes

    Response for a valid ISO14443A 106KBPS (Mifare Classic, etc.)
    should be in the following format.  See UM0701-02 section
    7.3.5 for more information

    byte            Description
    -------------   ------------------------------------------
    b0..6           Frame header and preamble
    b7              Tags Found
    b8              Tag Number (only one used in this example)
    b9..10          SENS_RES
    b11             SEL_RES
    b12             NFCID Length
    b13..NFCIDLen   NFCID

    SENS_RES   SEL_RES     Manufacturer/Card Type    NFCID Len
    --------   -------     -----------------------   ---------
    00 04      08          NXP Mifare Classic 1K     4 bytes
    00 02      18          NXP Mifare Classic 4K     4 bytes

    @note   Possible error messages are:

            - PN532_ERROR_WRONGCARDTYPE
*/
/**************************************************************************/
pn532_error_t pn532_mifareclassic_WaitForPassiveTarget (byte_t * pbtCUID, size_t * pszCUIDLen)
{
  byte_t abtResponse[PN532_RESPONSELEN_INLISTPASSIVETARGET];
  pn532_error_t error;
  size_t szLen;
    uint8_t i;

  #ifdef PN532_DEBUGMODE
    PN532_DEBUG("Waiting for an ISO14443A Card%s", CFG_PRINTF_NEWLINE);
  #endif

  /* Try to initialise a single ISO14443A tag at 106KBPS                  */
  /* Note:  To wait for a card with a known UID, append the four byte     */
  /*        UID to the end of the command.                                */
  byte_t abtCommand[] = { PN532_COMMAND_INLISTPASSIVETARGET, 0x01, PN532_MODULATION_ISO14443A_106KBPS};
  error = pn532Write(abtCommand, sizeof(abtCommand));
  if (error)
    return error;

  /* Wait until we get a valid response or a timeout                      */
  do
  {
    delay(25);
    error = pn532Read(abtResponse, &szLen);
  } while (error == PN532_ERROR_RESPONSEBUFFEREMPTY);
  if (error)
    return error;

  /* Check szLen ... if it's 10 we've probably timed out via MaxRetries */
  /* ToDo: Properly parse and handle this error case! */
  if (szLen == 10)
  {
    return PN532_ERROR_TIMEOUTWAITINGFORCARD;
  }

  /* Check SENSE_RES to make sure this is a Mifare Classic card           */
  /*          Classic 1K       = 00 04                                    */
  /*          Classic 4K       = 00 02                                    */
  /*          Classic Emulated = 00 08                                    */
  if ((abtResponse[10] == 0x02) ||
      (abtResponse[10] == 0x04) ||
      (abtResponse[10] == 0x08))
  {
    /* Card appears to be Mifare Classic */
    *pszCUIDLen = abtResponse[12];
    for (i=0; i < *pszCUIDLen; i++)
    {
      pbtCUID[i] = abtResponse[13+i];
    }
    #ifdef PN532_DEBUGMODE
      PN532_DEBUG("Card Found: %s", CFG_PRINTF_NEWLINE);
      PN532_DEBUG("      ATQA: ");
      pn532PrintHex(abtResponse+9, 2);
      PN532_DEBUG("      SAK: %02x%s", abtResponse[11], CFG_PRINTF_NEWLINE);
      PN532_DEBUG("      UID: ");
      pn532PrintHex(pbtCUID, *pszCUIDLen);
    #endif
  }
  else
  {
    /* Card is ISO14443A but doesn't appear to be Mifare Classic          */
    /*    Mifare Ultralight    = 0x0044                                   */
    /*    Mifare DESFire       = 0x0344                                   */
    /*    Innovision Jewel     = 0x0C00                                   */
    #ifdef PN532_DEBUGMODE
      PN532_DEBUG("Wrong Card Type (Expected ATQA 00 02, 00 04 or 00 08) %s%s", CFG_PRINTF_NEWLINE, CFG_PRINTF_NEWLINE);
      PN532_DEBUG("  ATQA       : ");
      pn532PrintHex(abtResponse+9, 2);
      PN532_DEBUG("  SAK        : %02x%s", abtResponse[11], CFG_PRINTF_NEWLINE);
      PN532_DEBUG("  UID Length : %d%s", abtResponse[12], CFG_PRINTF_NEWLINE);
      PN532_DEBUG("  UID        : ");
      size_t pos;
      for (pos=0; pos < abtResponse[12]; pos++)
      {
        printf("%02x ", abtResponse[13 + pos]);
      }
      printf("%s%s", CFG_PRINTF_NEWLINE, CFG_PRINTF_NEWLINE);
    #endif
    return PN532_ERROR_WRONGCARDTYPE;
  }

  return PN532_ERROR_NONE;
}

/**************************************************************************/
/*!
    Tries to detect MIFARE targets in passive mode.

    @param  pSak [out]  Pointer SAK byte
    @param  Atqa        Pointer to ATQA data

    Response for a valid ISO14443A 106KBPS (Mifare Classic, etc.)
    should be in the following format.  See UM0701-02 section
    7.3.5 for more information

    byte            Description
    -------------   ------------------------------------------
    b0..6           Frame header and preamble
    b7              Tags Found
    b8              Tag Number (only one used in this example)
    b9..10          SENS_RES
    b11             SEL_RES
    b12             NFCID Length
    b13..NFCIDLen   NFCID

    SENS_RES   SEL_RES     Manufacturer/Card Type    NFCID Len
    --------   -------     -----------------------   ---------
    00 04      08          NXP Mifare Classic 1K     4 bytes
    00 02      18          NXP Mifare Classic 4K     4 bytes

    @note   Possible error messages are:

            - PN532_ERROR_WRONGCARDTYPE
            - PN532_ERROR_TIMEOUTWAITINGFORCARD
*/
/**************************************************************************/
pn532_error_t pn532_mifareclassic_WaitForTypeATags (byte_t * pSak, uint16_t * pAtqa, byte_t * pbtCUID, size_t * szCUIDLen)
{
  byte_t abtResponse[PN532_RESPONSELEN_INLISTPASSIVETARGET];
  pn532_error_t error;
  size_t szLen;
  uint8_t i;

  #ifdef PN532_DEBUGMODE
    PN532_DEBUG("Waiting for an ISO14443A Card%s", CFG_PRINTF_NEWLINE);
  #endif

  /* Try to initialise a single ISO14443A tag at 106KBPS                  */
  /* Note:  To wait for a card with a known UID, append the four byte     */
  /*        UID to the end of the command.                                */
  byte_t abtCommand[] = { PN532_COMMAND_INLISTPASSIVETARGET, 0x01, PN532_MODULATION_ISO14443A_106KBPS};
  error = pn532Write(abtCommand, sizeof(abtCommand));
  if (error)
    return error;

  /* Wait until we get a valid response or a timeout                      */
  do
  {
    delay(25);
    error = pn532Read(abtResponse, &szLen);
  } while (error == PN532_ERROR_RESPONSEBUFFEREMPTY);
  if (error)
    return error;

  /* Check szLen ... if it's 10 we've probably timed out via MaxRetries */
  /* ToDo: Properly parse and handle this error case! */
  if (szLen == 10)
  {
    return PN532_ERROR_TIMEOUTWAITINGFORCARD;
  }

  *pAtqa = ((uint16_t)abtResponse[9]<<8) | (uint16_t)abtResponse[10];
  *pSak = abtResponse[11];
  *szCUIDLen = abtResponse[12];
  for (i=0; i < *szCUIDLen; i++)
  {
    pbtCUID[i] = abtResponse[13+i];
  }
#ifdef PN532_DEBUGMODE
  printf("SAK: %02x ",  *pSak);
  printf("ATQA: %04x ", *pAtqa);
#endif

  return PN532_ERROR_NONE;
}

/**************************************************************************/
/*!
    Tries to authenticate a block of memory on a MIFARE card using the
    INDATAEXCHANGE command.  See section 7.3.8 of the PN532 User Manual
    for more information on sending MIFARE and other commands.

    @param  pbtCUID       Pointer to a byte array containing the card UID
    @param  szCUIDLen     The length (in bytes) of the card's UID (Should
                          be 4 for MIFARE Classic)
    @param  uiBlockNumber The block number to authenticate.  (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  uiKeyType     Which key type to use during authentication
                          (PN532_MIFARE_CMD_AUTH_A or PN532_MIFARE_CMD_AUTH_B)
    @param  pbtKeys       Pointer to a byte array containing the 6 byte
                          key value
*/
/**************************************************************************/
pn532_error_t pn532_mifareclassic_AuthenticateBlock (byte_t * pbtCUID, size_t szCUIDLen, uint32_t uiBlockNumber, uint8_t uiKeyType, byte_t * pbtKeys)
{
  pn532_error_t error;
  byte_t abtCommand[17];
  byte_t abtResponse[PN532_RESPONSELEN_INDATAEXCHANGE];
  size_t szLen;
  uint8_t i;

  #ifdef PN532_DEBUGMODE
  PN532_DEBUG("Trying to authenticate card ");
  pn532PrintHex(pbtCUID, szCUIDLen);
  #endif

  /* Prepare the authentication command */
  abtCommand[0] = PN532_COMMAND_INDATAEXCHANGE;   /* Data Exchange Header */
  abtCommand[1] = 1;                              /* Max card numbers */
  abtCommand[2] = (uiKeyType == PN532_MIFARE_CMD_AUTH_B) ? PN532_MIFARE_CMD_AUTH_B : PN532_MIFARE_CMD_AUTH_A;
  abtCommand[3] = uiBlockNumber;                  /* Block Number (1K = 0..63, 4K = 0..255 */
  memcpy (abtCommand+4, pbtKeys, 6);
  for (i = 0; i < szCUIDLen; i++)
  {
    abtCommand[10+i] = pbtCUID[i];                /* 4 byte card ID */
  }

  /* Send the command */
  error = pn532Write(abtCommand, 10+szCUIDLen);
  if (error)
  {
    /* Problem with the serial bus, etc. */
    #ifdef PN532_DEBUGMODE
      PN532_DEBUG("Authentification failed%s", CFG_PRINTF_NEWLINE);
    #endif
    pn532_mifareclassic_reset();
    return error;
  }

  /* Read the authentication response */
  memset(abtResponse, 0, PN532_RESPONSELEN_INDATAEXCHANGE);
  do
  {
    delay(25);
    error = pn532Read(abtResponse, &szLen);
  }
  while (error == PN532_ERROR_RESPONSEBUFFEREMPTY);
  if (error)
  {
    #ifdef PN532_DEBUGMODE
      PN532_DEBUG("Authentification failed%s", CFG_PRINTF_NEWLINE);
    #endif
    pn532_mifareclassic_reset();

    return error;
  }

  /* For authentication success, bytes 5-7 should be: 0xD5 0x41 0x00 */
  /* Mifare auth error is technically byte 7: 0x14 but anything other and 0x00 */
  /* is not good */
  {
    #ifdef PN532_DEBUGMODE
      PN532_DEBUG("Authentification failed%s", CFG_PRINTF_NEWLINE);
    #endif
    pn532_mifareclassic_reset();
    return PN532_ERROR_AUTHENTICATE_FAIL;
  }

  /* Output the authentication data */
  #ifdef PN532_DEBUGMODE
    PN532_DEBUG("Authenticated block %d %s", uiBlockNumber, CFG_PRINTF_NEWLINE);
  #endif

  // Return OK signal
  return PN532_ERROR_NONE;
}

/**************************************************************************/
/*!
    Tries to read an entire 16-byte data block at the specified block
    address.

    @param  uiBlockNumber The block number to authenticate.  (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  pbtData       Pointer to the byte array that will hold the
                          retrieved data (if any)

    @note   Possible error messages are:

            - PN532_ERROR_BLOCKREADFAILED
*/
/**************************************************************************/
pn532_error_t pn532_mifareclassic_ReadDataBlock (uint8_t uiBlockNumber, byte_t * pbtData)
{
  pn532_error_t error;
  byte_t abtCommand[4];
  byte_t abtResponse[PN532_RESPONSELEN_INDATAEXCHANGE];
  size_t szLen;

  #ifdef PN532_DEBUGMODE
    PN532_DEBUG("Reading 16 bytes at block %03d%s", uiBlockNumber, CFG_PRINTF_NEWLINE);
  #endif

  /* Prepare the command */
  abtCommand[0] = PN532_COMMAND_INDATAEXCHANGE;
  abtCommand[1] = 1;                            /* Card number */
  abtCommand[2] = PN532_MIFARE_CMD_READ;        /* Mifare Read command = 0x30 */
  abtCommand[3] = uiBlockNumber;                /* Block Number (0..63 for 1K, 0..255 for 4K) */

  /* Send the commands */
  error = pn532Write(abtCommand, sizeof(abtCommand));
  if (error)
  {
    /* Bus error, etc. */
    #ifdef PN532_DEBUGMODE
      PN532_DEBUG("Read failed%s", CFG_PRINTF_NEWLINE);
    #endif
    return error;
  }

  /* Read the response */
  memset(abtResponse, 0, PN532_RESPONSELEN_INDATAEXCHANGE);
  do
  {
    delay(50);
    error = pn532Read(abtResponse, &szLen);
  }
  while (error == PN532_ERROR_RESPONSEBUFFEREMPTY);
  if (error)
  {
    #ifdef PN532_DEBUGMODE
      PN532_DEBUG("Read failed%s", CFG_PRINTF_NEWLINE);
    #endif
    return error;
  }

  /* Make sure we have a valid response (should be 26 bytes long) */
  if (szLen == 26)
  {
    /* Copy the 16 data bytes to the output buffer        */
    /* Block content starts at byte 9 of a valid response */
    memcpy (pbtData, abtResponse+8, 16);
  }
  else
  {
    #ifdef PN532_DEBUGMODE
      PN532_DEBUG("Unexpected response reading block %d.  Bad key?%s", uiBlockNumber, CFG_PRINTF_NEWLINE);
    #endif
    return PN532_ERROR_BLOCKREADFAILED;
  }

  /* Display data for debug if requested */
  #ifdef PN532_DEBUGMODE
    PN532_DEBUG("Block %03d: %s", uiBlockNumber, CFG_PRINTF_NEWLINE);
    pn532PrintHexChar(pbtData, 16);
  #endif

  // Return OK signal
  return PN532_ERROR_NONE;
}

/**************************************************************************/
/*!
    Tries to write an entire 16-byte data block at the specified block
    address.

    @param  uiBlockNumber The block number to write to (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  pbtData       Pointer to the byte array to write

    @note   Possible error messages are:

            - PN532_ERROR_BLOCKWRITEFAILED
*/
/**************************************************************************/
pn532_error_t pn532_mifareclassic_WriteDataBlock (uint8_t uiBlockNumber, byte_t * pbtData)
{
  pn532_error_t error;
  byte_t abtCommand[20]; /* 16 byte payload + 4 command bytes */
  byte_t abtResponse[PN532_RESPONSELEN_INDATAEXCHANGE];
  size_t szResponseLen;

  #ifdef PN532_DEBUGMODE
    PN532_DEBUG("Trying to write 16 bytes at block %02d%s", uiBlockNumber, CFG_PRINTF_NEWLINE);
  #endif

  /* Prepare the command */
  abtCommand[0] = PN532_COMMAND_INDATAEXCHANGE;
  abtCommand[1] = 1;                            /* Card number */
  abtCommand[2] = PN532_MIFARE_CMD_WRITE;       /* Mifare Write command = 0xA0 */
  abtCommand[3] = uiBlockNumber;                /* Block Number (0..63 for 1K, 0..255 for 4K) */
  memcpy (abtCommand+4, pbtData, 16);           /* Data Payload */

  /* Send the commands */
  error = pn532Write(abtCommand, sizeof(abtCommand));
  if (error)
  {
    /* Bus error, etc. */
    #ifdef PN532_DEBUGMODE
      PN532_DEBUG("Write failed%s", CFG_PRINTF_NEWLINE);
    #endif
    return error;
  }

  /* Read the response */
  memset(abtResponse, 0, PN532_RESPONSELEN_INDATAEXCHANGE);
  do
  {
    delay(50);
    error = pn532Read(abtResponse, &szResponseLen);
  }
  while (error == PN532_ERROR_RESPONSEBUFFEREMPTY);
  if (error)
  {
    #ifdef PN532_DEBUGMODE
      PN532_DEBUG("Write failed%s", CFG_PRINTF_NEWLINE);
    #endif
    return error;
  }

  /* Make sure we have a valid response (should be 26 bytes long) */
  if (szResponseLen == 26)
  {
    /* Copy the 16 data bytes to the output buffer        */
    /* Block content starts at byte 9 of a valid response */
    memcpy (pbtData, abtResponse+9, 16);
  }
  else
  {
    #ifdef PN532_DEBUGMODE
      PN532_DEBUG("Unexpected response writing block %d.  Bad key?%s", uiBlockNumber, CFG_PRINTF_NEWLINE);
    #endif
    return PN532_ERROR_BLOCKWRITEFAILED;
  }

  /* Display data for debug if requested */
  #ifdef PN532_DEBUGMODE
    PN532_DEBUG("Block %03d: ", uiBlockNumber);
    pn532PrintHexChar(pbtData, 16);
  #endif

  // Return OK signal
  return PN532_ERROR_NONE;
}

/**************************************************************************/
/*!
    Tries to create a 'value-block' at the specified block.  Value blocks
    contain a single 32-bit signed integer, and can be incremented,
    decremented, and manipulated using a set of pre-defined Mifare
    commands.

    For more information on 'Value Blocks' see section 8.6.2 in:
    http://www.nxp.com/documents/data_sheet/MF1S503x.pdf

    @param  uiBlockNumber The block number to write to (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  value         The 32-bit integer value to store

    @note   Possible error messages are:

            - PN532_ERROR_BLOCKWRITEFAILED
*/
/**************************************************************************/
pn532_error_t pn532_mifareclassic_CreateValueBlock (uint8_t uiBlockNumber, int32_t value)
{
  pn532_error_t error;
  byte_t buffer[16];
  int32_t inverted;

  /* Display data for debug if requested */
  #ifdef PN532_DEBUGMODE
    PN532_DEBUG("Writing value block at %d: %d %s", uiBlockNumber, value, CFG_PRINTF_NEWLINE);
  #endif

  /* First four bytes are the regular value in lsb format */
  buffer[0] = (value >> 24) & 0xFF;
  buffer[1] = (value >> 16) & 0xFF;
  buffer[2] = (value >> 8) & 0xFF;
  buffer[3] = value & 0xFF;

  /* Next four bytes are the value inverted */
  inverted = ~value;
  buffer[4] = (inverted >> 24) & 0xFF;
  buffer[5] = (inverted >> 16) & 0xFF;
  buffer[6] = (inverted >> 8) & 0xFF;
  buffer[7] = inverted & 0xFF;

  /* Next four bytes are the regular value in lsb format again */
  buffer[8] = (value >> 24) & 0xFF;
  buffer[9] = (value >> 16) & 0xFF;
  buffer[10] = (value >> 8) & 0xFF;
  buffer[11] = value & 0xFF;

  /* Last four bytes are the addr byte (use block number for now) */
  buffer[12] = uiBlockNumber;    /* Normal value */
  buffer[13] = ~uiBlockNumber;   /* Inverted */
  buffer[14] = uiBlockNumber;    /* Normal value */
  buffer[15] = ~uiBlockNumber;   /* Inverted */

  error = pn532_mifareclassic_WriteDataBlock(uiBlockNumber, buffer);
  if (error)
    return error;

  // Return OK signal
  return PN532_ERROR_NONE;
}

/**************************************************************************/
/*!
    Returns a 32-bit integer value from a Mifare Value Block

    @param  uiBlockNumber The block number to write to (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  value         Placeholder for the 32-bit integer
*/
/**************************************************************************/
pn532_error_t pn532_mifareclassic_ReadValueBlock (uint8_t uiBlockNumber, int32_t * value)
{
  pn532_error_t error;
  /* ToDo: This should only require 16 bytes ... but test for overflow to be sure */
  /* Was 'PN532_RESPONSELEN_INDATAEXCHANGE' for len */
  byte_t abtBlockData[16];

  // Make sure the block address is valid, and we're not in the sector trailer
  if (uiBlockNumber > 255 || uiBlockNumber < 1 || pn532_mifareclassic_isTrailerBlock(uiBlockNumber))
  {
    #ifdef PN532_DEBUGMODE
      PN532_DEBUG("Invalid block number%s", CFG_PRINTF_NEWLINE);
    #endif
    return PN532_ERROR_ADDRESSOUTOFRANGE;
  }

  /* Try to read the appropriate data block */
  error = pn532_mifareclassic_ReadDataBlock (uiBlockNumber, abtBlockData);
  if (!error)
  {
    int32_t vpos, vneg;
    /* Positive value */
    vpos =  abtBlockData[0] << 24;
    vpos |= abtBlockData[1] << 16;
    vpos |= abtBlockData[2] << 8;
    vpos |= abtBlockData[3] & 0xFF;
    /* Inverted value */
    vneg =  abtBlockData[4] << 24;
    vneg |= abtBlockData[5] << 16;
    vneg |= abtBlockData[6] << 8;
    vneg |= abtBlockData[7] & 0xFF;
    if (~vpos != vneg)
    {
      /* This isn't a properly formatted Mifare Value Block */
      return PN532_ERROR_INCORRECTBLOCKFORMAT;
    }
    else
    {
      *value = vpos;
      return PN532_ERROR_NONE;
    }
  }
  else
  {
    return error;
  }
}

/**************************************************************************/
/*!
    Increments a Value Block by the specified number, and then sends the
    transfer command to persist the changes.

    @param  uiBlockNumber The block number to write to (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  value         The 32-bit integer value to store
*/
/**************************************************************************/
pn532_error_t pn532_mifareclassic_IncrementValueBlock (uint8_t uiBlockNumber, int32_t value)
{
  pn532_error_t error;
  int32_t oldValue, newValue;

  // It's less code just to recreate the block rather than using
  // the Mifare increment command

  // First read the value
  error = pn532_mifareclassic_ReadValueBlock(uiBlockNumber, &oldValue);
  if (error)
    return error;

  // Increment the old value
  newValue = oldValue + value;

  // Write it back
  error = pn532_mifareclassic_CreateValueBlock(uiBlockNumber, newValue);
  if (error)
    return error;

  // Return OK signal
  return PN532_ERROR_NONE;
}

/**************************************************************************/
/*!
    Decrements a Value Block by the specified number, and then sends the
    transfer command to persist the changes.

    @param  uiBlockNumber The block number to write to (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  value         The 32-bit integer value to store
*/
/**************************************************************************/
pn532_error_t pn532_mifareclassic_DecrementValueBlock (uint8_t uiBlockNumber, int32_t value)
{
  pn532_error_t error;
  int32_t oldValue, newValue;

  // It's less code just to recreate the block rather than using
  // the Mifare decrement command

  // First read the value
  error = pn532_mifareclassic_ReadValueBlock(uiBlockNumber, &oldValue);
  if (error)
    return error;

  // Increment the old value
  newValue = oldValue - value;

  // Write it back
  error = pn532_mifareclassic_CreateValueBlock(uiBlockNumber, newValue);
  if (error)
    return error;

  // Return OK signal
  return PN532_ERROR_NONE;
}

pn532_error_t pn532_mifareclassic_RFfield(BOOL fieldOn)
{
  pn532_error_t error;

  byte_t RFcmdOff[] = {0xD4, 0x32, 0x01, 0x00};
  byte_t RFcmdOn[]  = {0xD4, 0x32, 0x01, 0x03};

  if(fieldOn == FALSE)
  {
    error = pn532Write(RFcmdOff, sizeof(RFcmdOff));
  }
  else
  {
    error = pn532Write(RFcmdOn, sizeof(RFcmdOn));
  }

  return error;
}

#endif  // #ifdef CFG_PN532
