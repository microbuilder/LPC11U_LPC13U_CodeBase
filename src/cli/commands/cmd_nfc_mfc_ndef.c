/**************************************************************************/
/*!
    @file     cmd_nfc_mifareultralight_memdump.c
    @author   K. Townsend (microBuilder.eu)

    @ingroup  CLI

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013 K. Townsend (microBuilder.eu)
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
#include <stdio.h>

#include "projectconfig.h"

#ifdef CFG_PN532

#include "core/gpio/gpio.h"
#include "cli/cli.h"
#include "cli/commands.h"

#include "drivers/rf/nfc/pn532/pn532.h"
#include "drivers/rf/nfc/pn532/pn532_bus.h"
#include "drivers/rf/nfc/pn532/helpers/pn532_config.h"
#include "drivers/rf/nfc/pn532/helpers/pn532_mifare_classic.h"
#include "drivers/rf/nfc/pn532/helpers/pn532_ndef.h"
#include "drivers/rf/nfc/pn532/helpers/pn532_ndef_cards.h"

void message_dump(pn532_ndef_message_t mess);
void print_tag_tech(Tag_t  *pTag);
void print_tag_type(Tag_t *pTag);

pn532_ndef_message_t gNdefMess = NULL;
/**************************************************************************/
/*
 @brief 'cmd_nfc_mfc_ndef_memdump' dump a NFC formated Tags. Input value
 if present is a timeout period of the waiting Tag present time.
 @param argc [in]: argument counter
 @param argv [in]: list of argument value
 @return: void
 */
/**************************************************************************/
void cmd_nfc_mfc_ndef_memdump(uint8_t argc, char **argv)
{
        pn532_error_t error;
        int32_t timeout;
        bool retriesChanged = false;
        uint8_t sak;
        uint16_t atqa;
        Tag_t tag;
        // Set a timeout waiting for passive targets (default = 0xFF, wait forever)
        if (argc > 0)
        {
                getNumber(argv[0], &timeout);
                if (timeout > 0xFF || timeout < 0)
                {
                        printf("\n\rInvalid timeout [0..255]%s", CFG_PRINTF_NEWLINE);
                        return;
                }
                else if (timeout > 0 || timeout < 0xFF)
                {
                        // We can safely ignore errors here since there is a default value anyway
                        pn532_config_SetPassiveActivationRetries(timeout);
                        retriesChanged = true;
                }
        }
        //clean up old ndef message
        // Use the MIFARE Classic Helper to read/write to the tag's EEPROM storage
        printf("Please place a Mifare Classic 1K or 4K card with NDEF data inside %s%s",
                CFG_PRINTF_NEWLINE, CFG_PRINTF_NEWLINE);

        error = pn532_mifareclassic_WaitForTypeATags(&sak, &atqa, &tag.uid[0],
                (size_t*) &tag.lenUid);

        if (!error)
        {
                error = pn532_ndef_mfc_tagType_identify(sak, atqa, &tag);
                if (error != PN532_ERROR_NONE)
                {
                        printf("\n\rIdentify tag error, code = 0x%02X", error);
                }
                print_tag_tech(&tag);

                error = pn532_ndef_mfc_get_tagType(&tag);
                if (error != PN532_ERROR_NONE)
                {
                        printf("\n\rGet tag type error, code = 0x%02X", error);
                        return;
                }

                //print out Ndef tag type
                print_tag_type(&tag);

                if ((tag.type != TAG_TYPE_MFC_1K) && (tag.type != TAG_TYPE_MFC_4K))
                {
                        printf("\n\rOnly support Mifare Classic 1K or 4K");
                        return;
                }

                //destroy previous message then destroy it to avoid memory leak
                if ((gNdefMess)&&(pn532_ndef_getNumberOfRecords(gNdefMess)))
                {
                        pn532_ndef_destroy(gNdefMess);
                        gNdefMess = NULL;
                }

                error = pn532_ndef_mfc_parseNtag(&tag, &gNdefMess);

                if (error != PN532_ERROR_NONE)
                {
                if (error == PN532_ERROR_NOT_NDEF_CARD)
                {
                  printf("\n\rCard has no Ndef Message");
                  printf("\n\rPlease create a Ndef Message by 'npn' command and then write to card!!!");
                }
                else
                {
                        printf("\n\rparsing tag error, code = 0x%02X", error);
                }
                        return;
                }

                message_dump(gNdefMess);

        }
        else
        {
                switch (error)
                {
                        case PN532_ERROR_TIMEOUTWAITINGFORCARD:
                                printf("\n\rTimed out waiting for a card%s", CFG_PRINTF_NEWLINE);
                                break;
                        default:
                                printf("\n\rError establishing passive connection (0x%02x)%s",
                                        error, CFG_PRINTF_NEWLINE);
                                break;
                }
        }

        // Set retry count back to infinite if it was changed
        if (retriesChanged)
        {
                pn532_config_SetPassiveActivationRetries(0xFF);
        }
}

void message_dump(pn532_ndef_message_t mess)
{
        uint8_t idx, numberOfRecord;
        pn532_ndef_record_t pRecord;

        numberOfRecord = pn532_ndef_getNumberOfRecords(mess);

        for (idx=0; idx<numberOfRecord; idx++)
        {
                pRecord = pn532_ndef_getRecord(mess, idx);

                printf("MB: %d\r\n", pn532_ndef_getMB(pRecord));
                printf("ME: %d\r\n", pn532_ndef_getME(pRecord));
                printf("TNF: %d\r\n", pn532_ndef_getTNF(pRecord));
                printf("Type: ");

                pn532PrintHex(pn532_ndef_getType(pRecord), pn532_ndef_getTypeLength(pRecord));
                printf("\n\rId: ");
                pn532PrintHex(pn532_ndef_getId(pRecord), pn532_ndef_getIdLength(pRecord));
                printf("\n\rPayload: ");
                pn532PrintHex(pn532_ndef_getPayload(pRecord), pn532_ndef_getPayloadLength(pRecord));
                printf("\n\rEntire: ");
                pn532PrintHexChar(pn532_ndef_getAll(pRecord), pn532_ndef_getLength(pRecord));
                printf("\r\n");
        }

        return;
}

void print_tag_tech(Tag_t  *pTag)
{
        switch (pTag->type)
        {
                case TAG_TYPE_MFC_1K:
                        printf("\n\r Mifare 1K detected");
                        break;
                case TAG_TYPE_MFC_4K:
                        printf("\n\r Mifare 4K detected");
                        break;
                case TAG_TYPE_MFPLUS_S_2K:
                        printf("\n\r Mifare Plus S 2K detected");
                        break;
                case TAG_TYPE_MFPLUS_S_4K:
                        printf("\n\r Mifare Plus S 4K detected");
                        break;
                case TAG_TYPE_MFPLUS_X_2K:
                        printf("\n\r Mifare Plus x 2K detected");
                        break;
                case TAG_TYPE_MFPLUS_X_4K:
                        printf("\n\r Mifare Plus x 4K detected");
                        break;
                case TAG_TYPE_MF_MINI:
                        printf("\n\r Mifare Mini detected");
                        break;
                case TAG_TYPE_MF_ULTRALIGHT:
                        printf("\n\r Mifare Ultralight detected");
                        break;
                default:
                        printf("\n\r Unknown card detected");
                        break;
        }
        return;
}

void print_tag_type(Tag_t *pTag)
{
        switch (pTag->ndefType)
        {
                case NDEF_TAG_TYPE_BLANK:
                        printf("\n\rType blank card");
                        break;
                case NDEF_TAG_TYPE_NDEF_READ_ONLY:
                        printf("\n\rType read only card");
                        break;
                case NDEF_TAG_TYPE_NDEF_WRITE_ENABLE:
                        printf("\n\rType read/write card");
                        break;
                case NDEF_TAG_UNKNOWN:
                        printf("\n\rUnknown card");
                        break;
                default:
                        break;
        }
        return;
}

/**************************************************************************/
/*
    @brief 'cmd_nfc_mfc_ndef_blankFormat' format a Mifare Classic NFC tag to
        become Blank by using transportation keys and default data . Input value
        if present is a timeout period of the waiting Tag present time.
        @param argc [in]: argument counter
        @param argv [in]: list of argument value
        @return: void
*/
/**************************************************************************/
void cmd_nfc_mfc_ndef_blankFormat(uint8_t argc, char **argv)
{
        pn532_error_t error;
        int32_t timeout;
        bool retriesChanged = false;
        uint8_t sak;
        uint16_t atqa;
        Tag_t tag;
        // Set a timeout waiting for passive targets (default = 0xFF, wait forever)
        if (argc > 0)
        {
                getNumber(argv[0], &timeout);
                if (timeout > 0xFF || timeout < 0)
                {
                        printf("Invalid timeout [0..255]%s", CFG_PRINTF_NEWLINE);
                        return;
                }
                else if (timeout > 0 || timeout < 0xFF)
                {
                        // We can safely ignore errors here since there is a default value anyway
                        pn532_config_SetPassiveActivationRetries(timeout);
                        retriesChanged = true;
                }
        }

        // Use the MIFARE Classic Helper to read/write to the tag's EEPROM storage
        printf("Please place a Nfc formated Mifare Classic 1K or 4K card %s%s",
                CFG_PRINTF_NEWLINE, CFG_PRINTF_NEWLINE);

        // Wait for any ISO14443A card
        error = pn532_mifareclassic_WaitForTypeATags(&sak, &atqa, &tag.uid[0], (size_t*)&tag.lenUid);
        if (!error)
        {
                error = pn532_ndef_mfc_tagType_identify(sak, atqa, &tag);
                if (error != PN532_ERROR_NONE)
                {
                        printf("identify tag error, code = 0x%02X", error);
                        return;
                }
                print_tag_tech(&tag);

                error = pn532_ndef_mfc_get_tagType(&tag);
                if (error != PN532_ERROR_NONE)
                {
                        printf("get tag type error, code = 0x%02X", error);
                        return;
                }
                //print out Ndef tag type
                print_tag_type(&tag);

                if ((tag.type != TAG_TYPE_MFC_1K) && (tag.type != TAG_TYPE_MFC_4K))
                {
                        printf("\n\rOnly support Mifare Classic 1K or 4K");
                        return;
                }

                //after check card, formatting card
                error = pn532_ndef_mfc_blank_format(&tag);
                if (error != PN532_ERROR_NONE)
                {
                        printf("\n\rFormat to blank card error");
                        return;
                }
                else
                {
                        printf("\n\rFormat to blank card success");
                }
        }
        else
        {
                switch (error)
                {
                        case PN532_ERROR_TIMEOUTWAITINGFORCARD:
                                printf("Timed out waiting for a card%s", CFG_PRINTF_NEWLINE);
                                break;
                        default:
                                printf("Error establishing passive connection (0x%02x)%s",
                                        error, CFG_PRINTF_NEWLINE);
                                break;
                }
        }

        // Set retry count back to infinite if it was changed
        if (retriesChanged)
        {
                pn532_config_SetPassiveActivationRetries(0xFF);
        }
}


/**************************************************************************/
/*
    @brief 'cmd_nfc_mfc_ndef_blankFormat' format a Mifare Classic NFC tag to
        become Blank by using transportation keys and default data . Input value
        if present is a timeout period of the waiting Tag present time.
        @param argc [in]: argument counter
        @param argv [in]: list of argument value
        @return: void
*/
/**************************************************************************/
void cmd_nfc_mfc_ndef_nfcFormat(uint8_t argc, char **argv)
{
        pn532_error_t error;
        int32_t timeout;
        bool retriesChanged = false;
        uint8_t sak;
        uint16_t atqa;
        Tag_t tag;
        // Set a timeout waiting for passive targets (default = 0xFF, wait forever)
        if (argc > 0)
        {
                getNumber(argv[0], &timeout);
                if (timeout > 0xFF || timeout < 0)
                {
                        printf("Invalid timeout [0..255]%s", CFG_PRINTF_NEWLINE);
                        return;
                }
                else if (timeout > 0 || timeout < 0xFF)
                {
                        // We can safely ignore errors here since there is a default value anyway
                        pn532_config_SetPassiveActivationRetries(timeout);
                        retriesChanged = true;
                }
        }

        // Use the MIFARE Classic Helper to read/write to the tag's EEPROM storage
        printf("Please place a Nfc formated Mifare Classic 1K or 4K card %s%s",
                CFG_PRINTF_NEWLINE, CFG_PRINTF_NEWLINE);

        // Wait for any ISO14443A card
        error = pn532_mifareclassic_WaitForTypeATags(&sak, &atqa, &tag.uid[0], (size_t*)&tag.lenUid);
        if (!error)
        {
                error = pn532_ndef_mfc_tagType_identify(sak, atqa, &tag);
                if (error != PN532_ERROR_NONE)
                {
                        printf("identify tag error, code = 0x%02X", error);
                        return;
                }
                print_tag_tech(&tag);

                error = pn532_ndef_mfc_get_tagType(&tag);
                if (error != PN532_ERROR_NONE)
                {
                        printf("get tag type error, code = 0x%02X", error);
                        return;
                }

                //print out Ndef tag type
                print_tag_type(&tag);

                if ((tag.type != TAG_TYPE_MFC_1K) && (tag.type != TAG_TYPE_MFC_4K))
                {
                        printf("\n\rOnly support Mifare Classic 1K or 4K");
                        return;
                }

                error = pn532_ndef_mfc_nfc_format(&tag, True);
                if (error != PN532_ERROR_NONE)
                {
                        printf("\n\rFormat to NFC format error");
                        return;
                }
                else
                {
                        printf("\n\rFormat to NFC format success");
                }
        }
        else
        {
                switch (error)
                {
                        case PN532_ERROR_TIMEOUTWAITINGFORCARD:
                                printf("Timed out waiting for a card%s", CFG_PRINTF_NEWLINE);
                                break;
                        default:
                                printf("Error establishing passive connection (0x%02x)%s",
                                        error, CFG_PRINTF_NEWLINE);
                                break;
                }
        }

        // Set retry count back to infinite if it was changed
        if (retriesChanged)
        {
                pn532_config_SetPassiveActivationRetries(0xFF);
        }
}

/**************************************************************************/
/*
 @brief 'cmd_nfc_mfc_ndef_write_ndef' write a Ndef form a card. Input value
 if present is a timeout period of the waiting Tag present time.
 @param argc [in]: argument counter
 @param argv [in]: list of argument value
 @return: void
 */
/**************************************************************************/
void cmd_nfc_mfc_ndef_write_ndef(uint8_t argc, char **argv)
{
        pn532_error_t error;
        bool retriesChanged = false;
        uint8_t sak;
        uint16_t atqa;
        Tag_t tag;
        uint32_t sector = 1; //default

        // Set a timeout waiting for passive targets (default = 0xFF, wait forever)

        // Use the MIFARE Classic Helper to read/write to the tag's EEPROM storage
        printf("Please place a Nfc formated Mifare Classic 1K or 4K card %s%s",
                CFG_PRINTF_NEWLINE, CFG_PRINTF_NEWLINE);

        // Wait for any ISO14443A card
        error = pn532_mifareclassic_WaitForTypeATags(&sak, &atqa, &tag.uid[0], (size_t*)&tag.lenUid);
        if (!error)
        {
                error = pn532_ndef_mfc_tagType_identify(sak, atqa, &tag);
                if (error != PN532_ERROR_NONE)
                {
                        printf("identify tag error, code = 0x%02X", error);
                        return;
                }
                print_tag_tech(&tag);

                error = pn532_ndef_mfc_get_tagType(&tag);
                if (error != PN532_ERROR_NONE)
                {
                        printf("get tag type error, code = 0x%02X", error);
                        return;
                }

                //print out Ndef tag type
                print_tag_type(&tag);

                if ((tag.type != TAG_TYPE_MFC_1K) && (tag.type != TAG_TYPE_MFC_4K))
                {
                        printf("\n\rOnly support Mifare Classic 1K or 4K");
                        return;
                }

                if (gNdefMess)
                {
                        error = pn532_ndef_mfc_ndef_write(&tag, (pn532_ndef_record_t)gNdefMess,
                                (uint8_t) sector);
                        if (error != PN532_ERROR_NONE)
                        {
                                printf("\n\rWrite NDEF to card error");
                                return;
                        }
                        else
                        {
                                printf("\n\rWrite NDEF to card success");
                        }
                }
                else
                {
                        printf("\n\rNdef Message empty!!!! ");
                }
        }
        else
        {
                switch (error)
                {
                        case PN532_ERROR_TIMEOUTWAITINGFORCARD:
                                printf("Timed out waiting for a card%s", CFG_PRINTF_NEWLINE);
                                break;
                        default:
                                printf("Error establishing passive connection (0x%02x)%s",
                                        error, CFG_PRINTF_NEWLINE);
                                break;
                }
        }

        // Set retry count back to infinite if it was changed
        if (retriesChanged)
        {
                pn532_config_SetPassiveActivationRetries(0xFF);
        }
}

/**************************************************************************/
/*
 @brief 'cmd_nfc_mfc_ndef_prepare_ndefMessage' create a Ndef Message to write to card
 @return: void
 */
/**************************************************************************/
void cmd_nfc_mfc_ndef_prepare_ndefMessage(uint8_t argc, char **argv)
{
        uint16_t length;
        uint8_t terminalBuff[4];
        uint8_t ndefPayloadBuff[256];
        uint8_t ndefType;
        uint8_t ndefTnf = NDEF_TNF_WELL_KNOWN;
        uint8_t payloadLength;
        pn532_error_t errCode;

        while (1)
        {
                printf("\r\n1. Bookmark/URI");
                printf("\r\n2. Text");
                printf("\r\n3. SMS");
                printf("\r\nPress a number to select type of Ndef Message:");
                // get type of ndef message
                length = sizeof(terminalBuff);
                cliReadLine(&terminalBuff[0], &length);
                if ((length == 1) && (terminalBuff[0] >= '1') && (terminalBuff[0] <= '3'))
                {
                        ndefType = terminalBuff[0];
                        break;
                }
        }

        switch (ndefType)
        {
                case '1':
                        ndefType = 'U';
                        while (1)
                        {
                                printf("\r\n0. N/A");
                                printf("\r\n1. http://www.");
                                printf("\r\n2. https://www.");
                                printf("\r\n3. http://");
                                printf("\r\n4. https://");
                                printf("\r\nPress a number to select type of Bookmark:");
                                // get the bookmark type
                                length = sizeof(terminalBuff);
                                cliReadLine(&terminalBuff[0], &length);
                                if ((length == 1) && (terminalBuff[0] >= '0')
                                        && (terminalBuff[0] <= '4'))
                                {
                                        break;
                                }
                        }

                        ndefPayloadBuff[0] = terminalBuff[0] - '0';

                        printf("\n\rEnter the content of NdefMessage:\n\r");
                        length = sizeof(ndefPayloadBuff) - 1;
                        cliReadLine(&ndefPayloadBuff[1], &length);
                        payloadLength = 1 + length;

                        break;
                case '2':
                        ndefType = 'T';
                        length = sizeof(ndefPayloadBuff) - 3;
                        ndefPayloadBuff[0] = 0x02; //UTF8
                        ndefPayloadBuff[1] = 'e';
                        ndefPayloadBuff[2] = 'n';
                        printf("\r\nEnter a text:\n\r");
                        cliReadLine(&ndefPayloadBuff[3], &length);
                        payloadLength = length + 3;
                        break;
                case '3':
                        ndefPayloadBuff[0] = 0x00; //NA
                        length = sizeof(ndefPayloadBuff) - 1;
                        printf("\r\nEnter a SMS:\n\r");
                        cliReadLine(&ndefPayloadBuff[1], &length);
                        payloadLength = 1 + length;
                        break;
                default:
                        break;
        }

        //destroy previous message then destroy it to avoid memory leak
        if ((gNdefMess)&&(pn532_ndef_getNumberOfRecords(gNdefMess)))
        {
                pn532_ndef_destroy(gNdefMess);
                gNdefMess = NULL;
        }

        errCode = pn532_ndef_createFromValue(&gNdefMess, ndefTnf, &ndefType, 1,
                                                                                NULL, 0, ndefPayloadBuff, payloadLength);
        if (errCode != PN532_ERROR_NONE)
        {
                printf("Create Ndef Record error, code = 0x%02X", errCode);

        }

        message_dump(gNdefMess);

        return;

}

#endif  // #ifdef CFG_PN532
