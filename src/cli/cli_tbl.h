/**************************************************************************/
/*!
    @file     cli_tbl.h
    @author   K. Townsend (microBuilder.eu)

    @brief    Command lookup table for the CLI
    @ingroup  CLI

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, K. Townsend
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

#ifndef __CLI_TBL_H__
#define __CLI_TBL_H__

#ifdef __cplusplus
extern "C" {
#endif

#define CMD_COUNT (sizeof(cli_tbl)/sizeof(cli_t))

#include <stdio.h>

#ifdef CFG_INTERFACE_UART
#include "core/uart/uart.h"
#endif

/* Function prototypes for the command table */
void cmd_help(uint8_t argc, char **argv);         /* handled by cli/cli.c */
void cmd_sysinfo(uint8_t argc, char **argv);
void cmd_dbg_memrd(uint8_t argc, char **argv);
void cmd_eeprom_read(uint8_t argc, char **argv);
void cmd_eeprom_write(uint8_t argc, char **argv);

#ifdef CFG_ENABLE_I2C
void cmd_i2c_scan(uint8_t argc, char **argv);
void cmd_i2c_write(uint8_t argc, char **argv);
void cmd_i2c_read(uint8_t argc, char **argv);
#endif

#ifdef CFG_PN532
void cmd_nfc_mifareclassic_memdump(uint8_t argc, char **argv);
void cmd_nfc_mifareultralight_memdump(uint8_t argc, char **argv);
void cmd_nfc_mifareclassic_valueblock_create(uint8_t argc, char **argv);
void cmd_nfc_mifareclassic_valueblock_increment(uint8_t argc, char **argv);
void cmd_nfc_mifareclassic_valueblock_decrement(uint8_t argc, char **argv);
void cmd_nfc_mifareclassic_valueblock_read(uint8_t argc, char **argv);
void cmd_nfc_mfc_ndef_memdump(uint8_t argc, char **argv);
void cmd_nfc_mfc_ndef_blankFormat(uint8_t argc, char **argv);
void cmd_nfc_mfc_ndef_nfcFormat(uint8_t argc, char **argv);
void cmd_nfc_mfc_ndef_write_ndef(uint8_t argc, char **argv);
void cmd_nfc_mfc_ndef_prepare_ndefMessage(uint8_t argc, char **argv);
#endif

#ifdef CFG_SDCARD
void cmd_sd_dir(uint8_t argc, char **argv);
#endif

#ifdef CFG_CHIBI
void cmd_chibi_addr(uint8_t argc, char **argv);
void cmd_chibi_ieeeaddr(uint8_t argc, char **argv);
void cmd_chibi_tx(uint8_t argc, char **argv);
#endif

#ifdef CFG_RTC
void cmd_rtc_read(uint8_t argc, char **argv);
void cmd_rtc_write(uint8_t argc, char **argv);
#endif

#ifdef CFG_CC3000
void cmd_wifi_moduleinfo(uint8_t argc, char **argv);
void cmd_wifi_smartConfig(uint8_t argc, char **argv);
void cmd_wifi_ssidscan(uint8_t argc, char **argv);
void cmd_wifi_connect(uint8_t argc, char **argv);
void cmd_wifi_disconnect(uint8_t argc, char **argv);
void cmd_wifi_ping(uint8_t argc, char **argv);
void cmd_wifi_gethostnameip(uint8_t argc, char **argv);
#endif

#define CMD_NOPARAMS ( "This command has no parameters" )

/**************************************************************************/
/*!
    Command list for the command-line interpreter and the name of the
    corresponding method that handles the command.

    Note that a trailing ',' is required on the last entry, which will
    cause a NULL entry to be appended to the end of the table.
*/
/**************************************************************************/
cli_t cli_tbl[] =
{
  // command name, min args, max args, hidden, function name, command description, syntax
  { "?",            0,  0,  0, cmd_help                                   , "Help"                              , CMD_NOPARAMS },
  { "V",            0,  0,  0, cmd_sysinfo                                , "System Info"                       , CMD_NOPARAMS },
  { "mr",           1,  3,  0, cmd_dbg_memrd                              , "Memory read"                       , "'mr <addr> [<len> <size(1..8)>]'" },
  { "er",           1,  1,  0, cmd_eeprom_read                            , "EEPROM read"                       , "'er <addr>'" },
  { "ew",           2,  2,  0, cmd_eeprom_write                           , "EEPROM write"                      , "'ew <addr> <val>'" },
  #ifdef CFG_ENABLE_I2C
  { "is",           0,  0,  0, cmd_i2c_scan                               , "I2C bus scan"                      , CMD_NOPARAMS },
  { "ir",           2,  2,  0, cmd_i2c_read                               , "I2C read"                          , "'ir <addr> <len>'" },
  { "iw",           2,  99, 0, cmd_i2c_write                              , "I2C write"                         , "'iw <addr> <value(s)>'" },
  #endif
  #ifdef CFG_PN532
  { "du",           0,  1,  0, cmd_nfc_mifareultralight_memdump           , "Dump Mifare Ultralight card"       , "'du [<timeout>]'" },
  { "dc",           0,  1,  0, cmd_nfc_mifareclassic_memdump              , "Dump Mifare Classic card"          , "'dc [<timeout>]'" },
  { "dn",           0,  0,  0, cmd_nfc_mfc_ndef_memdump                   , "Dump Mifare Classic NFC messages"  , CMD_NOPARAMS },
  { "fn",           0,  0,  0, cmd_nfc_mfc_ndef_nfcFormat                 , "Format Mifare Classic as NFC Tag"  , CMD_NOPARAMS },
  { "fb",           0,  0,  0, cmd_nfc_mfc_ndef_blankFormat               , "Format NFC Tag as Mifare Classic"  , CMD_NOPARAMS },
  { "npn",          0,  0,  0, cmd_nfc_mfc_ndef_prepare_ndefMessage       , "Create NDEF msg in mem for 'nw'"   , CMD_NOPARAMS },
  { "nw",           0,  0,  0, cmd_nfc_mfc_ndef_write_ndef                , "Write msg from 'npn' to MFC card"  , "'nw [<sector>]'"},
  { "vc",           2,  2,  0, cmd_nfc_mifareclassic_valueblock_create    , "Create Mifare value block"         , "'vc <block> <value>'" },
  { "vi",           2,  2,  0, cmd_nfc_mifareclassic_valueblock_increment , "Increment Mifare value block"      , "'vi <block> <value>'" },
  { "vd",           2,  2,  0, cmd_nfc_mifareclassic_valueblock_decrement , "Decrement Mifare value block"      , "'vd <block> <value>'" },
  { "vr",           1,  1,  0, cmd_nfc_mifareclassic_valueblock_read      , "Read Mifare value block"           , "'vr <block> <value>'" },
  #endif
  #ifdef CFG_SDCARD
  { "d",            0,  1,  0, cmd_sd_dir                                 , "Dir (SD Card)"                     , "'d [<path>]'" },
  #endif
  #ifdef CFG_CHIBI
  { "ca",           0,  1,  0, cmd_chibi_addr                             , "Get/set node address"              , "'ca [<1-65534>|<OxFFFE>]'" },
  { "cs",           2, 99,  0, cmd_chibi_tx                               , "Send msg to node"                  , "'cs <destaddr> <msg>'" },
  #endif
  #ifdef CFG_RTC
  { "tr",           0, 0,  0, cmd_rtc_read                                , "RTC read"                           , CMD_NOPARAMS },
  { "tw",           6, 7,  0, cmd_rtc_write                               , "RTC write"                          , "'tw <yr> <mon> <day> <hr> <min> <sec>'" },
  #endif
  #ifdef CFG_CC3000
  { "winfo",        0, 0,  0, cmd_wifi_moduleinfo                         , "Wifi module info"                   , CMD_NOPARAMS },
  { "wsc",          0, 1,  0, cmd_wifi_smartConfig                        , "Wifi SmartConnect"                  , "'wsc [<usekey(0|1)>]'" },
  { "ws",           0, 0,  0, cmd_wifi_ssidscan                           , "Wifi SSID scan"                     , CMD_NOPARAMS },
  { "wc",           2, 3,  0, cmd_wifi_connect                            , "Wifi connect"                       , "'wc <sec[0|1|2|3]> <ssid> <key>'" },
  { "wd",           0, 0,  0, cmd_wifi_disconnect                         , "Wifi disconnect"                    , CMD_NOPARAMS },
  { "wp",           1, 1,  0, cmd_wifi_ping                               , "Wifi ping"                          , "'wp <ipaddress>'" },
  { "wl",           1, 1,  0, cmd_wifi_gethostnameip                      , "Wifi host name lookup"              , "'wl <hostname>'" },
  #endif
};

#ifdef __cplusplus
}
#endif

#endif
