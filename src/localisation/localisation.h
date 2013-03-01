/**************************************************************************/
/*!
    @defgroup Localisation Localisation

    @brief    Basic localisation system to provide localised strings and
                  values.
*/
/**************************************************************************/

/**************************************************************************/
/*!
    @file     localisation.h
    @author   K. Townsend (microBuilder.eu)

    @ingroup  Localisation

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2012, K. Townsend
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
#ifndef _LOCALISATION_H_
#define _LOCALISATION_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Supported languages/cultures */
typedef enum
{
  CULTURE_EN,
  CULTURE_FR,
  CULTURE_COUNT
} culture_t;

/* Localised text index keys

   Note: The C standard (5.2.4.1) only guarantees 63 characters for names
   These names break the cardinal rule of constants in all caps, but it
   seems better to indicate case-intent in the constant to avoid issues
   with localization later                                                */
typedef enum
{
  LOCALISATION_SYMBOL_MHZ,                                   // "MHz"
  LOCALISATION_SYMBOL_SECONDS,                               // "s"
  LOCALISATION_SYMBOL_KILOBYTES,                             // "KB"
  LOCALISATION_DATETIME_JANUARY_FULL,                        // "January"
  LOCALISATION_DATETIME_FEBRUARY_FULL,                       // "February"
  LOCALISATION_DATETIME_MARCH_FULL,                          // "March"
  LOCALISATION_DATETIME_APRIL_FULL,                          // "April"
  LOCALISATION_DATETIME_MAY_FULL,                            // "May"
  LOCALISATION_DATETIME_JUNE_FULL,                           // "June"
  LOCALISATION_DATETIME_JULY_FULL,                           // "July"
  LOCALISATION_DATETIME_AUGUST_FULL,                         // "August"
  LOCALISATION_DATETIME_SEPTEMBER_FULL,                      // "September"
  LOCALISATION_DATETIME_OCTOBER_FULL,                        // "October"
  LOCALISATION_DATETIME_NOVEMBER_FULL,                       // "November"
  LOCALISATION_DATETIME_DECEMBER_FULL,                       // "December"
  LOCALISATION_DATETIME_JANUARY_SHORT,                       // "Jan"
  LOCALISATION_DATETIME_FEBRUARY_SHORT,                      // "Feb"
  LOCALISATION_DATETIME_MARCH_SHORT,                         // "Mar"
  LOCALISATION_DATETIME_APRIL_SHORT,                         // "Apr"
  LOCALISATION_DATETIME_MAY_SHORT,                           // "May"
  LOCALISATION_DATETIME_JUNE_SHORT,                          // "Jun"
  LOCALISATION_DATETIME_JULY_SHORT,                          // "Jul"
  LOCALISATION_DATETIME_AUGUST_SHORT,                        // "Aug"
  LOCALISATION_DATETIME_SEPTEMBER_SHORT,                     // "Sep"
  LOCALISATION_DATETIME_OCTOBER_SHORT,                       // "Oct"
  LOCALISATION_DATETIME_NOVEMBER_SHORT,                      // "Nov"
  LOCALISATION_DATETIME_DECEMBER_SHORT,                      // "Dec"
  LOCALISATION_TEXT_This_command_has_no_parameters,          // "This command has no parameters"
  LOCALISATION_TEXT_SD_init_failed,                          // "SD init failed"
  LOCALISATION_TEXT_No_SD_card,                              // "No SD card"
  LOCALISATION_TEXT_Failed_to_mount_partition,               // "Failed to mount partition"
  LOCALISATION_TEXT_Failed_to_open,                          // "Failed to open"
  LOCALISATION_TEXT_Contents_of,                             // "Contents of"
  LOCALISATION_TEXT_Filename,                                // "Filename"
  LOCALISATION_TEXT_Size,                                    // "Size"
  LOCALISATION_TEXT_Bytes,                                   // "Bytes"
  LOCALISATION_TEXT_bytes,                                   // "bytes"
  LOCALISATION_TEXT_Folder_Size_COLON_SPACE,                 // "Folder Size: "
  LOCALISATION_TEXT_Disk_Size_COLON_SPACE,                   // "Disk Size: "
  LOCALISATION_TEXT_Space_Available_COLON_SPACE,             // "Space Available: "
  LOCALISATION_TEXT_Firmware,                                // "Firmware"
  LOCALISATION_TEXT_MCU,                                     // "MCU"
  LOCALISATION_TEXT_Language,                                // "Language"
  LOCALISATION_TEXT_System_Clock,                            // "System Clock"
  LOCALISATION_TEXT_System_Uptime,                           // "System Uptime"
  LOCALISATION_TEXT_RF_Transceiver,                          // "RF Transceiver"
  LOCALISATION_TEXT_RF_Receive_Mode,                         // "RF Receive Mode"
  LOCALISATION_TEXT_Normal,                                  // "Normal"
  LOCALISATION_TEXT_Promiscuous,                             // "Promiscuous"
  LOCALISATION_TEST_802154_PAN_ID,                           // "802.15.4 PAN ID"
  LOCALISATION_TEXT_802154_Node_Address,                     // "802.15.4 Node Address"
  LOCALISATION_TEXT_802154_Channel,                          // "802.15.4 Channel"
  LOCALISATION_TEXT_CLI_Max_Command_Size,                    // "CLI Max Command Size"
  LOCALISATION_TEXT_CLI_IRQ_Enabled,                         // "CLI IRQ Enabled"
  LOCALISATION_TEXT_CLI_IRQ_Location,                        // "CLI IRQ Location"
  LOCALISATION_TEXT_True,                                    // "True"
  LOCALISATION_TEXT_False,                                   // "False"
  LOCALISATION_TEXT_UART_Baud_Rate,                          // "UART Baud Rate"
  LOCALISATION_TEXT_SD_Card_Present,                         // "SD Card Present"
  LOCALISATION_TEXT_FAT_File_System,                         // "FAT File System"
  LOCALISATION_TEXT_Read_Only,                               // "Read Only"
  LOCALISATION_TEXT_Read_FORWARDSLASH_Write,                 // "Read/Write"
  LOCALISATION_TEXT_LCD_Width,                               // "LCD Width"
  LOCALISATION_TEXT_LCD_Height,                              // "LCD Height"
  LOCALISATION_TEXT_LCD_Controller_ID,                       // "LCD Controller ID"
  LOCALISATION_TEXT_LCD_Small_Fonts,                         // "LCD Small Fonts"
  LOCALISATION_TEXT_LCD_AA_Fonts,                            // "LCD AA Fonts"
  LOCALISATION_TEXT_Touch_Screen,                            // "Touch Screen"
  LOCALISATION_TEXT_Touch_Screen_Threshold,                  // "Touch Screen Threshold"
  LOCALISATION_TEXT_LED_Location,                            // "LED Location"
  LOCALISATION_TEXT_Too_few_arguments,                       // "Too few arguments"
  LOCALISATION_TEXT_Expected,                                // "Expected"
  LOCALISATION_TEXT_for_more_information,                    // "for more information"
  LOCALISATION_TEXT_Too_many_arguments,                      // "Too many arguments"
  LOCALISATION_TEXT_Maximum,                                 // "Maximum"
  LOCALISATION_TEXT_Command_Not_Recognized,                  // "Command Not Recognized"
  LOCALISATION_TEXT_Type_QUESTION_for_a_list_of,             // "Type '?' for a list of all available commands"
  LOCALISATION_TEXT_Command,                                 // "Command"
  LOCALISATION_TEXT_Description,                             // "Description"
  LOCALISATION_TEXT_Command_parameters_can_be_seen,          // "Command parameters can be seen by entering: <command-name> ?"
  LOCALISATION_TEXT_Scanning_I2C_bus_for_devices,            // "Scanning I2C bus for devices"
  LOCALISATION_TEXT_All_addresses_are_in_7bit_L1,            // "All addresses are in 7-bit format and are not"
  LOCALISATION_TEXT_All_addresses_are_in_7bit_L2,            // "shifted to include the read bit."
  LOCALISATION_TEXT_No_response_on_the_I2C_bus,              // "No response on the I2C bus. Check address/connections"
  LOCALISATION_TEXT_Malformed_Number,                        // "Malformed number. Must be decimal number or hex value preceeded by '0x'"
  LOCALISATION_TEXT_Error,                                   // "Error"
  LOCALISATION_TEXT_Invalid_I2C_Address,                     // "ADDR must be between 0x03 and 0x78"
  LOCALISATION_TEXT_len_must_be_less_than_equal,             // "'len' must be <="
  LOCALISATION_TEXT_No_ACK_received,                         // "No ACK received"
  LOCALISATION_TEXT_Timeout,                                 // "Timeout"
  LOCALISATION_TEXT_Unknown_Error,                           // "Unknown Error"
  LOCALISATION_TEXT_Too_large_for_I2C_buffer,                // "Too large for I2C buffer"
  LOCALISATION_TEXT_Invalid_argument,                        // "Invalid argument"
  LOCALISATION_TEXT_OK,                                      // "OK"
  LOCALISATION_TEXT_EEPROM_Size_COLON_SPACE,                 // "EEPROM Size: "
  LOCALISATION_TEXT_Serial_Number,                           // "Serial Number: "
  LOCALISATION_TEXT_Code_Base_COLON_SPACE,                   // "Code Base: "
  LOCALISATION_FINAL
} localisedTextKeys_t;

char* localisation_GetString ( localisedTextKeys_t key );
void  localisation_SetCulture ( culture_t culture );

// Macros to make localisation a bit cleaner looking in code
#define STRING(key)    localisation_GetString(key)

#ifdef __cplusplus
}
#endif 

#endif
