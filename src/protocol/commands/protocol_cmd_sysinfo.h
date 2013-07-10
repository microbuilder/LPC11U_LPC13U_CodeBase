/**************************************************************************/
/*!
    @file     protocol_cmd_sysinfo.h
    @author   K. Townsend (microBuilder.eu)

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

#ifndef __PROTOCOL_CMD_SYSINFO_H__
#define __PROTOCOL_CMD_SYSINFO_H__

#ifdef __cplusplus
 extern "C" {
#endif

/**************************************************************************/
/*!
    SYSINFO Keys (indicates what specific system information we want)
*/
/**************************************************************************/
typedef enum
{
  PROT_CMD_SYSINFO_KEY_FIRST                = 0x0000,
  PROT_CMD_SYSINFO_KEY_CODEBASE_VERSION     = 0x0001,   /**< Code base version (3*U8) */
  PROT_CMD_SYSINFO_KEY_FIRMWARE_VERSION     = 0x0002,   /**< Firmware version (3*U8) */
  PROT_CMD_SYSINFO_KEY_MCU_STRING           = 0x0003,   /**< MCU model (string) */
  PROT_CMD_SYSINFO_KEY_SERIAL_NUMBER        = 0x0004,   /**< Unique on-chip serial number (4*U32) */
  PROT_CMD_SYSINFO_KEY_CLOCKSPEED           = 0x0005,   /**< Core clock speed in Hz (U32) */
  PROT_CMD_SYSINFO_KEY_EEPROMSIZE           = 0x0006,   /**< EEPROM size in bytes (U32) */
  PROT_CMD_SYSINFO_KEY_LAST
} prot_cmd_sysinfo_key_t;

#ifdef __cplusplus
 }
#endif

#endif /* __PROTOCOL_CMD_LED_H__ */

/** @} */
