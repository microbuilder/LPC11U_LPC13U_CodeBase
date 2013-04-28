/**************************************************************************/
/*!
    @file     protocol.h
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
#include "projectconfig.h"

#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Max message length is 64 bytes to align with USB HID, though this may
   be a bit large for I2C or SPI ... maybe adjust down, but in no case
   should this be more than 64 bytes                                     */
#define PROT_MAX_MSG_SIZE     (64)

/**************************************************************************/
/*!
    The first byte of every transfer defines the message type
*/
/**************************************************************************/
typedef enum
{
  /**< Command message */
  PROT_MSGTYPE_COMMAND          = 0x01,
  /**< Response to a command message */
  PROT_MSGTYPE_RESPONSE         = 0x02,
  /**< Signals an error condition */
  PROT_MSGTYPE_ERROR            = 0x80
} protMsgType_t;

//------------- X macros for create consistent command enum, function prototyp & cmd table -------------//
#define PROTOCOL_COMMAND_TABLE(ENTRY) \
    ENTRY(PROT_CMDTYPE_HELP, cmd_help)\
    ENTRY(PROT_CMDTYPE_CMD1, cmd_cmd1)\

//------------- command type -------------//
#define CMDTYPE_EXPAND(command_id, function) \
  command_id,\

typedef enum {
  PROTOCOL_COMMAND_TABLE(CMDTYPE_EXPAND)
  PROT_CMDTYPE_COUNT /**< number of commands */
}protCmdType_t;

//------------- command prototype -------------//
#define CMD_PROTOTYPE_EXPAND(command_id, function) \
  void function(uint8_t argc, char **argv);\

PROTOCOL_COMMAND_TABLE(CMD_PROTOTYPE_EXPAND);

typedef void (* const protCmdFunc_t)(uint8_t, char**);
protCmdFunc_t protocol_cmd_tbl[];

#ifdef __cplusplus
}
#endif

#endif
