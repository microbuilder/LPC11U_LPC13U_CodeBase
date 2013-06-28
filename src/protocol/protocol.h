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
#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#include "projectconfig.h"
#include "prot_cmdtable.h"
#include "core/usb/usb_hid.h"
#include "core/usb/usb_custom_class.h"

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
  PROT_MSGTYPE_COMMAND          = 0x10,
  PROT_MSGTYPE_RESPONSE         = 0x20,
  PROT_MSGTYPE_ALERT            = 0x40,
  PROT_MSGTYPE_ERROR            = 0x80
} protMsgType_t;

/**************************************************************************/
/*!
    Command message struct
*/
/**************************************************************************/
typedef PRE_PACK struct POST_PACK {
  uint8_t msg_type;
  union {
    uint16_t cmd_id;
    struct {
      uint8_t cmd_id_low;     // Little-endian encoding
      uint8_t cmd_id_high;
    };
  };
  uint8_t length;
  uint8_t payload[PROT_MAX_MSG_SIZE-4];
} protMsgCommand_t;

STATIC_ASSERT(sizeof(protMsgCommand_t) == 64);

/**************************************************************************/
/*!
    Response message struct
*/
/**************************************************************************/
typedef protMsgCommand_t protMsgResponse_t;

/**************************************************************************/
/*!
    Alert message struct
*/
/**************************************************************************/
typedef protMsgCommand_t protMsgAlert_t;

/**************************************************************************/
/*!
    Error message struct
*/
/**************************************************************************/
typedef PRE_PACK struct POST_PACK {
  uint8_t msg_type;
  union {
    uint16_t error_id;
    struct {
      uint8_t error_id_low;   // Little-endian encoding
      uint8_t error_id_high;
    };
  };
} protMsgError_t;

STATIC_ASSERT(sizeof(protMsgError_t) == 3);

//--------------------------------------------------------------------+
// PUBLIC API
//--------------------------------------------------------------------+
void prot_task(void * p_para);
void prot_init(void);

//--------------------------------------------------------------------+
// Callback API
//--------------------------------------------------------------------+
void prot_cmd_received_cb(protMsgCommand_t const * p_mess) __attribute__ ((weak));
void prot_cmd_executed_cb(protMsgResponse_t const * p_resonse) __attribute__ ((weak));
void prot_cmd_error_cb(protMsgError_t const * p_error) __attribute__ ((weak));

#ifdef __cplusplus
}
#endif

#endif
