/**************************************************************************/
/*!
    @file     protocol.c
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

#ifdef CFG_PROTOCOL

//--------------------------------------------------------------------+
// INCLUDE & DECLARATION
//--------------------------------------------------------------------+
#include "protocol.h"
#include "core/fifo/fifo.h"

#if defined(CFG_PROTOCOL_VIA_HID)
  #define command_received_isr usb_hid_generic_recv_isr
  #define command_send         usb_hid_generic_send
#elif defined(CFG_PROTOCOL_VIA_BULK)
  #define command_received_isr usb_custom_received_isr
  #define command_send         usb_custom_send
#endif

#define U16_HIGH_U8(u16) ((uint8_t) (((u16) >> 8) & 0x00FF))
#define U16_LOW_U8(u16) ( (uint8_t) ( (u16)& 0x00FF) )

/**************************************************************************/
/*!
    @brief standard function type for a protocol command

    @param[in]  payload's length

    @param[in]  payload's content

    @param[out] response message, already filled with msg type & command id

    @returns  error code if there is, otherwise ERROR_NONE
*/
/**************************************************************************/
typedef error_t (* const protCmdFunc_t)(uint8_t, uint8_t const [], protMsgResponse_t*);

//------------- command prototype -------------//
#define CMD_PROTOTYPE_EXPAND(command, function) \
  error_t function(uint8_t length, uint8_t const payload[], protMsgResponse_t* mess_response);\

PROTOCOL_COMMAND_TABLE(CMD_PROTOTYPE_EXPAND);

//------------- command lookup table -------------//
#define CMD_LOOKUP_EXPAND(command, function)\
  [command] = function,\

static protCmdFunc_t protocol_cmd_tbl[] =
{
  PROTOCOL_COMMAND_TABLE(CMD_LOOKUP_EXPAND)
};


#define CMD_FIFO_DEPTH 4

#if defined CFG_MCU_FAMILY_LPC11UXX
  FIFO_DEF(ff_command, CMD_FIFO_DEPTH, protMsgCommand_t, true , USB_IRQn);
#elif defined CFG_MCU_FAMILY_LPC13UXX
  FIFO_DEF(ff_command, CMD_FIFO_DEPTH, protMsgCommand_t, true , USB_IRQ_IRQn);
#else
  #error __FILE__ No MCU defined
#endif

//--------------------------------------------------------------------+
// Public API
//--------------------------------------------------------------------+
void prot_init(void)
{
  fifo_clear(&ff_command);
}

void prot_task(void * p_para)
{
  if ( !fifo_isEmpty(&ff_command) )
  {
    error_t error;

    //------------- command received -------------//
    protMsgCommand_t message_cmd = { 0 };
    protMsgResponse_t message_reponse = {0};
    uint16_t command_id;

    fifo_read(&ff_command, &message_cmd);

    // command_id is at odd address, directly use the value in message_cmd can lead to alignment issue on M0
    command_id = (message_cmd.cmd_id_high << 8) + message_cmd.cmd_id_low;

    if ( !(PROT_MSGTYPE_COMMAND == message_cmd.msg_type && 0 < command_id && command_id < PROT_CMDTYPE_COUNT) )
    {
      error = ERROR_PROT_UNKNOWN_COMMAND;
    }else if (message_cmd.length > (PROT_MAX_MSG_SIZE-4))
    {
      error = ERROR_PROT_INVALID_PARAM;
    } else
    {
      message_reponse.msg_type    = PROT_MSGTYPE_RESPONSE;
      message_reponse.cmd_id_high = message_cmd.cmd_id_high;
      message_reponse.cmd_id_low  = message_cmd.cmd_id_low;

      // invoke callback before executing command
      if (prot_cmd_received_cb)
      {
        prot_cmd_received_cb(&message_cmd);
      }

      //------------- command executed: invoke the command handler associated with command_id-------------//
      error = protocol_cmd_tbl[command_id] ( message_cmd.length, message_cmd.payload, &message_reponse );
    }

    //------------- response phase -------------//
//    uint32_t start_time = delayGetSecondsActive();
//    while (  )
    // TODO make sure usb command is ready to send
    // in case there are a bunch of cmd queued in fifo

    if (error == ERROR_NONE)
    {
      if (prot_cmd_executed_cb)
      {
        prot_cmd_executed_cb(&message_reponse);
      }
      command_send( &message_reponse, sizeof(protMsgResponse_t));
    }else
    {
      protMsgError_t message_error =
      {
          .msg_type      = PROT_MSGTYPE_ERROR,
          .error_id_high = U16_HIGH_U8(error),
          .error_id_low  = U16_LOW_U8 (error)
      };

      if (prot_cmd_error_cb)
      {
        prot_cmd_error_cb(&message_error);
      }
      command_send( &message_error, sizeof(protMsgError_t));
    }
  }
}

/**************************************************************************/
/*!
    USB HID Generic callback (captures incoming commands)
*/
/**************************************************************************/
void command_received_isr(void * p_data, uint32_t length)
{
  (void) length; // for simplicity, always write fixed size to fifo even if host sends out short packet
  fifo_write(&ff_command, p_data);
}

#endif
