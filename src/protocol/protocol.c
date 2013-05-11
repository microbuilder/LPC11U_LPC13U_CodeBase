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
#include "core/usb/usb_hid.h"

typedef PRE_PACK struct POST_PACK {
  uint8_t msg_type;
  uint8_t cmd_id_high;
  uint8_t cmd_id_low;
  uint8_t length;
  uint8_t payload[PROT_MAX_MSG_SIZE-4];
} protMsgCommand_t;

typedef PRE_PACK struct POST_PACK {
  uint8_t msg_type;
  uint8_t cmd_id_high;
  uint8_t cmd_id_low;
  uint8_t length;
  uint8_t payload[PROT_MAX_MSG_SIZE-4];
} protMsgResponse_t;

typedef PRE_PACK struct POST_PACK {
  uint8_t msg_type;
  uint8_t error_id_high;
  uint8_t error_id_low;
} protMsgError_t;

#define CMD_FIFO_DEPTH 128

static uint8_t ff_command_buffer[CMD_FIFO_DEPTH];
static fifo_t ff_command =
{
    .buf          = ff_command_buffer,
    .size         = CMD_FIFO_DEPTH,
    .overwritable = true,
    #if defined CFG_MCU_FAMILY_LPC11UXX
    .irq          = USB_IRQn
    #elif defined CFG_MCU_FAMILY_LPC13UXX
    .irq          = USB_IRQ_IRQn
    #else
      #error "protocol.c: No MCU defined"
    #endif
};

//------------- command lookup table -------------//
typedef protError_t (* const protCmdFunc_t)(uint8_t, uint8_t[]);

#define CMD_LOOKUP_EXPAND(command_id, function)\
  [command_id] = function, \

static protCmdFunc_t protocol_cmd_tbl[] =
{
  PROTOCOL_COMMAND_TABLE(CMD_LOOKUP_EXPAND)
};


//--------------------------------------------------------------------+
// Public API
//--------------------------------------------------------------------+
void prot_task(void * p_para)
{
  if ( fifo_getLength(&ff_command) >= 64 )
  {
    protError_t error;

    //------------- command phase -------------//
    {
      protMsgCommand_t message_cmd;
      uint16_t command_id;

      fifo_readArray(&ff_command, (uint8_t*) &message_cmd, 64);
      ASSERT( PROT_MSGTYPE_COMMAND == message_cmd.msg_type, (void) 0);

      command_id = (message_cmd.cmd_id_high << 8) + message_cmd.cmd_id_low;
      ASSERT( command_id < PROT_CMDTYPE_COUNT && message_cmd.length <= (PROT_MAX_MSG_SIZE-4), (void) 0);

      error = protocol_cmd_tbl[command_id] ( message_cmd.length, message_cmd.payload );
    }

    //------------- response phase -------------//
    {
      protMsgError_t message_error;
      message_error.msg_type = PROT_MSGTYPE_ERROR;
      message_error.error_id_high = (uint8_t)((error & 0xFFFF) << 8);
      message_error.error_id_low = (uint8_t)(error & 0xFF);
      usb_hid_generic_send( (uint8_t*) &message_error, sizeof(protMsgError_t));
    }
  }
}

#ifdef CFG_USB_HID_GENERIC
//--------------------------------------------------------------------+
// USB Generic callback
//--------------------------------------------------------------------+
// received command
void usb_hid_generic_recv_isr(uint8_t out_report[], uint32_t length)
{
  (void) length; // for simplicity, always write fixed size to fifo even if host sends out short packet

  for(uint32_t i=0; i<CFG_USB_HID_GENERIC_REPORT_SIZE; i++)
  {
    fifo_write(&ff_command, out_report[i]);
  }
}
#endif

#endif
