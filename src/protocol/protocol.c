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
#define CMD_LOOKUP_EXPAND(command_id, function)\
  [command_id] = function, \

protCmdFunc_t protocol_cmd_tbl[] =
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
    uint8_t message[64];
    uint16_t command_id;

    fifo_readArray(&ff_command, message, 64);
    ASSERT( PROT_MSGTYPE_COMMAND == message[0], (void) 0);

    command_id = (message[1] << 8) + message[2];
    ASSERT( command_id < PROT_CMDTYPE_COUNT, (void) 0);

    protocol_cmd_tbl[command_id] ( message[3], message+4 );
  }
}

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
