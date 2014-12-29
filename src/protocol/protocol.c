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

/*  SIMPLE BINARY PROTOCOL DESCRIPTION
    ==================================

    This simple messaging protocol can be used to send and receive binary
    messages using any binary serial bus (USB HID, USB Bulk, SPI, I2C,
    Wireless, etc.).

    The protocol is designed to be flexible and extensible, with the only
    requirement being that individual messages are 64 bytes or smaller,
    and that the first byte of every message is a one byte (U8) identifier
    that indicates the format for the rest of the payload (if there is one).

    IMPLEMENTATION NOTES
    ====================

    The protocol implementation must be transport mechanism agnostic, and
    should be able to function with any binary serial bus. The original
    design goal is a protocol that can function via USB HID, SPI Slave,
    and I2C Slave modes, but any similar protocol should be able to be
    added to the implementation (wireless transfers, etc.).

    ENDIANNESS
    ==========

    All values larger than 8-bits are expected to be little endian, such
    as the command IDs, and the payload contents.  Any deviation from
    this rule should be clearly documented.

    MESSAGE TYPES
    =============

    Each message is preceded by an 8-bit message identifier that indicates
    the format and content of the rest of the message:

    |-----------------------+---------+--------------------------------------|
    | Message Type          | ID (U8) | Meaning                              |
    |-----------------------+---------+--------------------------------------|
    | Command               | 0x10    |                                      |
    | Response              | 0x20    |                                      |
    | Alert                 | 0x40    |                                      |
    | Error                 | 0x80    |                                      |
    |-----------------------+---------+--------------------------------------|

    COMMAND MESSAGES
    ================

    Command messages (Message Type = 0x10) have the following structure:

    |-------------------+----------+-----------------------------------------|
    | Name              | Type     | Meaning                                 |
    |-------------------+----------+-----------------------------------------|
    | Message Type      | U8       | Always '0x10'                           |
    | Command ID        | U16      | Unique command identifier               |
    | Payload Length    | U8       | Payload length (0..60)                  |
    | Payload           | ...      | Optional command payload (params, etc.) |
    |-------------------+----------+-----------------------------------------|

    'Command ID' (bytes 1-2) and 'Payload Length' (byte 3) are mandatory in
    any command message.  The message payload is optional, and will be ignored
    if Payload Length is set to '0' bytes.  When a message payload is present,
    it's length can be anywhere from 1..60 bytes, to stay within the 64 byte
    maximum message length.

    The contents of the payload is user defined, and can change for each
    command. Any payload error checking should be done by the individual
    command handler.

    A sample command message would be:

    Byte   0    1  2    3    4
          [10] [34 12] [01] [FF]

    - The first byte is the Message Type (0x10), which identifies this as a
      command message.
    - The second and third bytes are 0x1234 (34 12 in little-endian notation),
      which is the unique command ID that will be parsed to the command lookup
      function, and redirected to an appropriate command handler function.
    - The fourth byte indicates that we have a message payload of 1 byte
    - The fifth bytes is the 1 byte payload: 0xFF

    RESPONSE MESSAGES
    =================

    Responses messages (Message Type = 0x20) are generated in response to an
    incoming command, and have the following structure:

    |-------------------+----------+-----------------------------------------|
    | Name              | Type     | Meaning                                 |
    |-------------------+----------+-----------------------------------------|
    | Message Type      | U8       | Always '0x20'                           |
    | Command ID        | U16      | Command ID of the command this message  |
    |                   |          | is a response to, to correlate          |
    |                   |          | Responses and commands                  |
    | Payload Length    | U8       | Payload length (0..60)                  |
    | Payload           | ...      | Optional response payload               |
    |-------------------+----------+-----------------------------------------|

    By including the 'Command ID' that this response message is related to,
    the recipient can more easily correlate responses and commands.  This is
    useful in situations where multiple commands are sent, and some commands
    may take a longer period of time to execute than subsequent commands with
    a different command ID.

    Response message can only be generates in response to a command message,
    so the Command ID should always be present.

    If more precise command/response correlation is required, a custom
    protocol can be developped, where a unique message identifier is included
    in the payload of each command/response, but this is beyond the scope of
    this high-level protocol.

    A sample response message would be:

    Byte   0    1  2    3    4
          [20] [34 12] [01] [FF]

    - The first byte is the Message Type (0x20), which identifies this as a
      response message.
    - The second and third bytes are 0x1234, which is the unique command ID
      that this response is related to.
    - The fourth byte indicates that we have a message payload of 1 byte
    - The fifth byte is the 1 byte payload: 0xFF

    ALERT MESSAGES
    ==============

    Alert messages (Message Type = 0x40) are sent whenever an alert condition
    is present on the system (low battery, etc.), and have the following
    structure:

    |-------------------+----------+-----------------------------------------|
    | Name              | Type     | Meaning                                 |
    |-------------------+----------+-----------------------------------------|
    | Message Type      | U8       | Always '0x40'                           |
    | Alert ID          | U16      | Unique ID for the alert condition       |
    | Payload Length    | U8       | Payload length (0..60)                  |
    | Payload           | ...      | Optional response payload               |
    |-------------------+----------+-----------------------------------------|

    A sample alert message would be:

    Byte   0    1  2    3    4    5    6    7
          [40] [34 12] [04] [42] [07] [00] [10]

    - The first byte is the Message Type (0x40), which identifies this as an
      alert message.
    - The second and third bytes are 0x1234, which is the unique alert ID.
    - The fourth byte indicates that we have a message payload of 4 byte
    - The last four bytes are the actual payload: '0x10000742' in this case,
      assuming we were transmitting a 32-bit value in little-endian format.

    ERROR MESSAGES
    ==============

    Error messages (Message Type = 0x80) are returned whenever an error
    condition is present on the system, and have the following structure:

    |-------------------+----------+-----------------------------------------|
    | Name              | Type     | Meaning                                 |
    |-------------------+----------+-----------------------------------------|
    | Message Type      | U8       | Always '0x80'                           |
    | Error ID          | U16      | Unique ID for the error condition       |
    |-------------------+----------+-----------------------------------------|

    Whenever an error condition is present and the system needs to be alerted
    (such as a failed request, an attempt to access a non-existing resource,
    etc.) the system can return a specific error message with an appropriate
    Error ID.

    A sample error message would be:

    Byte   0    1  2
          [80] [00 01]

    - The first byte is the Message Type (0x80), which identifies this is an
      error message.
    - The second and third bytes are 0x0100 (00 01 in little-endian notation),
      which is the error code corresponding to PROT_ERROR_INVALID_PARAM.

*/

#include "projectconfig.h"

#ifdef CFG_PROTOCOL

#include "protocol.h"
#include "core/fifo/fifo.h"

/* Callback functions to let us know when new data arrives via USB, etc. */
#if defined(CFG_PROTOCOL_VIA_HID)
  #define command_received_isr  usb_hid_generic_recv_isr
  #define command_send          usb_hid_generic_send
#elif defined(CFG_PROTOCOL_VIA_BULK)
  #define command_received_isr  usb_custom_received_isr
  #define command_send          usb_custom_send
#endif

#define U16_HIGH_U8(u16)  ((uint8_t) (((u16) >> 8) & 0x00FF))
#define U16_LOW_U8(u16)   ((uint8_t) ((u16) & 0x00FF))

/**************************************************************************/
/*!
    @brief      The standard function prototype for protocol commands (all
                commnands need to implement the same signature)

    @param[in]  Payload length (normally max 64 bytes)

    @param[in]  Payload contents

    @param[out] Response message, already filled with msg type and cmd id

    @returns    Error code if there is an error, otherwise ERROR_NONE
*/
/**************************************************************************/
typedef err_t (* const protCmdFunc_t)(uint8_t, uint8_t const [], protMsgResponse_t*);

/**************************************************************************/
/*
    Expands the function to have the standard function signature
*/
/**************************************************************************/
#define CMD_PROTOTYPE_EXPAND(command, function) \
  err_t function(uint8_t length, uint8_t const payload[], protMsgResponse_t* mess_response);\

PROTOCOL_COMMAND_TABLE(CMD_PROTOTYPE_EXPAND);

/**************************************************************************/
/*
    Expands the command/function combination to something that we can
    insert into the command lookup table (functions are expanded to have
    the full function signature)
*/
/**************************************************************************/
#define CMD_LOOKUP_EXPAND(command, function)\
  [command] = function,\

/**************************************************************************/
/*!
    The command lookup table implementation, which is based on the
    commmands and implementation functions defined in prot_cmdtable.h.

    The actual command/function pairs are broken off into a separate
    header file so that you can more easily change the support command
    list from project to project without changing the underlying
    protocol code and files.
*/
/**************************************************************************/
static protCmdFunc_t protocol_cmd_tbl[] =
{
  PROTOCOL_COMMAND_TABLE(CMD_LOOKUP_EXPAND)
};

/* FIFO buffer for incoming commands (Note: 64 bytes per command) */
#define CMD_FIFO_DEPTH 4

#if defined CFG_MCU_FAMILY_LPC11UXX
  FIFO_DEF(ff_prot_cmd, CMD_FIFO_DEPTH, protMsgCommand_t, true , USB_IRQn);
#elif defined CFG_MCU_FAMILY_LPC13UXX
  FIFO_DEF(ff_prot_cmd, CMD_FIFO_DEPTH, protMsgCommand_t, true , USB_IRQ_IRQn);
#else
  #error __FILE__ No MCU defined
#endif

/**************************************************************************/
/*!
    @brief      Initialises the simple binary protocol (FIFO init, etc.)
*/
/**************************************************************************/
void prot_init(void)
{
  fifo_clear(&ff_prot_cmd);
}

/**************************************************************************/
/*!
    @brief      Checks if there are any commands for the simple binary
                protocol to process in the FIFO, and hands them off to the
                command parser if anything was found

    @code

    // Note: Assumes CFG_PROTOCOL is defined in the board config file!

    prot_init();

    // Commands will be added to the protocol FIFO as they arrive via
    // the command_received_isr callback further down in this file

    while(1)
    {
      // Constantly check for incoming messages (this can of course be
      // handled more efficiently depending on your requirements)
      prot_task(NULL);
    }

    @endcode
*/
/**************************************************************************/
void prot_task(void * p_para)
{
  if ( !fifo_isEmpty(&ff_prot_cmd) )
  {
    /* If we get here, it means a command was received */
    protMsgCommand_t  message_cmd     = { 0 };
    protMsgResponse_t message_reponse = { 0 };
    uint16_t          command_id;
    err_t           error;

    /* COMMAND PHASE */
    fifo_read(&ff_prot_cmd, &message_cmd);

    /* Command_id is at an odd address ... directly using the value in *
     * message_cmd can lead to alignment issues on the M0              */
    command_id = (message_cmd.cmd_id_high << 8) + message_cmd.cmd_id_low;

    /* Make sure we have a command with a valid ID */
    if ( !(PROT_MSGTYPE_COMMAND == message_cmd.msg_type) )
    {
      error = ERROR_PROT_INVALIDMSGTYPE;
    }
    else if ( !(0 < command_id && command_id < PROT_CMDTYPE_COUNT) )
    {
      error = ERROR_PROT_INVALIDCOMMANDID;
    }
    else if (message_cmd.length > (PROT_MAX_MSG_SIZE-4))
    {
      error = ERROR_INVALIDPARAMETER;
    }
    else
    {
      /* Keep track of the command ID for the response message */
      message_reponse.msg_type    = PROT_MSGTYPE_RESPONSE;
      message_reponse.cmd_id_high = message_cmd.cmd_id_high;
      message_reponse.cmd_id_low  = message_cmd.cmd_id_low;

      /* Invoke 'cmd_received' callback before executing command */
      if (prot_cmd_received_cb)
      {
        prot_cmd_received_cb(&message_cmd);
      }

      /* Fire the appropriate handler based on the command ID */
      error = protocol_cmd_tbl[command_id] ( message_cmd.length, message_cmd.payload, &message_reponse );
    }

    /* RESPONSE PHASE */

    // TODO:  Make sure the usb command is ready to send
    // in case there are a bunch of cmds queued in FIFO

    if (error == ERROR_NONE)
    {
      /* Invoke the 'cmd_executed' callback */
      if (prot_cmd_executed_cb)
      {
        prot_cmd_executed_cb(&message_reponse);
      }

      /* Send the response message (cmd successfully executed) */
      command_send( (uint8_t*) &message_reponse, sizeof(protMsgResponse_t));
    }
    else
    {
      /* Something went wrong ... parse the error ID */
      protMsgError_t message_error =
      {
        .msg_type      = PROT_MSGTYPE_ERROR,
      };
      message_error.error_id_high = U16_HIGH_U8(error);
      message_error.error_id_low  = U16_LOW_U8 (error);

      /* Invoke the 'cmd_error' callback */
      if (prot_cmd_error_cb)
      {
        prot_cmd_error_cb(&message_error);
      }

      /* Send back a mandatory error message */
      command_send( (uint8_t*)  &message_error, sizeof(protMsgError_t));
    }
  }
}

/**************************************************************************/
/*!
    USB callback for incoming commands (the exact callback function
    depends on the interface used by the simple binary protocol, and is
    defined in a macro at the top of this file).

    This callback will write the incoming command into the FIFO for
    processing by prot_task when there is enough bandwidth to run
    the command parser.
*/
/**************************************************************************/
void command_received_isr(uint8_t * p_data, uint32_t length)
{
  fifo_write(&ff_prot_cmd, p_data);
}

#endif
