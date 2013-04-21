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

/* This is an attempt to have a generic messaging protocol to send and
   receive commands via USB HID, SPI Slave (MCU = SPI device), I2C Slave
   (MCU = I2C device) using binary data, similar to the way cli handles
   byte data.

   As commands come in via USB HID, SPI or I2C interrupt handlers, they
   should be placed in a queue, and if they are commands, should be sent
   to the command lookup table, which redirects them to the appropriate
   function.

   If required, these functions can generate response or error messages
   and send them back out on whatever endpoint is select in the config
   file (HID, SPI or I2C).

   This should all be peripherals neutral, and should work with USB HID,
   SPI or I2C transparently.  See the way the 'CLI' code works with both
   USB CDC and UART for an example, since this is what we are trying
   to emulation but with different protocols and binary data.            */

/* Command messages have the following structure:
   U8   Message Type (PROT_MSGTYPE_COMMAND)
   U16  Command ID (used to lookup function in protocol_cmd_tbl)
   ..   Optional args (61 bytes max!)                                    */

/* Response message have the following structure:
   U8   Message Type (PROT_MSGTYPE_RESPONSE)
   U16  Command ID (CMD ID that this msg is a response to)
   ..   Response content (61 bytes max!)                                 */

/* ACK messages are a single byte, and contain PROT_MSGTYPE_ACK          */

/* Error messages have the following structure:
   U8   Message Type (PROT_MSGTYPE_ERROR)
   U16  Error Code
   ..   Optional error arguments (if any, max 61 bytes)                  */

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
  /**< 1-byte ACK */
  PROT_MSGTYPE_ACK              = 0x40,
  /**< Signals an error condition */
  PROT_MSGTYPE_ERROR            = 0x80
} protMsgType_t;

#ifdef __cplusplus
}
#endif

#endif
