/**************************************************************************/
/*!
    @file     test_protocol_cmd_sysinfo.c
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

#include <string.h>
#include "unity.h"
#include "fifo.h"
#include "protocol.h"
#include "protocol_cmd_led.h"
#include "protocol_cmd_sysinfo.h"
#include "protocol_support.h"

#include "mock_iap.h"
#include "mock_usb_hid.h"
#include "mock_usb_custom_class.h"
#include "mock_board.h"
#include "mock_protocol_callback.h"

uint32_t SystemCoreClock = 12000000; // overshadow the variable used to determine core lock

static protMsgCommand_t  message_cmd;
static protMsgError_t    message_error;
static protMsgResponse_t message_response;

#define U16_HIGH_U8(u16)  ((uint8_t) (((u16) >> 8) & 0x00FF))
#define U16_LOW_U8(u16)   ((uint8_t) ((u16) & 0x00FF))

void setUp(void)
{
  prot_init();
  memset(&message_cmd, 0, sizeof(protMsgCommand_t) );
}

void tearDown(void)
{

}

/**************************************************************************/
/*!
    Makes sure sys info commands with an invalid payload are rejected

    Every sysinfo command must have at least a two-byte key. This function
    sends a sysinfo command with a zero-length payload (no key).
*/
/**************************************************************************/
void test_sysinfo_invalid_length_zero(void)
{
  message_cmd = (protMsgCommand_t)
  {
    .msg_type    = PROT_MSGTYPE_COMMAND,
    .cmd_id      = PROT_CMDTYPE_SYSINFO,
    .length      = 0,
    .payload     = { U16_LOW_U8(PROT_CMD_SYSINFO_KEY_CODEBASE_VERSION),  U16_HIGH_U8(PROT_CMD_SYSINFO_KEY_CODEBASE_VERSION) }
  };

  message_error = (protMsgError_t)
  {
    .msg_type = PROT_MSGTYPE_ERROR,
    .error_id = ERROR_PROT_INVALIDPAYLOAD
  };

  fifo_write(&ff_prot_cmd, &message_cmd);

  prot_cmd_received_cb_Expect(&message_cmd);
  prot_cmd_error_cb_Expect(&message_error);
  MOCK_PROT(command_send, _IgnoreAndReturn)(LPC_OK);

  //------------- Code Under Test -------------//
  prot_task(NULL);
}

/**************************************************************************/
/*!
    Makes sure sys info commands with an invalid payload are rejected

    Sends a sysinfo command with an unexpectedly large payload
*/
/**************************************************************************/
void test_sysinfo_invalid_length_too_long(void)
{
  message_cmd = (protMsgCommand_t)
  {
    .msg_type    = PROT_MSGTYPE_COMMAND,
    .cmd_id      = PROT_CMDTYPE_SYSINFO,
    .length      = 40,
    .payload     = { U16_LOW_U8(PROT_CMD_SYSINFO_KEY_CODEBASE_VERSION),  U16_HIGH_U8(PROT_CMD_SYSINFO_KEY_CODEBASE_VERSION) }
  };

  message_error = (protMsgError_t)
  {
    .msg_type = PROT_MSGTYPE_ERROR,
    .error_id = ERROR_PROT_INVALIDPAYLOAD
  };

  fifo_write(&ff_prot_cmd, &message_cmd);

  prot_cmd_received_cb_Expect(&message_cmd);
  prot_cmd_error_cb_Expect(&message_error);
  MOCK_PROT(command_send, _IgnoreAndReturn)(LPC_OK);

  //------------- Code Under Test -------------//
  prot_task(NULL);
}

/**************************************************************************/
/*!
    Make sure that sysinfo commands with invalid keys are rejected
*/
/**************************************************************************/
void test_sysinfo_invalid_key(void)
{
  message_cmd = (protMsgCommand_t)
  {
    .msg_type    = PROT_MSGTYPE_COMMAND,
    .cmd_id      = PROT_CMDTYPE_SYSINFO,
    .length      = 40,
    .payload     = { U16_LOW_U8(PROT_CMD_SYSINFO_KEY_LAST),  U16_HIGH_U8(PROT_CMD_SYSINFO_KEY_LAST) }
  };

  message_error = (protMsgError_t)
  {
    .msg_type = PROT_MSGTYPE_ERROR,
    .error_id = ERROR_INVALIDPARAMETER
  };

  fifo_write(&ff_prot_cmd, &message_cmd);

  prot_cmd_received_cb_Expect(&message_cmd);
  prot_cmd_error_cb_Expect(&message_error);
  MOCK_PROT(command_send, _IgnoreAndReturn)(LPC_OK);

  //------------- Code Under Test -------------//
  prot_task(NULL);
}

/**************************************************************************/
/*!
    Basic PROT_CMD_SYSINFO_KEY_CODEBASE_VERSION tester
*/
/**************************************************************************/
void test_sysinfo_codebase_version(void)
{
  /* Define the command message that will be sent */
  message_cmd = (protMsgCommand_t)
  {
    .msg_type    = PROT_MSGTYPE_COMMAND,
    .cmd_id      = PROT_CMDTYPE_SYSINFO,
    .length      = 2,
    .payload     = { U16_LOW_U8(PROT_CMD_SYSINFO_KEY_CODEBASE_VERSION),
                     U16_HIGH_U8(PROT_CMD_SYSINFO_KEY_CODEBASE_VERSION) }
  };

  /* Define the response message that we are expecting */
  message_response = (protMsgResponse_t)
  {
    .msg_type    = PROT_MSGTYPE_RESPONSE,
    .cmd_id      = PROT_CMDTYPE_SYSINFO,
    .length      = 3,
    .payload     = { (uint8_t)CFG_CODEBASE_VERSION_MAJOR,
                     (uint8_t)CFG_CODEBASE_VERSION_MINOR,
                     (uint8_t)CFG_CODEBASE_VERSION_REVISION }
  };

  /* Put the command message into the protocol FIFO buffer */
  fifo_write(&ff_prot_cmd, &message_cmd);

  /* Indicate what we're expecting in the prot_cmd_received_cb callback */
  prot_cmd_received_cb_Expect(&message_cmd);

  /* Indicate what we're expecting in the prot_cmd_executed_cb callback */
  prot_cmd_executed_cb_Expect(&message_response);

  MOCK_PROT(command_send, _IgnoreAndReturn)(LPC_OK);

  /* ------------- Code Under Test ------------- */
  prot_task(NULL);
}

/**************************************************************************/
/*!
    Basic PROT_CMD_SYSINFO_KEY_SERIAL_NUMBER tester
*/
/**************************************************************************/
static uint32_t expected_uid[4] = {0x12345678, 0x87654321, 0xCAFEBABE, 0xBABECAFE };
err_t fake_iapReadUID(uint32_t uid[], int num_call)
{
  memcpy(uid, expected_uid, 16);

  return ERROR_NONE;
}

void test_sysinfo_uid(void)
{

  /* Define the command message that will be sent */
  message_cmd = (protMsgCommand_t)
  {
    .msg_type    = PROT_MSGTYPE_COMMAND,
    .cmd_id      = PROT_CMDTYPE_SYSINFO,
    .length      = 2,
    .payload     = { U16_LOW_U8 (PROT_CMD_SYSINFO_KEY_SERIAL_NUMBER),
                     U16_HIGH_U8(PROT_CMD_SYSINFO_KEY_SERIAL_NUMBER) }
  };

  /* Define the response message that we are expecting */
  message_response = (protMsgResponse_t)
  {
    .msg_type    = PROT_MSGTYPE_RESPONSE,
    .cmd_id      = PROT_CMDTYPE_SYSINFO,
    .length      = 16,
  };

  memcpy(message_response.payload, expected_uid, 16);

  /* Put the command message into the protocol FIFO buffer */
  fifo_write(&ff_prot_cmd, &message_cmd);

  /* Indicate what we're expecting in the prot_cmd_received_cb callback */
  prot_cmd_received_cb_Expect(&message_cmd);

  iapReadUID_StubWithCallback(fake_iapReadUID);
  /* Indicate what we're expecting in the prot_cmd_executed_cb callback */
  prot_cmd_executed_cb_Expect(&message_response);

  MOCK_PROT(command_send, _IgnoreAndReturn)(LPC_OK);

  /* ------------- Code Under Test ------------- */
  prot_task(NULL);
}
