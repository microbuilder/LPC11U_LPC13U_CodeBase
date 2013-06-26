/**************************************************************************/
/*!
    @file     test_fifo.c
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
#include <stddef.h>
#include "unity.h"
#include "fifo.h"
#include "protocol.h"
#include "protocol_support.h"
#include "protocol_cmd_led.h"

#include "mock_usb_hid.h"
#include "mock_usb_custom_class.h"
#include "mock_board.h"
#include "mock_protocol_callback.h"

static protMsgCommand_t message_cmd;

void setUp(void)
{
  prot_init();
  memset(&message_cmd, 0, sizeof(protMsgCommand_t) );
}

void tearDown(void)
{

}

#ifdef CFG_PROTOCOL

static void stub_cmd_err_cb(protMsgError_t const * p_mess_err, int num_call)
{
  TEST_ASSERT_NOT_NULL(p_mess_err);
  TEST_ASSERT_EQUAL(PROT_MSGTYPE_ERROR, p_mess_err->msg_type);
  TEST_ASSERT_EQUAL(ERROR_PROT_UNKNOWN_COMMAND, p_mess_err->error_id);
}

static bool stub_command_send(void const * p_data, uint32_t length, int num_call)
{
  protMsgError_t *p_error = (protMsgError_t *) p_data;
  TEST_ASSERT_NOT_NULL(p_error);
  TEST_ASSERT_EQUAL(3, length);
  TEST_ASSERT_EQUAL(PROT_MSGTYPE_ERROR, p_error->msg_type);
  TEST_ASSERT_EQUAL(ERROR_PROT_UNKNOWN_COMMAND, p_error->error_id);
}

//--------------------------------------------------------------------+
// COMMON
//--------------------------------------------------------------------+
void test_message_structure(void)
{
  TEST_ASSERT_EQUAL(0, offsetof(protMsgCommand_t, msg_type) );

  TEST_ASSERT_EQUAL(1, offsetof(protMsgCommand_t, cmd_id) );
  TEST_ASSERT_EQUAL(1, offsetof(protMsgCommand_t, cmd_id_low) );
  TEST_ASSERT_EQUAL(2, offsetof(protMsgCommand_t, cmd_id_high) );

  TEST_ASSERT_EQUAL(3, offsetof(protMsgCommand_t, length) );
  TEST_ASSERT_EQUAL(4, offsetof(protMsgCommand_t, payload) );

  TEST_ASSERT_EQUAL(64, sizeof(protMsgCommand_t));
}

void test_error_structure(void)
{
  TEST_ASSERT_EQUAL(0, offsetof(protMsgError_t, msg_type) );

  TEST_ASSERT_EQUAL(1, offsetof(protMsgError_t, error_id) );
  TEST_ASSERT_EQUAL(1, offsetof(protMsgError_t, error_id_low) );
  TEST_ASSERT_EQUAL(2, offsetof(protMsgError_t, error_id_high) );

  TEST_ASSERT_EQUAL(3, sizeof(protMsgError_t));
}

void test_invalid_message_type(void)
{
  message_cmd.msg_type = 0;

  fifo_write(&ff_command, &message_cmd);

  prot_cmd_error_cb_StubWithCallback( stub_cmd_err_cb );
  MOCK_PROT(command_send, _StubWithCallback) ( stub_command_send );

  //------------- Code Under Test -------------//
  prot_task(NULL);

}

void test_invalid_command(void)
{
  message_cmd.msg_type = PROT_MSGTYPE_COMMAND;
  message_cmd.cmd_id   = 0;

  fifo_write(&ff_command, &message_cmd);

  prot_cmd_error_cb_StubWithCallback( stub_cmd_err_cb );
  MOCK_PROT(command_send, _StubWithCallback) ( stub_command_send );

  //------------- Code Under Test -------------//
  prot_task(NULL);

}

#endif
