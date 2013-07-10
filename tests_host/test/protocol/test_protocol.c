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
#include "protocol_cmd_sysinfo.h"

#include "mock_iap.h"
#include "mock_usb_hid.h"
#include "mock_usb_custom_class.h"
#include "mock_board.h"
#include "mock_protocol_callback.h"

uint32_t SystemCoreClock = 12000000; // overshadow the variable used to determine core lock

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

//--------------------------------------------------------------------+
// Message Layout
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

//--------------------------------------------------------------------+
// INVALID PARAMETER
//--------------------------------------------------------------------+
void test_invalid_message_type(void)
{
  // Setup the test input (message_cmd)
  message_cmd.msg_type = 0;

  // Add the faulty message to the FIFO
  fifo_write(&ff_prot_cmd, &message_cmd);

  // Setup the expected output for comparison purposes
  protMsgError_t invalid_msg_type_error =
  {
    .msg_type = PROT_MSGTYPE_ERROR,
    .error_id = ERROR_PROT_INVALIDMSGTYPE
  };

  // Ensure that 'prot_cmd_error_cb' was invoked, expecting
  // the results we defined above in 'invalid_msg_type_error'
  prot_cmd_error_cb_Expect(&invalid_msg_type_error);
  
  // Ensure that 'command_send' was invoked (this is a macro that 
  // abstract the actual 'usb_xxx_send' call), stating the results
  // we expect: LPC_OK will be returned, but a protMsgError_t array
  // will be created
  MOCK_PROT(command_send, _ExpectWithArrayAndReturn) (
      (uint8_t *) &invalid_msg_type_error, sizeof(protMsgError_t),
      sizeof(protMsgError_t), LPC_OK);

  //------------- Code Under Test -------------//
  // Process the FIFO contents and run the test code
  prot_task(NULL);
}

void test_invalid_command(void)
{
  message_cmd.msg_type = PROT_MSGTYPE_COMMAND;
  message_cmd.cmd_id   = 0;

  fifo_write(&ff_prot_cmd, &message_cmd);

  protMsgError_t invalid_cmd_error =
  {
    .msg_type = PROT_MSGTYPE_ERROR,
    .error_id = ERROR_PROT_INVALIDCOMMANDID
  };

  prot_cmd_error_cb_Expect(&invalid_cmd_error);
  MOCK_PROT(command_send, _ExpectWithArrayAndReturn) (
      (uint8_t *) &invalid_cmd_error, sizeof(protMsgError_t),
      sizeof(protMsgError_t), LPC_OK);

  //------------- Code Under Test -------------//
  prot_task(NULL);

}

#endif
