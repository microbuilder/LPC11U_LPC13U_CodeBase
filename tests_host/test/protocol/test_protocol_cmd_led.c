/**************************************************************************/
/*!
    @file     test_protocol_cmd_led.c
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
static protMsgCommand_t message_cmd;

void setUp(void)
{
  prot_init();
  memset(&message_cmd, 0, sizeof(protMsgCommand_t) );
}

void tearDown(void)
{

}


static void cmd_err_stub(protMsgError_t const * p_mess_err, int num_call)
{
  TEST_ASSERT_NOT_NULL(p_mess_err);
  TEST_ASSERT_EQUAL(PROT_MSGTYPE_ERROR, p_mess_err->msg_type);
//  ERROR_PROT_INVALID_PARAM
}

ErrorCode_t hid_send_stub(uint8_t report_in[], uint32_t length, int num_call)
{

}

//--------------------------------------------------------------------+
// LED COMMAND
//--------------------------------------------------------------------+
void test_cmd_led_invalid_length(void)
{
  message_cmd = (protMsgCommand_t)
  {
    .msg_type    = PROT_MSGTYPE_COMMAND,
    .cmd_id      = PROT_CMDTYPE_LED,
    .length      = 10
  };

  fifo_write(&ff_prot_cmd, &message_cmd);

  prot_cmd_error_cb_StubWithCallback(cmd_err_stub);

  prot_cmd_received_cb_Expect(&message_cmd);
  MOCK_PROT(command_send, _IgnoreAndReturn)(LPC_OK);

  //------------- Code Under Test -------------//
  prot_task(NULL);
}

void test_cmd_led_on(void)
{
  message_cmd = (protMsgCommand_t)
  {
    .msg_type    = PROT_MSGTYPE_COMMAND,
    .cmd_id      = PROT_CMDTYPE_LED,
    .length      = 1,
    .payload[0]  = 1
  };
  fifo_write(&ff_prot_cmd, &message_cmd);

  prot_cmd_received_cb_Ignore();
  boardLED_Expect(CFG_LED_ON);
  prot_cmd_executed_cb_Ignore();
  MOCK_PROT(command_send, _IgnoreAndReturn) (LPC_OK);

  //------------- CUT -------------//
  prot_task(NULL);
}

void test_cmd_led_off(void)
{
  message_cmd = (protMsgCommand_t)
  {
    .msg_type    = PROT_MSGTYPE_COMMAND,
    .cmd_id      = PROT_CMDTYPE_LED,
    .length      = 1,
    .payload[0]  = 0
  };
  fifo_write(&ff_prot_cmd, &message_cmd);

  prot_cmd_received_cb_Ignore();
  boardLED_Expect(CFG_LED_OFF);
  prot_cmd_executed_cb_Ignore();
  MOCK_PROT(command_send, _IgnoreAndReturn) (LPC_OK);

  //------------- CUT -------------//
  prot_task(NULL);
}
