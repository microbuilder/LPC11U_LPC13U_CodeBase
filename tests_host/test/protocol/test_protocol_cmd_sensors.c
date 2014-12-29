/**************************************************************************/
/*!
    @file     test_protocol_cmd_sensors.c
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
#include <stdlib.h>
#include "unity.h"
#include "fifo.h"
#include "protocol.h"

#include "protocol_support.h"
#include "protocol_cmd_sensors.h"

#include "mock_usb_hid.h"
#include "mock_usb_custom_class.h"
#include "mock_protocol_callback.h"

#include "sensors.h"
#include "mock_lsm303accel.h"
#include "mock_lsm303mag.h"

static protMsgCommand_t  message_cmd;
static protMsgError_t    message_error;
static protMsgResponse_t message_response;

#define U16_HIGH_U8(u16)  ((uint8_t) (((u16) >> 8) & 0x00FF))
#define U16_LOW_U8(u16)   ((uint8_t) ((u16) & 0x00FF))

void setUp(void)
{
  prot_init();

  message_cmd = (protMsgCommand_t)
  {
    .msg_type = PROT_MSGTYPE_COMMAND,
    .cmd_id   = PROT_CMDTYPE_SENSORS_EVENT,
    .length   = 2
  };
}

void tearDown(void)
{

}

/**************************************************************************/
/*!
    @brief  Simulates the HW call in 'lsm303accelGetSensorEvent'
*/
/**************************************************************************/
err_t stub_lsm303accel_get(sensors_event_t *event, int num_call)
{
  static float32_t _lsm303accel_MG_LSB = 0.001F;

  (*event) = (sensors_event_t)
  {
    .version        = sizeof(sensors_event_t),
    .sensor_id      = 12345,
    .type           = SENSOR_TYPE_ACCELEROMETER,
    //.timestamp      = num_call+1,
    //.acceleration.x = ( (rand()%(2048*2)) - 2048 )  * _lsm303accel_MG_LSB * SENSORS_GRAVITY_STANDARD,
    //.acceleration.y = ( (rand()%(2048*2)) - 2048 )  * _lsm303accel_MG_LSB * SENSORS_GRAVITY_STANDARD,
    //.acceleration.z = ( (rand()%(2048*2)) - 2048 )  * _lsm303accel_MG_LSB * SENSORS_GRAVITY_STANDARD
    .timestamp      = 0,
    .acceleration.x = 1024 * _lsm303accel_MG_LSB * SENSORS_GRAVITY_STANDARD,
    .acceleration.y = -512 * _lsm303accel_MG_LSB * SENSORS_GRAVITY_STANDARD,
    .acceleration.z = -256 * _lsm303accel_MG_LSB * SENSORS_GRAVITY_STANDARD
  };

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Tries to read the LSM303 accelerometer (using simulated HW
            via 'stub_lsm303accel_get' above)
*/
/**************************************************************************/
void test_prot_sensor_accel_lsm303(void)
{
  static float32_t _lsm303accel_MG_LSB = 0.001F;
  uint8_t buffer[sizeof(sensors_event_t)];

  /* Request sensor data from the LSM303 accelerometer */
  message_cmd.payload[0] = U16_LOW_U8(PROT_CMD_SENSORS_KEY_ACCEL_LSM303);
  message_cmd.payload[1] = U16_HIGH_U8(PROT_CMD_SENSORS_KEY_ACCEL_LSM303);

  /* Define the response message that we are expecting */
  message_response = (protMsgResponse_t)
  {
    .msg_type    = PROT_MSGTYPE_RESPONSE,
    .cmd_id      = PROT_CMDTYPE_SENSORS_EVENT,
    .length      = 36
  };

  /* Define and insert the response message payload */
  sensors_event_t event =
  {
    .version        = sizeof(sensors_event_t),
    .sensor_id      = 12345,
    .type           = SENSOR_TYPE_ACCELEROMETER,
    .timestamp      = 0,
    .acceleration.x = 1024 * _lsm303accel_MG_LSB * SENSORS_GRAVITY_STANDARD,
    .acceleration.y = -512 * _lsm303accel_MG_LSB * SENSORS_GRAVITY_STANDARD,
    .acceleration.z = -256 * _lsm303accel_MG_LSB * SENSORS_GRAVITY_STANDARD
  };
  sensorsSerializeSensorsEvent(buffer, &event);
  memcpy(&message_response.payload, buffer, sizeof(sensors_event_t));

  /* Put the command message into the protocol FIFO buffer */
  fifo_write(&ff_prot_cmd, &message_cmd);

  /* Substitute 'lsm303accelGetSensorEvent' with the mocked version */
  lsm303accelGetSensorEvent_StubWithCallback(stub_lsm303accel_get);

  /* Indicate what we're expecting in the prot_cmd_received_cb callback */
  prot_cmd_received_cb_Expect(&message_cmd);

  /* Indicate what we're expecting in the prot_cmd_executed_cb callback */
  prot_cmd_executed_cb_Expect(&message_response);
  // prot_cmd_executed_cb_Ignore();

  MOCK_PROT(command_send, _IgnoreAndReturn)(LPC_OK);

  /* ------------- Code Under Test ------------- */
  prot_exec(NULL);
}
