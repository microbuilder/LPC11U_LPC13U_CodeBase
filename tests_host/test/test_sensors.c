/**************************************************************************/
/*!
    @file     test_sensors.c
    @ingroup  Unit Tests

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
#include "sensors.h"

uint8_t buffer_event[sizeof(sensors_event_t)];
uint8_t buffer_sensor[sizeof(sensor_t)];

void setUp(void)
{
  memset(buffer_event, 0, sizeof(sensors_event_t));
  memset(buffer_sensor, 0, sizeof(sensor_t));
}

void tearDown(void)
{
}

void test_sensors_serialiseSensor(void)
{
  /* Setup the sensor to serialise */
  sensor_t sensor =
  {
    .name           = "TEST",
    .version        = sizeof(sensor_t),
    .sensor_id      = 12345,
    .type           = SENSOR_TYPE_ACCELEROMETER,
    .max_value      = 1.0F,
    .min_value      = 0.1F,
    .resolution     = 0.001F,
    .min_delay      = 1
  };

  /* Setup a blank object to compare against */
  sensor_t compare;
  memset(&compare, 0, sizeof(sensor_t));

  /* Serialize sensor into a byte array */
  sensorsSerializeSensor(buffer_sensor, &sensor);

  /* Deserialise the data back into a sensor object */
  sensorsDeserializeSensor(&compare, buffer_sensor);

  /* Compare the two structs */
  TEST_ASSERT_EQUAL_MEMORY(&sensor, &compare, sizeof(sensor_t));
}

void test_sensors_serialiseEvent(void)
{
  /* Setup the event to serialise */
  sensors_event_t event =
  {
    .version        = sizeof(sensors_event_t),
    .sensor_id      = 12345,
    .type           = SENSOR_TYPE_ACCELEROMETER,
    .timestamp      = 0,
    .acceleration.x = 12.34F,
    .acceleration.y = 5.0F,
    .acceleration.z = 7.5F
  };

  /* Setup a blank object to compare against */
  sensors_event_t compare;  
  memset(&compare, 0, sizeof(sensors_event_t));
  
  /* Serialize sensor data into a byte array */
  sensorsSerializeSensorsEvent(buffer_event, &event);
  
  /* Deserialise the data back into an event object */
  sensorsDeserializeSensorsEvent(&compare, buffer_event);

  /* Compare the two structs */
  TEST_ASSERT_EQUAL_MEMORY(&event, &compare, sizeof(sensors_event_t));
}

