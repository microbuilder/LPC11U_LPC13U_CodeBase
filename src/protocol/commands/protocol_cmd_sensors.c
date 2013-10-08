/**************************************************************************/
/*!
    @file     protocol_cmd_sensors.c
    @author   K. Townsend (microBuilder.eu)

    This command can be used to read sensors based on a pre-defined key
    value. For example, sending the command with
    PROT_CMD_SENSORS_KEY_ACCEL_LSM303 (0x0101) for the key will try to
    read the LSM303DLHC accelerometer and return the results.

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

#include <stdio.h>
#include <string.h>

#include "../protocol.h"
#include "protocol_cmd_sensors.h"

#include "drivers/sensors/sensors.h"
#include "drivers/sensors/accelerometers/accelerometers.h"
#include "drivers/sensors/accelerometers/lsm303accel.h"
#include "drivers/sensors/magnetometers/magnetometers.h"
#include "drivers/sensors/magnetometers/lsm303mag.h"

/**************************************************************************/
/*!
    Returns the sensor event results from a read attempt

    PAYLOAD:  2-byte sensor ID key in little-endian format
              (Ex.: PROT_CMD_SENSORS_KEY_MAG_LSM303 = 0x0201 so [01 02])

    RESPONSE: - A serialised sensors_event_t object (36 bytes in length)
              - Potentially a blank response (all zeros) if the sensor
                event wasn't handled in the code, but a valid key way
                passed in.

    EXAMPLE:  Attempt to read the LSM303 magnetometer (ID = 0x0201)
              [10 06 00 02] 01 02

              ERROR_I2C_DEVICENOTFOUND would return:
              [80 01 01]

              A successful read will return:
              [20 06 00 24] ... followed by a 36 byte (0x24 hex) serialised
              'sensors_event_t' object
*/
/**************************************************************************/
error_t protcmd_sensors_getevent(uint8_t length, uint8_t const payload[], protMsgResponse_t* mess_response)
{
  sensors_event_t event;

  uint16_t key = payload[1] << 8 | payload[0];

  /* Make sure we have a valid key */
  ASSERT( key > PROT_CMD_SENSORS_KEY_FIRST, ERROR_INVALIDPARAMETER);
  ASSERT( key < PROT_CMD_SENSORS_KEY_LAST, ERROR_INVALIDPARAMETER);

  /* Clear the sensor event data in case nothing is assigned to it */
  memset(&event, 0, sizeof(sensors_event_t));

  /* ToDo: Move each sensor family into a dedicate handler function */
  switch(key)
  {
  /*=======================================================================
    Accelerometers                                         0x0100 .. 0x01FF
    -----------------------------------------------------------------------*/
    case (PROT_CMD_SENSORS_KEY_ACCEL_DEFAULT):
      break;
    case (PROT_CMD_SENSORS_KEY_ACCEL_LSM303):
      ASSERT_STATUS(lsm303accelGetSensorEvent(&event));
      break;
  /*=======================================================================*/


  /*=======================================================================
    Magnetometers                                          0x0200 .. 0x02FF
    -----------------------------------------------------------------------*/
    case (PROT_CMD_SENSORS_KEY_MAG_DEFAULT):
      break;
    case (PROT_CMD_SENSORS_KEY_MAG_LSM303):
      ASSERT_STATUS(lsm303magGetSensorEvent(&event));
      break;
  /*=======================================================================*/


  /*=======================================================================
    Orientation                                            0x0300 .. 0x03FF
    -----------------------------------------------------------------------*/
    case (PROT_CMD_SENSORS_KEY_ORIENT_DEFAULT):
      break;
  /*=======================================================================*/


  /*=======================================================================
    Gyroscopes                                             0x0400 .. 0x04FF
    -----------------------------------------------------------------------*/
    case (PROT_CMD_SENSORS_KEY_GYRO_DEFAULT):
      break;
  /*=======================================================================*/


  /*=======================================================================
    Light Sensors                                          0x0500 .. 0x05FF
    -----------------------------------------------------------------------*/
    case (PROT_CMD_SENSORS_KEY_LIGHT_DEFAULT):
      break;
  /*=======================================================================*/


  /*=======================================================================
    Pressure Sensors                                       0x0600 .. 0x06FF
    -----------------------------------------------------------------------*/
    case (PROT_CMD_SENSORS_KEY_PRESS_DEFAULT):
      break;
  /*=======================================================================*/


  /*=======================================================================
    Color                                                  0x0700 .. 0x07FF
    -----------------------------------------------------------------------*/
    case (PROT_CMD_SENSORS_KEY_COLOR_DEFAULT):
      break;
  /*=======================================================================*/


  /*=======================================================================
    Proximity                                              0x0800 .. 0x08FF
    -----------------------------------------------------------------------*/
    case (PROT_CMD_SENSORS_KEY_PROX_DEFAULT):
      break;
  /*=======================================================================*/


  /*=======================================================================
    Humidity                                               0x0900 .. 0x09FF
    -----------------------------------------------------------------------*/
    case (PROT_CMD_SENSORS_KEY_HUMIDITY_DEFAULT):
      break;
  /*=======================================================================*/


  /*=======================================================================
    Ambient Temperature                                    0x0A00 .. 0x0AFF
    -----------------------------------------------------------------------*/
    case (PROT_CMD_SENSORS_KEY_AMBTEMP_DEFAULT):
      break;
  /*=======================================================================*/


  /*=======================================================================
    Voltage                                                0x0B00 .. 0x0BFF
    -----------------------------------------------------------------------*/
    case (PROT_CMD_SENSORS_KEY_VOLT_DEFAULT):
      break;
  /*=======================================================================*/


  /*=======================================================================
    Current                                                0x0C00 .. 0x0CFF
    -----------------------------------------------------------------------*/
    case (PROT_CMD_SENSORS_KEY_CURRENT_DEFAULT):
      break;
  /*=======================================================================*/

    default:
      return ERROR_INVALIDPARAMETER;
  }

  /* Serialise sensors_event_t and dump it into the message payload */
  size_t results = sensorsSerializeSensorsEvent(&mess_response->payload[0], &event);
  mess_response->length = results;

  return ERROR_NONE;
}

#endif
