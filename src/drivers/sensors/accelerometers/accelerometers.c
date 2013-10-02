/**************************************************************************/
/*!
    @file     accelerometers.c
    @author   Nguyen Quang Huy, Nguyen Thien Tin, K. Townsend
    @ingroup  Sensors

    @brief    Helper functions for accelerometers

    @code

    error_t error;
    sensors_event_t event;
    sensors_vec_t orientation;

    // Initialise the accelerometer
    error = lsm303accelInit();

    while (1)
    {
      if (!error)
      {
        // Get sensor data
        error = lsm303accelGetSensorEvent(&event);
        if (!error)
        {
          // Calculate pitch and roll in degrees, placing the calculated
          // values into the .pitch and .roll of our sensors_vec_t variable
          accelGetOrientation(&event, &orientation);

          // Display the accel + orientation values
          printf("X: %f, Y: %f, Z: %f, Pitch: %d, Roll: %d\r\n",
            event.acceleration.x,
            event.acceleration.y,
            event.acceleration.z,
            (int)orientation.pitch,
            (int)orientation.roll);

          // Wait a bit before the next sample
          delay(100);
        }
      }
    }

    @endcode

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, K. Townsend
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
#include "accelerometers.h"
#include "core/eeprom/eeprom.h"
#include <math.h>

/**************************************************************************/
/*!
    @brief  Populates the .pitch/.roll fields in the sensors_vec_t struct
            with the right angular data (in degree)

    @param  event         The sensors_event_t variable containing the
                          data from the accelerometer
    @param  orientation   The sensors_vec_t object that will have it's
                          .pitch and .roll fields populated

    @code

    error_t error;
    sensors_event_t event;
    sensors_vec_t orientation;
    ...
    error = lsm303accelGetSensorEvent(&event);
    accelGetOrientation(&event, &orientation);

    @endcode
*/
/**************************************************************************/
error_t accelGetOrientation(sensors_event_t *event, sensors_vec_t *orientation)
{
  /* Make sure the input is valid, not null, etc. */
  ASSERT(event != NULL, ERROR_INVALIDPARAMETER);
  ASSERT(orientation != NULL, ERROR_INVALIDPARAMETER);

  float t_pitch, t_roll;
  float const PI = 3.14159265F;
  float signOfZ = event->acceleration.z >= 0 ? 1.0F : -1.0F;
  
  /* roll: Rotation around the longitudinal axis (the plane body, 'X axis'). -90<=roll<=90    */
  /* roll is positive and increasing when moving downward                                     */
  /*                                                                                          */
  /*                                 y                                                        */
  /*             roll = atan(-----------------)                                               */
  /*                          sqrt(x^2 + z^2)                                                 */
  /* where:  x, y, z are returned value from accelerometer sensor                             */
  
  t_roll = event->acceleration.x * event->acceleration.x + event->acceleration.z * event->acceleration.z;
  orientation->roll = (float)atan2(event->acceleration.y, sqrt(t_roll)) * 180 / PI;

  /* pitch: Rotation around the lateral axis (the wing span, 'Y axis'). -180<=pitch<=180)     */
  /* pitch is positive and increasing when moving upwards                                     */
  /*                                                                                          */
  /*                                 x                                                        */
  /*             roll = atan(-----------------)                                               */
  /*                          sqrt(y^2 + z^2)                                                 */
  /* where:  x, y, z are returned value from accelerometer sensor                             */

  t_pitch = event->acceleration.y * event->acceleration.y + event->acceleration.z * event->acceleration.z;
  orientation->pitch = (float)atan2(event->acceleration.x, signOfZ * sqrt(t_pitch)) * 180 / PI;
  
  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Loads the calibration settings from EEPROM, or returns
            ERROR_UNEXPECTEDVALUE if no calibration data was found

    @param  calib_data    The calib parameter placeholder

    @code

    accel_calib_data_t calib_data;
    ...
    if (accelLoadCalData(&calib_data))
    {
      // Do something with the cal data
    }
    else
    {
      printf("No calibration data was found in memory");
    }

    @endcode
*/
/**************************************************************************/
error_t accelLoadCalData(accel_calib_data_t *calib_data)
{
  /* Try to read the accel config data from the EEPROM memory */
  uint16_t accelConfig;
  ASSERT_STATUS(readEEPROM((uint8_t*)CFG_EEPROM_SENSORS_CAL_ACCEL_CONFIG, (uint8_t*)&accelConfig, 2));

  /* Check the config bit first to make sure data is present, and if not return an error! */
  if (accelConfig & SENSORS_CAL_DATA_PRESENT)
  {
    /* If data is present, load calibration data from EEPROM and assign it to calib_data   */
    calib_data->config = accelConfig;
    ASSERT_STATUS(readEEPROM((uint8_t*)CFG_EEPROM_SENSORS_CAL_ACCEL_SENSORID, (uint8_t*)&calib_data->sensorID, 2));
    ASSERT_STATUS(readEEPROM((uint8_t*)CFG_EEPROM_SENSORS_CAL_ACCEL_X_SCALE, (uint8_t*)&calib_data->x.scale, 4));
    ASSERT_STATUS(readEEPROM((uint8_t*)CFG_EEPROM_SENSORS_CAL_ACCEL_X_OFFSET, (uint8_t*)&calib_data->x.offset, 4));
    ASSERT_STATUS(readEEPROM((uint8_t*)CFG_EEPROM_SENSORS_CAL_ACCEL_Y_SCALE, (uint8_t*)&calib_data->y.scale, 4));
    ASSERT_STATUS(readEEPROM((uint8_t*)CFG_EEPROM_SENSORS_CAL_ACCEL_Y_OFFSET, (uint8_t*)&calib_data->y.offset, 4));
    ASSERT_STATUS(readEEPROM((uint8_t*)CFG_EEPROM_SENSORS_CAL_ACCEL_Z_SCALE, (uint8_t*)&calib_data->z.scale, 4));
    ASSERT_STATUS(readEEPROM((uint8_t*)CFG_EEPROM_SENSORS_CAL_ACCEL_Z_OFFSET, (uint8_t*)&calib_data->z.offset, 4));
  }
  else
  {
    return ERROR_UNEXPECTEDVALUE;
  }  
  
  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Re-scale the sensor event data with the calibration parameter
                 calib_output = sensor_output * scale_factor + offset

    @param  event         The raw accelerometer sensor data to use when
                          calculating out pitch and roll

    @param  calib_data    The calib parameter placeholder

    @code

    sensors_event_t event;
    accel_calib_data_t calib_data;
    ...
    accelCalibrateEventData(&event, &calib_data);

    @endcode
*/
/**************************************************************************/
error_t accelCalibrateEventData(sensors_event_t *event, accel_calib_data_t *calib_data)
{
  /* Make sure event and calib_data are valid, not NULL, etc.!*/
  ASSERT(event != NULL, ERROR_INVALIDPARAMETER);
  ASSERT(calib_data != NULL, ERROR_INVALIDPARAMETER);

  event->acceleration.x = event->acceleration.x * calib_data->x.scale + calib_data->x.offset;
  event->acceleration.y = event->acceleration.y * calib_data->y.scale + calib_data->y.offset;
  event->acceleration.z = event->acceleration.z * calib_data->z.scale + calib_data->z.offset;
  
  return ERROR_NONE;
}
