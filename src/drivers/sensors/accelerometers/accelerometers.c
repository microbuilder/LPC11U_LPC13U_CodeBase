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
#include "core/delay/delay.h"
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
void accelGetOrientation(sensors_event_t *event, sensors_vec_t *orientation)
{
  float t_pitch, t_roll;
  float const PI = 3.14159265F;
  float signOfZ = event->acceleration.z > 0 ? 1.0F : -1.0F;
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
}

/**************************************************************************/
/*!
    @brief  Find the absolute minimum or maximum for each stationary position

    @param  pos                      The stationary position (X_MAX/X_MIN...)
    @param  accel_cal_params         The calib parameter placeholder
    @param  (*pGetSensorEvent)       Pointer to the "GetEvent" function of
                                     accelerometer sensor to calibrate

    @note   The calibration is performed at 6 stationary positions

            - X down/up positions: min/max values for 'X-axis'
            - Y down/up positions: min/max values for 'Y-axis'
            - Z down/up positions: min/max values for 'Z-axis'

            The sensor is also rotated slowly about each stationary
            position for error elimination
    @code

    accel_calib_data_t calib_data;
    ...
    // Find the absolute maximum for X-axis
    // Placing the board with X pointing up and slowly rotating about its center
    accelGetPeakRawData(X_MAX, &calib_data, &lsm303accelGetSensorEvent);

    // Find the absolute minimum for X-axis
    // Placing the board with X pointing down and slowly rotating about its center
    accelGetPeakRawData(X_MIN, &calib_data, &lsm303accelGetSensorEvent);

    // Find the absolute maximum for Y-axis
    // Placing the board with Y pointing up and slowly rotating about its center
    accelGetPeakRawData(Y_MAX, &calib_data, &lsm303accelGetSensorEvent);

    // Find the absolute minimum for Y-axis
    // Placing the board with Y pointing down and slowly rotating about its center
    accelGetPeakRawData(Y_MIN, &calib_data, &lsm303accelGetSensorEvent);

    // Find the absolute maximum for Z-axis
    // Placing the board with Z pointing up and slowly rotating about its center
    accelGetPeakRawData(Z_MAX, &calib_data, &lsm303accelGetSensorEvent);

    // Find the absolute minimum for Z-axis
    // Placing the board with Z pointing down and slowly rotating about its center
    accelGetPeakRawData(Z_MIN, &calib_data, &lsm303accelGetSensorEvent);

    @endcode
*/
/**************************************************************************/
error_t accelGetPeakRawData(accel_calib_pos_t pos, accel_calib_data_t *calib_data,
                            error_t (*pGetSensorEvent)(sensors_event_t *))
{
  uint16_t const CALIB_TIME = 10;  /**< in seconds */

  float *peakVal;
  float *accelRawVal;
  sensors_event_t event;
  uint8_t max_min = 0;  /**< Minimum by default */

  /* Set the accelerometer's values according to the position */
  switch (pos)
  {
    case X_MAX:
      accelRawVal = &(event.acceleration.x);
      peakVal = &(calib_data->x.max);
      max_min = 1;
      break;
    case X_MIN:
      accelRawVal = &(event.acceleration.x);
      peakVal = &(calib_data->x.min);
      break;
    case Y_MAX:
      accelRawVal = &(event.acceleration.y);
      peakVal = &(calib_data->y.max);
      max_min = 1;
      break;
    case Y_MIN:
      accelRawVal = &(event.acceleration.y);
      peakVal = &(calib_data->y.min);
      break;
    case Z_MAX:
      accelRawVal = &(event.acceleration.z);
      peakVal = &(calib_data->z.max);
      max_min = 1;
      break;
    case Z_MIN:
      accelRawVal = &(event.acceleration.z);
      peakVal = &(calib_data->z.min);
      break;
    default:
      return ERROR_INVALIDPARAMETER;
  }

  /* Initialise the minimum / maximum for peak value */
  if (max_min == 1)
    *peakVal = -2 * SENSORS_GRAVITY_EARTH; /**< maximum */
  else
    *peakVal = 2 * SENSORS_GRAVITY_EARTH;  /**< minimum */

  /* Be sure to rotate the sensor slowly about its center so that we can eliminate the linearity error  */
  uint32_t startSec = delayGetSecondsActive();
  while (delayGetSecondsActive() < CALIB_TIME + startSec)
  {
    /* Get accelerometer sensor data */
    ASSERT_STATUS(pGetSensorEvent(&event));

    /* Update the maximum / minimum for peak value */
    if (max_min == 1)
    {
      /* maximum */
      if (*accelRawVal > *peakVal) *peakVal = *accelRawVal;
    }
    else
    {
      /* minimum */
      if (*accelRawVal < *peakVal) *peakVal = *accelRawVal;
    }
  }

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Determine the calibration parameters (offset and scale factor)
            for each axis by using the absolute min and max values

    @param  calib_data           The calib parameter placeholder

    @code

    accel_calib_data_t calib_data;
    ...
    accelGetCalParamsForAxis(&calib_data);

    @endcode
*/
/**************************************************************************/
void accelGetCalibParams(accel_calib_data_t *calib_data)
{
  /* Calculate scale factor and offset                                                       */
  /*                                                                                         */
  /*            2 x SENSORS_GRAVITY_EARTH                                 (max + min)        */
  /*   scale = ---------------------------          offset = -scale  x   -------------       */
  /*                   max - min                                               2             */
  /*                                                                                         */
  calib_data->x.scale = 2 * SENSORS_GRAVITY_EARTH / (calib_data->x.max - calib_data->x.min);
  calib_data->x.offset = (-1) * calib_data->x.scale * ((calib_data->x.max + calib_data->x.min) / 2);
  calib_data->y.scale = 2 * SENSORS_GRAVITY_EARTH / (calib_data->y.max - calib_data->y.min);
  calib_data->y.offset = (-1) * calib_data->y.scale * ((calib_data->y.max + calib_data->y.min) / 2);
  calib_data->z.scale = 2 * SENSORS_GRAVITY_EARTH / (calib_data->z.max - calib_data->z.min);
  calib_data->z.offset = (-1) * calib_data->z.scale * ((calib_data->z.max + calib_data->z.min) / 2);
}

/**************************************************************************/
/*!
    @brief  Re-scale the sensor event data with the calibration parameter

            calib_output = sensor_output * scale_factor + offset

    @code

    sensors_event_t event;
    accel_calib_data_t calib_data;
    ...
    accelCalibrateEvent(&event, &calib_data);

    @endcode
*/
/**************************************************************************/
void accelGetCalibEvent(sensors_event_t *event, accel_calib_data_t *calib_data)
{
  event->acceleration.x = event->acceleration.x * calib_data->x.scale + calib_data->x.offset;
  event->acceleration.y = event->acceleration.y * calib_data->y.scale + calib_data->y.offset;
  event->acceleration.z = event->acceleration.z * calib_data->z.scale + calib_data->z.offset;
}
