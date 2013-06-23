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
    @brief  Determine the calibration parameters (offset and scale factor)
            for given axis by recording the absolute min and max values

    @param  axis                     The given axis (SENSOR_AXIS_X/Y/Z)
    @param  accel_cal_params         The calib parameter placeholder
    @param  (*pGetSensorEvent)       Pointer to the "GetEvent" function of
                                     accelerometer sensor to calibrate

    @note   The calibration is performed at 6 stationary positions, which
            need to be covered over a 30 second calibration period:

            - X down/up positions: max and min values for 'X-axis'
            - Y down/up positions: max and min values for 'Y-axis'
            - Z down/up positions: max and min values for 'Z-axis'

            The sensor is also rotated slowly about each stationary
            position for error elimination
    @code

    accel_cal_params_list_t accel_cal_params_list;

    // Calibrate the X-axis by placing the board with X pointing up
    // and slowly rotating 360° around the X axis, then placing the
    // board with the X pointing down and slowly rotating 360°.
    accelGetCalParamsForAxis(SENSOR_AXIS_X,
      &(accel_cal_params_list.X_axis), &lsm303accelGetSensorEvent);

    // Calibrate the Y-axis by placing the board with Y pointing up
    // and slowly rotating 360° around the Y axis, then placing the
    // board with the Y pointing down and slowly rotating 360°.
    accelGetCalParamsForAxis(SENSOR_AXIS_Y,
      &(accel_cal_params_list.Y_axis), &lsm303accelGetSensorEvent);

    // Calibrate the Z-axis by placing the board with Z pointing up
    // and slowly rotating 360° around the Z axis, then placing the
    // board with the Z pointing down and slowly rotating 360°.
    accelGetCalParamsForAxis(SENSOR_AXIS_Z,
      &(accel_cal_params_list.Z_axis), &lsm303accelGetSensorEvent);

    @endcode
*/
/**************************************************************************/
error_t accelGetCalParamsForAxis(sensors_axis_t axis,
                                 accel_cal_params_t *accel_cal_params,
                                 error_t (*pGetSensorEvent)(sensors_event_t *))
{
  uint16_t const CALIB_TIME = 30;  /**< in seconds */

  sensors_event_t event;
  float accelMin, accelMax;

  /* Initialise the minimum and maximum accelerometer values */
  accelMin = 2 * SENSORS_GRAVITY_EARTH;
  accelMax = -2 * SENSORS_GRAVITY_EARTH;

  /* Calibration process                                             */
  /* Data is collected as the accelerometer sensor is rotated 360°   */

  /* Set the accelerometer's values according to the measured axis   */
  float *accel_value;
  switch (axis)
  {
    case SENSOR_AXIS_Y:
      accel_value = &(event.acceleration.y);
      break;
    case SENSOR_AXIS_Z:
      accel_value = &(event.acceleration.z);
      break;
    case SENSOR_AXIS_X:
    default:
      accel_value = &(event.acceleration.x);
      break;
  }

  /* Calibration process                                                                                */
  /* Be sure to rotate the sensor slowly about its center so that we can eliminate the linearity error  */
  uint32_t startSec = delayGetSecondsActive();
  while (delayGetSecondsActive() < CALIB_TIME + startSec)
  {
    /* Get accelerometer sensor data */
    ASSERT_STATUS(pGetSensorEvent(&event));

    /* Update the maximum and minimum accelerometer values for each axis */
    if (*accel_value < accelMin) accelMin = *accel_value;
    if (*accel_value > accelMax) accelMax = *accel_value;
  }

  /* Calculate scale factor and offset                                                       */
  /*                                                                                         */
  /*            2 x SENSORS_GRAVITY_EARTH                                 (max + min)        */
  /*   scale = ---------------------------          offset = -scale  x   -------------       */
  /*                   max - min                                               2             */
  /*                                                                                         */
  accel_cal_params->scale = 2 * SENSORS_GRAVITY_EARTH / (accelMax - accelMin);
  accel_cal_params->offset = (-1) * accel_cal_params->scale * ((accelMax + accelMin) / 2);

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Re-scale the sensor event data with the calibration parameter

            calib_output = sensor_output * scale_factor + offset

    @code

    // Apply the calibration data to the accelerometer event
    accelCalibrateEvent(&event, &accel_cal_params_list);

    @endcode
*/
/**************************************************************************/
void accelCalibrateEvent(sensors_event_t *event, accel_cal_params_list_t *accel_cal_params_list)
{
  event->acceleration.x = event->acceleration.x * accel_cal_params_list->X_axis.scale + accel_cal_params_list->X_axis.offset;
  event->acceleration.y = event->acceleration.y * accel_cal_params_list->Y_axis.scale + accel_cal_params_list->Y_axis.offset;
  event->acceleration.z = event->acceleration.z * accel_cal_params_list->Z_axis.scale + accel_cal_params_list->Z_axis.offset;
}

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

    // Get some sensor data and store it in our event variable
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
    }

    @endcode
*/
/**************************************************************************/
void accelGetOrientation(sensors_event_t *event, sensors_vec_t *orientation)
{
  float t_pitch, t_roll;
  float const PI = 3.14159265F;

  /* roll: Rotation around the longitudinal axis (the plane body, 'X axis'). -180<=roll<=180  */
  /* roll is positive and increasing when moving downward                                     */
  /*                                                                                          */
  /*                                 y                                                        */
  /*             roll = atan(-----------------)                                               */
  /*                          sqrt(x^2 + z^2)                                                 */
  /* where:  x, y, z are returned value from accelerometer sensor                             */

  t_roll = event->acceleration.x * event->acceleration.x + event->acceleration.z * event->acceleration.z;
  orientation->roll = (float)atan2(event->acceleration.y, sqrt(t_roll)) * 180 / PI;

  /* scale the angle of Roll in the range [-180, 180] */
  if (event->acceleration.z < 0)
  {
    if (event->acceleration.y > 0)
      orientation->roll = 180 - orientation->roll;
    else
      orientation->roll = -180 - orientation->roll ;
  }

  /* pitch: Rotation around the lateral axis (the wing span, 'Y axis'). -180<=pitch<=180)     */
  /* pitch is positive and increasing when moving upwards                                     */
  /*                                                                                          */
  /*                                 x                                                        */
  /*             roll = atan(-----------------)                                               */
  /*                          sqrt(y^2 + z^2)                                                 */
  /* where:  x, y, z are returned value from accelerometer sensor                             */

  t_pitch = event->acceleration.y * event->acceleration.y + event->acceleration.z * event->acceleration.z;
  orientation->pitch = (float)atan2(event->acceleration.x, sqrt(t_pitch)) * 180 / PI;

  /* scale the angle of Pitch in the range [-180, 180] */
  if (event->acceleration.z < 0)
  {
    if (event->acceleration.x > 0)
      orientation->pitch = 180 - orientation->pitch;
    else
      orientation->pitch = -180 - orientation->pitch ;
  }
}
