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
    accel_cal_params_list_t accel_cal_params_list;

    // Initialise the accelerometer
    error = lsm303accelInit();

    // Optional: This normally only needs to be done once per sensor!
    // Determine the calibration parameters for this accelerometer, passing
    // in a reference to the sensor's "GetSensorEvent" function to retrieve
    // the sensor data during calibration
    accelGetCalParamsForAxis(SENSOR_AXIS_X, &(accel_cal_params_list.X_axis), &lsm303accelGetSensorEvent);
    accelGetCalParamsForAxis(SENSOR_AXIS_Y, &(accel_cal_params_list.Y_axis), &lsm303accelGetSensorEvent);
    accelGetCalParamsForAxis(SENSOR_AXIS_Z, &(accel_cal_params_list.Z_axis), &lsm303accelGetSensorEvent);

    while (1)
    {
      if (!error)
      {
        // Get sensor data
        error = lsm303accelGetSensorEvent(&event);
        if (!error)
        {
          // Optional: Apply the calibration data to the accelerometer event
          accelCalibrateEvent(&event, &accel_cal_params_list);

          // Optional: Calculate the correct angle/orientation values in
          //           degrees, placing the calculated data in the
          //          .pitch and .roll fields of our sensors_vec_t variable
          accelGetOrientation(&event, &orientation);

          // Do something with orientation data (event.*)
          debug_printf("X: %f, Y: %f, Z: %f, Pitch: %d, Roll: %d\r\n",
            event.acceleration.x,
            event.acceleration.y,
            event.acceleration.z,
            (int)orientation.pitch,
            (int)orientation.roll);
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
    case SENSOR_AXIS_X:
      accel_value = &(event.acceleration.x);
      break;
    case SENSOR_AXIS_Y:
      accel_value = &(event.acceleration.y);
      break;
    case SENSOR_AXIS_Z:
      accel_value = &(event.acceleration.z);
      break;
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
