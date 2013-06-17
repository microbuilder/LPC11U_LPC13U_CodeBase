/**************************************************************************/
/*!
    @file     accelerometers.c
    @author   Nguyen Quang Huy, Nguyen Thien Tin
    @ingroup  Sensors

    @brief    Helper functions for accelerometers

    @code

    error_t error;
    sensors_event_t event;
    sensors_vec_t orientation;

    // Initialise the accelerometer
    error = lsm303accelInit();

    // Get the calibration parameters
    accel_calib_para_t accel_calib_para;
    accelGetCalibParameter(&accel_calib_para);

    while (1)
    {
      if (!error)
      {
        // Get sensor data
        error = lsm303accelGetSensorEvent(&event);
        if (!error)
        {
          // Calibrate the accelerometer with calibration parameters (optional)
          accelCalibration(&event, accel_calib_para);

          // Calculate the right angle (in degree)
          accelGetOrientation(&event, &orientation);

          // Do something with orientation data
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
#include "lsm303accel.h"
#include <math.h>
#include <time.h>

/**************************************************************************/
/*!
    @brief  Populates the .pitch/.roll fields in the sensors_vec_t struct
            with the right angular data (in degree)
*/
/**************************************************************************/
void accelGetOrientation(sensors_event_t *event, sensors_vec_t *orientation)
{
  float t_pitch, t_roll;
  float const PI = 3.14159265;

  /* roll: Rotation around the longitudinal axis (the plane body, 'X axis'). -180<=roll<=180  */
  /* roll is positive and increasing when moving downward                                     */
  /*                                                                                          */
  /*             roll = atan(y / sqrt(x*x + z*z))                                             */
  /* where:  x, y, z are returned value from accelerometer sensor                             */

  t_roll = event->acceleration.x * event->acceleration.x + event->acceleration.z * event->acceleration.z;
  orientation->roll = atan2(event->acceleration.y, sqrt(t_roll)) * 180 / PI;

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
  /*             pitch = atan(x / sqrt(y*y + z*z))                                            */
  /* where:  x, y, z are returned value from accelerometer sensor                             */

  t_pitch = event->acceleration.y * event->acceleration.y + event->acceleration.z * event->acceleration.z;
  orientation->pitch = atan2(event->acceleration.x, sqrt(t_pitch)) * 180 / PI;

  /* scale the angle of pitch in the range [-180, 180] */
  if (event->acceleration.z < 0)
  {
    if (event->acceleration.x > 0)
      orientation->pitch = 180 - orientation->pitch;
    else
      orientation->pitch = -180 - orientation->pitch ;
  }
}

/**************************************************************************/
/*!
    @brief  Determine the calibration parameter (offset and scale factor)
            by recording the absolute minimums and maximums for each axis
*/
/**************************************************************************/
void accelGetCalibParameter(accel_calib_para_t *accel_calib_para)
{
  uint8_t const CALIB_TIME = 60;     /**< in seconds and bigger is better */
  error_t error;
  sensors_event_t event;

  float accelMinX, accelMaxX;
  float accelMinY, accelMaxY;
  float accelMinZ, accelMaxZ;

  /* Initialise the Max, Min accel values for each axis */
  accelMaxX = accelMaxY = accelMaxZ = -2 * SENSORS_GRAVITY_EARTH;
  accelMinX = accelMinY = accelMinZ = 2 * SENSORS_GRAVITY_EARTH;

  time_t start_time, current_time;
  time(&start_time);
  time(&current_time);

  /* Calibration process                                                   */
  /* Slowly rotate the accelerometer sensor multiple times in all 3 axis   */
  /*                                                                       */
  /* Be sure to rotate the sensor slowly about its center so that          */
  /* we can eliminate the linearity error                                  */
  while (difftime(current_time, start_time) < CALIB_TIME)
  {
    /* Get accelerometer data */
    error = lsm303accelGetSensorEvent(&event);

    /* Update the maximum and minimum values from accelerometer for each axis */
    if (!error)
    {
      if (event.acceleration.x < accelMinX) accelMinX = event.acceleration.x;
      if (event.acceleration.x > accelMaxX) accelMaxX = event.acceleration.x;

      if (event.acceleration.y < accelMinY) accelMinY = event.acceleration.y;
      if (event.acceleration.y > accelMaxY) accelMaxY = event.acceleration.y;

      if (event.acceleration.z < accelMinZ) accelMinZ = event.acceleration.z;
      if (event.acceleration.z > accelMaxZ) accelMaxZ = event.acceleration.z;
    }
    time(&current_time);
  }

  /* Calculate scale factor and offset in each axis */
  accel_calib_para->scaleX = 2 * SENSORS_GRAVITY_EARTH / (accelMaxX - accelMinX);
  accel_calib_para->offsetX = -(accelMaxX + accelMinX) / (accelMaxX - accelMinX);

  accel_calib_para->scaleY = 2 * SENSORS_GRAVITY_EARTH / (accelMaxY - accelMinY);
  accel_calib_para->offsetY = -(accelMaxY + accelMinY) / (accelMaxY - accelMinY);

  accel_calib_para->scaleZ = 2 * SENSORS_GRAVITY_EARTH / (accelMaxZ - accelMinZ);
  accel_calib_para->offsetZ = -(accelMaxZ + accelMinZ) / (accelMaxZ - accelMinZ);
}

/**************************************************************************/
/*!
    @brief  Re-scale the output of the sensor with the calibration parameter
*/
/**************************************************************************/
void accelCalibration(sensors_event_t *event, accel_calib_para_t *accel_calib_para)
{
  /* calib_output = sensor_output * scale_factor + offset */
  event->acceleration.x = event->acceleration.x * accel_calib_para->scaleX + accel_calib_para->offsetX;
  event->acceleration.y = event->acceleration.y * accel_calib_para->scaleY + accel_calib_para->offsetY;
  event->acceleration.z = event->acceleration.z * accel_calib_para->scaleZ + accel_calib_para->offsetZ;
}
