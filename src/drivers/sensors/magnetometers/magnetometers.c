/**************************************************************************/
/*!
    @file     magnetometers.c
    @author   Nguyen Quang Huy, Nguyen Thien Tin
    @ingroup  Sensors

    @brief    Helper functions for magnetometers

    @code

    error_t error;
    sensors_event_t mag_event;
    sensors_vec_t orientation;

    // Initialise the magnetometer
    error = lsm303magInit();

    // Get the calibration parameters
    mag_calib_para_t mag_calib_para;
    magGetCalibParameter(&mag_calib_para, &lsm303magGetSensorEvent);

    while (1)
    {
      if (!error)
      {
        // Get sensor data
        error = lsm303magGetSensorEvent(&mag_event);
        if (!error)
        {
          // Calibrate the magnetometer with calibration parameters (optional optional but should be invoked for accurate data)
          magCalibration(&mag_event, mag_calib_para);

          // Tilt Compensation (optional)
          sensors_event_t accel_event;
          error = lsm303accelGetSensorEvent(&accel_event);
          if (!error)
          {
            magTiltCompensation(&mag_event, &accel_event);
          }

          // Calculate the right angle (in degree)
          magGetOrientation(&mag_event, &orientation);

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
#include "magnetometers.h"
#include "core/delay/delay.h"
#include <math.h>

/**************************************************************************/
/*!
    @brief  Determine the calibration parameter (offset and scale factor)
            by recording the absolute minimums and maximums for each axis

    @para   mag_calib_para          Parameters used in calibration process
    @para   (*pGetSensorEvent)      Pointer to the "GetEvent" function of
                                    magnetometer sensor
*/
/**************************************************************************/
void magGetCalibParameter(mag_calib_para_t *mag_calib_para, error_t (*pGetSensorEvent)(sensors_event_t *))
{
	uint16_t const CALIB_TIME = 60000;	/**< in miliseconds                                                                 */
	                                    /**< This time should be enough to rotate sensor full 360° for accurate calibration */
  error_t error;
  sensors_event_t event;

  float magMinX, magMaxX;
  float magMinY, magMaxY;

  /* Initialise the maximum and minimum magnetic values for each axis */
  magMaxX = magMaxY = -3.4e38F;   /**< Min float */
  magMinX = magMinY = 3.4e38F;    /**< Max float */

  /* Initialise timer for delay function */
  delayInit();

  /* Calibration process                                            */
  /* Data is collected as the magnetometer sensor is rotated 360°   */
  while (delayGetTicks() < CALIB_TIME)
  {
    /* Get magnetic data */
    error = pGetSensorEvent(&event);

    /* Update the maximum and minimum magnetic values for each axis */
    if (!error)
    {
      if (event.magnetic.x < magMinX) magMinX = event.magnetic.x;
      if (event.magnetic.x > magMaxX) magMaxX = event.magnetic.x;

      if (event.magnetic.y < magMinY) magMinY = event.magnetic.y;
      if (event.magnetic.y > magMaxY) magMaxY = event.magnetic.y;
    }
  }

  /* Calculate scale factor and offset in each axis                                   */

  /* We set scale factor of X-axis to one (normalized)                                */
  /*                                                           (maxX + minX)          */
  /*   scaleX = 1                      offsetX = -scaleX  x   ---------------         */
  /*                                                                 2                */
  /*                                                                                  */
  mag_calib_para->scaleX = 1.0F;
  mag_calib_para->offsetX = (-1) * mag_calib_para->scaleX * ((magMaxX + magMinX) / 2);

  /* Scale factor of Y-axis is calculated accordingly to scale factor of X-axis       */
  /*              maxX - minX                                  (maxY + minY)          */
  /*   scaleY =  -------------         offsetY = -scaleY  x   ---------------         */
  /*              maxY - minY                                        2                */
  /*                                                                                  */
  mag_calib_para->scaleY = (magMaxX - magMinX) / (magMaxY - magMinY);
  mag_calib_para->offsetY = (-1) * mag_calib_para->scaleY * ((magMaxY + magMinY) / 2);
}

/**************************************************************************/
/*!
    @brief  Re-scale the output of the sensor with the calibration parameter
            calib_output = sensor_output * scale_factor + offset
*/
/**************************************************************************/
void magCalibration(sensors_event_t *event, mag_calib_para_t *mag_calib_para)
{
  /* Calculate slope and offset in each axis */
  event->magnetic.x = event->magnetic.x * mag_calib_para->scaleX + mag_calib_para->offsetX;
  event->magnetic.y = event->magnetic.y * mag_calib_para->scaleY + mag_calib_para->offsetY;
}

/**************************************************************************/
/*!
    @brief  Utilize the sensor data from accelerometer to compensate
            the magnetic sensor measurements
*/
/**************************************************************************/
void magTiltCompensation(sensors_event_t *mag_event, sensors_event_t *accel_event)
{
  float t_roll = accel_event->acceleration.x * accel_event->acceleration.x + accel_event->acceleration.z * accel_event->acceleration.z;
  float rollRadians = (float)atan2(accel_event->acceleration.y, sqrt(t_roll));

  float t_pitch = accel_event->acceleration.y * accel_event->acceleration.y + accel_event->acceleration.z * accel_event->acceleration.z;
  float pitchRadians = (float)atan2(accel_event->acceleration.x, sqrt(t_pitch));

  float cosRoll = (float)cos(rollRadians);
  float sinRoll = (float)sin(rollRadians);
  float cosPitch = (float)cos(pitchRadians);
  float sinPitch = (float)sin(pitchRadians);

  /* The tilt compensation algorithm                            */
  /* Xh = X.cosPitch + Z.sinPitch                               */
  /* Yh = X.sinRoll.sinPitch + Y.cosRoll - Z.sinRoll.cosPitch   */
  mag_event->magnetic.x = mag_event->magnetic.x * cosPitch + mag_event->magnetic.z * sinPitch;
  mag_event->magnetic.y = mag_event->magnetic.x * sinRoll * sinPitch + mag_event->magnetic.y * cosRoll - mag_event->magnetic.z * sinRoll * cosPitch;
}
/**************************************************************************/
/*!
    @brief  Populates the .heading fields in the sensors_vec_t
            struct with the right angular data (in degree)
*/
/**************************************************************************/
void magGetOrientation(sensors_event_t *event, sensors_vec_t *orientation)
{
  float const PI = 3.14159265F;

  /* heading (0-359°): Angle between the longitudinal axis (the plane body) and magnetic north, measured clockwise when viewing from the top of the device */
  /* heading = atan(My / Mx) */
  orientation->heading = (float)atan2(event->magnetic.y, event->magnetic.x) * 180 / PI;

  /* Normalize to 0-360 degree */
  if (orientation->heading < 0)
  {
    orientation->heading = 360 + orientation->heading;
  }
}
