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

    while (1)
    {
      if (!error)
      {
        // Get sensor data
        error = lsm303magGetSensorEvent(&mag_event);
        if (!error)
        {
          // Calculate the heading (in degrees)
          magGetOrientation(&mag_event, &orientation);

          // Display the mag and orientation data
          printf("Mag X: %f, Y: %f, Z: %f, Heading: %d\r\n",
            magEvent.magnetic.x,
            magEvent.magnetic.y,
            magEvent.magnetic.z,
            (int)orientation.heading);
        }
        // Wait a bit between sensor updates
        delay(100);
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
    @brief  Determine the calibration parameters (offset and scale factor)
            for given axis by recording the absolute min and max values

    @param  axis                     The given axis (SENSOR_AXIS_X/Y/Z)
    @param  mag_cal_params           The calib parameter placeholder
    @param  (*pGetSensorEvent)       Pointer to the "GetEvent" function of
                                     the magnetometer sensor to calibrate

    @note   Because we do not know exactly the direction of the Earth's
            magnetic field, an amount of 3D rotation needs to be performed
            for determining the maximum and minimum magnetic values for each
            axis.

            The more the sensor is rotated, the more likely absolute peak
            can be captured, but the rotation should be relatively slow
            for best results.

    @code

    mag_cal_params_list_t mag_cal_params_list;

    // Get the calibration parameters for the X axis (slowly rotating
    // the sensor 360° around multiple times with X pointing out towards
    // the horizon)
    magGetCalParamsForAxis(SENSOR_AXIS_X,
      &(mag_cal_params_list->X_axis), &lsm303magGetSensorEvent);

    // Get the calibration parameters for the Y axis (slowly rotating
    // the sensor 360° around multiple times with Y pointing out towards
    // the horizon)
    magGetCalParamsForAxis(SENSOR_AXIS_Y,
      &(mag_cal_params_list->Y_axis), &lsm303magGetSensorEvent);

    // Get the calibration parameters for the Z axis (slowly rotating
    // the sensor 360° around multiple times with Z pointing out towards
    // the horizon)
    magGetCalParamsForAxis(SENSOR_AXIS_Z,
      &(mag_cal_params_list->Z_axis), &lsm303magGetSensorEvent);

    @endcode
*/
/**************************************************************************/
error_t magGetCalParamsForAxis(sensors_axis_t axis,
                               mag_cal_params_t *mag_cal_params,
                               error_t (*pGetSensorEvent)(sensors_event_t *))
{
  /* The calibration time should be enough to rotate sensor a full 360°   */
  uint16_t const CALIB_TIME = 30;         /* Seconds */

  sensors_event_t event;

  float magMin, magMax;

  /* Initialise the maximum and minimum magnetic values */
  magMax = -3.4e38F;   /* Min float */
  magMin = 3.4e38F;    /* Max float */

  /* Calibration process                                            */
  /* Data is collected as the magnetometer sensor is rotated 360°   */

  /* Set the magnetometer's values according to the measured axis   */
  float *mag_value;
  switch (axis)
  {
    case SENSOR_AXIS_X:
      mag_value = &(event.magnetic.x);
      break;
    case SENSOR_AXIS_Y:
      mag_value = &(event.magnetic.y);
      break;
    case SENSOR_AXIS_Z:
      mag_value = &(event.magnetic.z);
      break;
    default:
      mag_value = &(event.magnetic.x);
      break;
  }

  /* Calibration process               */
  /* Perform an amount of 3D rotations */
  uint32_t startSec = delayGetSecondsActive();
  while (delayGetSecondsActive() < CALIB_TIME + startSec)
  {
    /* Get magnetic data */
    ASSERT_STATUS(pGetSensorEvent(&event));

    /* Update the maximum and minimum magnetic values for each axis */
    if (*mag_value < magMin) magMin = *mag_value;
    if (*mag_value > magMax) magMax = *mag_value;
  }

  /* Calculate scale factor and offset                                              */
  /*                                                                                */
  /* We normalized magnetometer's returned values to [-1; 1] range                  */
  /*                2                                         (max + min)           */
  /*   scale = -----------              offset = -scale  x   -------------          */
  /*            max - min                                          2                */
  /*                                                                                */
  mag_cal_params->scale = 2.0F / (magMax - magMin);
  mag_cal_params->offset = (-1) * mag_cal_params->scale * ((magMax + magMin) / 2);

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Re-scale the output of the sensor with the calibration data
            calib_output = sensor_output * scale_factor + offset

    @param  event                 The sensor event with the raw mag data
    @param  mag_cal_params_list   The cal params (calculated using
                                  magGetCalParamsForAxis) to use when
                                  adjusting the values in 'event'

    @code

    // Calibrate the magnetometer with calibration parameters
    // (optional optional but should be invoked for accurate data)
    magCalibrateEvent(&mag_event, mag_cal_params_list);

    @endcode
*/
/**************************************************************************/
void magCalibrateEvent(sensors_event_t *event, mag_cal_params_list_t *mag_cal_params_list)
{
  /* Calculate slope and offset in each axis */
  event->magnetic.x = event->magnetic.x * mag_cal_params_list->X_axis.scale + mag_cal_params_list->X_axis.offset;
  event->magnetic.y = event->magnetic.y * mag_cal_params_list->Y_axis.scale + mag_cal_params_list->Y_axis.offset;
  event->magnetic.z = event->magnetic.z * mag_cal_params_list->Z_axis.scale + mag_cal_params_list->Z_axis.offset;
}

/**************************************************************************/
/*!
    @brief  Utilize the sensor data from an accelerometer to compensate
            the magnetic sensor measurements when the sensor is tilted
            (the pitch and roll angles are not equal 0°)

    @param  mag_event     The raw magnetometer data to adjust for tilt
    @param  accel_event   The accelerometer event data to use to determine
                          the tilt when compensating the mag_event values

    @code

    // Perform tilt compensation with matching accelerometer data
    sensors_event_t accel_event;
    error = lsm303accelGetSensorEvent(&accel_event);
    if (!error)
    {
      magTiltCompensation(&mag_event, &accel_event);
    }

    @endcode
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

            Heading is the angle between the 'X axis' and magnetic north
            on the horizontal plane (Oxy), measured clockwise when viewing
            from the top of the device

    @param  event         The raw magnetometer sensor data to use when
                          calculating out heading
    @param  orientation   The sensors_vec_t object where we will
                          assign an 'orientation.heading' value

    @code

    magGetOrientation(&mag_event, &orientation);

    @endcode
*/
/**************************************************************************/
void magGetOrientation(sensors_event_t *event, sensors_vec_t *orientation)
{
  float const PI = 3.14159265F;

  /* heading = atan(My / Mx)                                                                               */
  orientation->heading = (float)atan2(event->magnetic.y, event->magnetic.x) * 180 / PI;

  /* Normalize to 0-359° */
  if (orientation->heading < 0)
  {
    orientation->heading = 360 + orientation->heading;
  }
}
