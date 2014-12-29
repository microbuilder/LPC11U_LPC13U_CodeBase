/**************************************************************************/
/*!
    @file     sensorfusion.c
    @author   Nguyen Quang Huy
    @ingroup  Sensors

    @brief    Helper functions for full 360°/spherical rotation

    @code

    err_t accel_error, mag_error;
    sensors_event_t accel_event, mag_event;
    sensors_vec_t orientation;

    // Initialise the accelerometer and magnetometer
    accel_error = lsm303accelInit();
    mag_error = lsm303magInit();

    while (1)
    {
      if (!accel_error && !mag_error)
      {
        // Get sensor data
        accel_error = lsm303accelGetSensorEvent(&accel_event);
        mag_error = lsm303magGetSensorEvent(&mag_event);
        if (!accel_error && !mag_error)
        {
          // Calculate roll, pitch and heading in degrees, placing the calculated
          // values into the .roll, .pitch, .heading of our sensors_vec_t variable
          lsm303GetOrientation(&accel_event, &mag_event, &orientation);

          // Display the orientation values
          printf("Roll: %f,\t Pitch: %f,\t Heading: %f\r\n",
            orientation.roll, orientation.pitch, orientation.heading);

          // Wait a bit before the next sample
          delay(100);
        }
      }
    }

    @endcode

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2014, K. Townsend
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
#include "sensorfusion.h"
#include <math.h>

/**************************************************************************/
/*!
    @brief  Populates the .roll/.pitch/.heading fields in the sensors_vec_t
            struct with the right angular data (in degree).

            The starting position is set by placing the object flat and
            pointing northwards (Z-axis pointing upward and X-axis pointing
            northwards).

            The orientation of the object can be modeled as resulting from
            3 consecutive rotations in turn: heading (Z-axis), pitch (Y-axis),
            and roll (X-axis) applied to the starting position.


    @param  accel_event   The sensors_event_t variable containing the
                          data from the accelerometer

    @param  mag_event     The sensors_event_t variable containing the
                          data from the magnetometer

    @param  orientation   The sensors_vec_t object that will have it's
                          .roll, .pitch and .heading fields populated
*/
/**************************************************************************/
err_t lsm303GetOrientation(sensors_event_t *accel_event, sensors_event_t *mag_event, sensors_vec_t *orientation)
{
  /* Make sure the input is valid, not null, etc. */
  ASSERT(accel_event != NULL, ERROR_INVALIDPARAMETER);
  ASSERT(mag_event != NULL, ERROR_INVALIDPARAMETER);
  ASSERT(orientation != NULL, ERROR_INVALIDPARAMETER);

  float const PI = 3.14159265F;

  /* roll: Rotation around the X-axis. -180 <= roll <= 180                                          */
  /* a positive roll angle is defined to be a clockwise rotation about the positive X-axis          */
  /*                                                                                                */
  /*                    y                                                                           */
  /*      roll = atan2(---)                                                                         */
  /*                    z                                                                           */
  /*                                                                                                */
  /* where:  y, z are returned value from accelerometer sensor                                      */
  orientation->roll = (float)atan2(accel_event->acceleration.y, accel_event->acceleration.z);

  /* pitch: Rotation around the Y-axis. -180 <= roll <= 180                                         */
  /* a positive pitch angle is defined to be a clockwise rotation about the positive Y-axis         */
  /*                                                                                                */
  /*                                 -x                                                             */
  /*      pitch = atan(-------------------------------)                                             */
  /*                    y * sin(roll) + z * cos(roll)                                               */
  /*                                                                                                */
  /* where:  x, y, z are returned value from accelerometer sensor                                   */
  if (accel_event->acceleration.y * sin(orientation->roll) + accel_event->acceleration.z * cos(orientation->roll) == 0)
    orientation->pitch = accel_event->acceleration.x > 0 ? (PI / 2) : (-PI / 2);
  else
    orientation->pitch = (float)atan(-accel_event->acceleration.x / (accel_event->acceleration.y * sin(orientation->roll) + \
                                                                     accel_event->acceleration.z * cos(orientation->roll)));

  /* heading: Rotation around the Z-axis. -180 <= roll <= 180                                       */
  /* a positive heading angle is defined to be a clockwise rotation about the positive Z-axis       */
  /*                                                                                                */
  /*                                       z * sin(roll) - y * cos(roll)                            */
  /*   heading = atan2(--------------------------------------------------------------------------)  */
  /*                    x * cos(pitch) + y * sin(pitch) * sin(roll) + z * sin(pitch) * cos(roll))   */
  /*                                                                                                */
  /* where:  x, y, z are returned value from magnetometer sensor                                    */
  orientation->heading = (float)atan2(mag_event->magnetic.z * sin(orientation->roll) - mag_event->magnetic.y * cos(orientation->roll), \
                                      mag_event->magnetic.x * cos(orientation->pitch) + \
                                      mag_event->magnetic.y * sin(orientation->pitch) * sin(orientation->roll) + \
                                      mag_event->magnetic.z * sin(orientation->pitch) * cos(orientation->roll));


  /* Convert angular data to degree */
  orientation->roll = orientation->roll * 180 / PI;
  orientation->pitch = orientation->pitch * 180 / PI;
  orientation->heading = orientation->heading * 180 / PI;

  return ERROR_NONE;
}
