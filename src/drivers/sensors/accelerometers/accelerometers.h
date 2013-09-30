/**************************************************************************/
/*!
    @file     accelerometers.h
    @author   Nguyen Quang Huy, Nguyen Thien Tin
    @ingroup  Sensors

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
#ifndef _ACCELEROMETERS_H_
#define _ACCELEROMETERS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"
#include "drivers/sensors/sensors.h"

typedef enum
{
  X_MAX = (1),
  X_MIN = (2),
  Y_MAX = (3),
  Y_MIN = (4),
  Z_MAX = (5),
  Z_MIN = (6)
} accel_calib_pos_t;

typedef struct
{
  float max;     /**< max raw data */
  float min;     /**< min raw data */
  float scale;   /**< scale factor */
  float offset;  /**< offset error */
} calib_params_t;

typedef struct
{
  calib_params_t x;
  calib_params_t y;
  calib_params_t z;
} accel_calib_data_t;

error_t accelGetPeakRawData(accel_calib_pos_t pos, accel_calib_data_t *calib_data,
                            error_t (*pGetSensorEvent)(sensors_event_t *));
void accelGetCalibParams   (accel_calib_data_t *calib_data);
void accelGetCalibEvent    (sensors_event_t *event, accel_calib_data_t *calib_data);
void accelGetOrientation   (sensors_event_t *event, sensors_vec_t *orientation);

#ifdef __cplusplus
}
#endif

#endif // _ACCELEROMETERS_H_
