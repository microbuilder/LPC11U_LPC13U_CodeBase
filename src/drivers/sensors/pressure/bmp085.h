/**************************************************************************/
/*!
    @file     bmp085.h
    @author   K. Townsend (microBuilder.eu)
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
#ifndef _BMP085_H_
#define _BMP085_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"
#include "core/i2c/i2c.h"
#include "drivers/sensors/sensors.h"

#define BMP085_ADDRESS                (0x77 << 1)
#define BMP085_READBIT                (0x01)

typedef enum
{
  BMP085_MODE_ULTRALOWPOWER          = 0,
  BMP085_MODE_STANDARD               = 1,
  BMP085_MODE_HIGHRES                = 2,
  BMP085_MODE_ULTRAHIGHRES           = 3
} bmp085_mode_t;

enum
{
  BMP085_REGISTER_CAL_AC1            = 0xAA,  // R   Calibration data (16 bits)
  BMP085_REGISTER_CAL_AC2            = 0xAC,  // R   Calibration data (16 bits)
  BMP085_REGISTER_CAL_AC3            = 0xAE,  // R   Calibration data (16 bits)
  BMP085_REGISTER_CAL_AC4            = 0xB0,  // R   Calibration data (16 bits)
  BMP085_REGISTER_CAL_AC5            = 0xB2,  // R   Calibration data (16 bits)
  BMP085_REGISTER_CAL_AC6            = 0xB4,  // R   Calibration data (16 bits)
  BMP085_REGISTER_CAL_B1             = 0xB6,  // R   Calibration data (16 bits)
  BMP085_REGISTER_CAL_B2             = 0xB8,  // R   Calibration data (16 bits)
  BMP085_REGISTER_CAL_MB             = 0xBA,  // R   Calibration data (16 bits)
  BMP085_REGISTER_CAL_MC             = 0xBC,  // R   Calibration data (16 bits)
  BMP085_REGISTER_CAL_MD             = 0xBE,  // R   Calibration data (16 bits)
  BMP085_REGISTER_CHIPID             = 0xD0,
  BMP085_REGISTER_VERSION            = 0xD1,
  BMP085_REGISTER_SOFTRESET          = 0xE0,
  BMP085_REGISTER_CONTROL            = 0xF4,
  BMP085_REGISTER_TEMPDATA           = 0xF6,
  BMP085_REGISTER_PRESSUREDATA       = 0xF6,
  BMP085_REGISTER_READTEMPCMD        = 0x2E,
  BMP085_REGISTER_READPRESSURECMD    = 0x34
};

typedef struct
{
  int16_t  ac1;
  int16_t  ac2;
  int16_t  ac3;
  uint16_t ac4;
  uint16_t ac5;
  uint16_t ac6;
  int16_t  b1;
  int16_t  b2;
  int16_t  mb;
  int16_t  mc;
  int16_t  md;
} bmp085_calib_data;

err_t bmp085Init(bmp085_mode_t mode);
err_t bmp085GetTemperature(float *temp);
err_t bmp085GetPressure(float *pressure);
void    bmp085GetSensor(sensor_t *sensor);
err_t bmp085GetSensorEvent(sensors_event_t *event);

#ifdef __cplusplus
}
#endif 

#endif
