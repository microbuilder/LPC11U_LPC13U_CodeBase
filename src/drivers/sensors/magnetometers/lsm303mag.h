/**************************************************************************/
/*!
    @file     lsm303mag.h
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
#ifndef _LSM303DLHC_MAG_H_
#define _LSM303DLHC_MAG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"
#include "core/i2c/i2c.h"
#include "drivers/sensors/sensors.h"

#define LSM303_ADDRESS_MAG        (0x1E<<1) // 0011110

enum {
  LSM303_REGISTER_MAG_CRA_REG_M             = 0x00,
  LSM303_REGISTER_MAG_CRB_REG_M             = 0x01,
  LSM303_REGISTER_MAG_MR_REG_M              = 0x02,
  LSM303_REGISTER_MAG_OUT_X_H_M             = 0x03,
  LSM303_REGISTER_MAG_OUT_X_L_M             = 0x04,
  LSM303_REGISTER_MAG_OUT_Z_H_M             = 0x05,
  LSM303_REGISTER_MAG_OUT_Z_L_M             = 0x06,
  LSM303_REGISTER_MAG_OUT_Y_H_M             = 0x07,
  LSM303_REGISTER_MAG_OUT_Y_L_M             = 0x08,
  LSM303_REGISTER_MAG_SR_REG_Mg             = 0x09,
  LSM303_REGISTER_MAG_IRA_REG_M             = 0x0A,
  LSM303_REGISTER_MAG_IRB_REG_M             = 0x0B,
  LSM303_REGISTER_MAG_IRC_REG_M             = 0x0C,
  LSM303_REGISTER_MAG_TEMP_OUT_H_M          = 0x31,
  LSM303_REGISTER_MAG_TEMP_OUT_L_M          = 0x32
};

typedef enum
{
  LSM303_MAGGAIN_1_3                        = 0x20,  // +/- 1.3
  LSM303_MAGGAIN_1_9                        = 0x40,  // +/- 1.9
  LSM303_MAGGAIN_2_5                        = 0x60,  // +/- 2.5
  LSM303_MAGGAIN_4_0                        = 0x80,  // +/- 4.0
  LSM303_MAGGAIN_4_7                        = 0xA0,  // +/- 4.7
  LSM303_MAGGAIN_5_6                        = 0xC0,  // +/- 5.6
  LSM303_MAGGAIN_8_1                        = 0xE0   // +/- 8.1
} lsm303MagGain_t;

typedef struct lsm303MagData_s
{
  float x;
  float y;
  float z;
} lsm303MagData_t;

err_t  lsm303magInit(void);
err_t  lsm303magReadRaw(int16_t *x, int16_t *y, int16_t *z);
err_t  lsm303magSetGain(lsm303MagGain_t gain);
err_t  lsm303magGetSensorEvent(sensors_event_t *event);
void     lsm303magGetSensor(sensor_t *sensor);

#ifdef __cplusplus
}
#endif

#endif
