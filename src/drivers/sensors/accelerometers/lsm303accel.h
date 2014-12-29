/**************************************************************************/
/*!
    @file     lsm303accel.h
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
#ifndef _LSM303DLHC_ACCEL_H_
#define _LSM303DLHC_ACCEL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"
#include "core/i2c/i2c.h"
#include "drivers/sensors/sensors.h"

#define LSM303_ADDRESS_ACCEL      (0x19<<1) // 0011001

enum
{                                                                                                          // DEFAULT    TYPE
  LSM303_REGISTER_ACCEL_CTRL_REG1_A         = 0x20,   // 00000111   rw
  LSM303_REGISTER_ACCEL_CTRL_REG2_A         = 0x21,   // 00000000   rw
  LSM303_REGISTER_ACCEL_CTRL_REG3_A         = 0x22,   // 00000000   rw
  LSM303_REGISTER_ACCEL_CTRL_REG4_A         = 0x23,   // 00000000   rw
  LSM303_REGISTER_ACCEL_CTRL_REG5_A         = 0x24,   // 00000000   rw
  LSM303_REGISTER_ACCEL_CTRL_REG6_A         = 0x25,   // 00000000   rw
  LSM303_REGISTER_ACCEL_REFERENCE_A         = 0x26,   // 00000000   r
  LSM303_REGISTER_ACCEL_STATUS_REG_A        = 0x27,   // 00000000   r
  LSM303_REGISTER_ACCEL_OUT_X_L_A           = 0x28,
  LSM303_REGISTER_ACCEL_OUT_X_H_A           = 0x29,
  LSM303_REGISTER_ACCEL_OUT_Y_L_A           = 0x2A,
  LSM303_REGISTER_ACCEL_OUT_Y_H_A           = 0x2B,
  LSM303_REGISTER_ACCEL_OUT_Z_L_A           = 0x2C,
  LSM303_REGISTER_ACCEL_OUT_Z_H_A           = 0x2D,
  LSM303_REGISTER_ACCEL_FIFO_CTRL_REG_A     = 0x2E,
  LSM303_REGISTER_ACCEL_FIFO_SRC_REG_A      = 0x2F,
  LSM303_REGISTER_ACCEL_INT1_CFG_A          = 0x30,
  LSM303_REGISTER_ACCEL_INT1_SOURCE_A       = 0x31,
  LSM303_REGISTER_ACCEL_INT1_THS_A          = 0x32,
  LSM303_REGISTER_ACCEL_INT1_DURATION_A     = 0x33,
  LSM303_REGISTER_ACCEL_INT2_CFG_A          = 0x34,
  LSM303_REGISTER_ACCEL_INT2_SOURCE_A       = 0x35,
  LSM303_REGISTER_ACCEL_INT2_THS_A          = 0x36,
  LSM303_REGISTER_ACCEL_INT2_DURATION_A     = 0x37,
  LSM303_REGISTER_ACCEL_CLICK_CFG_A         = 0x38,
  LSM303_REGISTER_ACCEL_CLICK_SRC_A         = 0x39,
  LSM303_REGISTER_ACCEL_CLICK_THS_A         = 0x3A,
  LSM303_REGISTER_ACCEL_TIME_LIMIT_A        = 0x3B,
  LSM303_REGISTER_ACCEL_TIME_LATENCY_A      = 0x3C,
  LSM303_REGISTER_ACCEL_TIME_WINDOW_A       = 0x3D
};

typedef struct lsm303AccelData_s
{
  float x;
  float y;
  float z;
} lsm303AccelData_t;

err_t  lsm303accelInit(void);
err_t  lsm303accelReadRaw(int16_t *x, int16_t *y, int16_t *z);
err_t  lsm303accelGetSensorEvent(sensors_event_t *event);
void     lsm303accelGetSensor(sensor_t *sensor);

#ifdef __cplusplus
}
#endif

#endif
