/**************************************************************************/
/*! 
    @file     mpl115a2.h
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
#ifndef _MPL115A2_H_
#define _MPL115A2_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"
#include "core/i2c/i2c.h"
#include "drivers/sensors/sensors.h"

#define MPL115A2_ADDRESS              (0x60 << 1)    // 1100 000
#define MPL115A2_READBIT              (0x01)

enum
{
  MPL115A2_REGISTER_PRESSURE_MSB     = 0x00,
  MPL115A2_REGISTER_PRESSURE_LSB     = 0x01,
  MPL115A2_REGISTER_TEMP_MSB         = 0x02,
  MPL115A2_REGISTER_TEMP_LSB         = 0x03,
  MPL115A2_REGISTER_A0_COEFF_MSB     = 0x04,
  MPL115A2_REGISTER_A0_COEFF_LSB     = 0x05,
  MPL115A2_REGISTER_B1_COEFF_MSB     = 0x06,
  MPL115A2_REGISTER_B1_COEFF_LSB     = 0x07,
  MPL115A2_REGISTER_B2_COEFF_MSB     = 0x08,
  MPL115A2_REGISTER_B2_COEFF_LSB     = 0x09,
  MPL115A2_REGISTER_C12_COEFF_MSB    = 0x0A,
  MPL115A2_REGISTER_C12_COEFF_LSB    = 0x0B,
  MPL115A2_REGISTER_STARTCONVERSION  = 0x12
};

err_t mpl115a2Init(void);
err_t mpl115a2GetPressure(float *pressure);
void    mpl115a2GetSensor(sensor_t *sensor);
err_t mpl115a2GetSensorEvent(sensors_event_t *event);

#ifdef __cplusplus
}
#endif 

#endif
