/**************************************************************************/
/*!
    @file     pca9685.h
    @author   K. Townsend (microBuilder.eu)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2012 K. Townsend
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
#ifndef _PCA9685_H_
#define _PCA9685_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"
#include "core/i2c/i2c.h"

#define PCA9685_ADDRESS           (0x40<<1)    // 1000000
#define PCA9685_READBIT           (0x01)

enum
{
  PCA9685_REG_SUBADR1            = 0x02,
  PCA9685_REG_SUBADR2            = 0x03,
  PCA9685_REG_SUBADR3            = 0x04,
  PCA9685_REG_MODE1              = 0x00,
  PCA9685_REG_PRESCALE           = 0xFE,
  PCA9685_REG_LED0_ON_L          = 0x06,
  PCA9685_REG_LED0_ON_H          = 0x07,
  PCA9685_REG_LED0_OFF_L         = 0x08,
  PCA9685_REG_LED0_OFF_H         = 0x09,
  PCA9685_REG_ALLLED_ON_L        = 0xFA,
  PCA9685_REG_ALLLED_ON_H        = 0xFB,
  PCA9685_REG_ALLLED_OFF_L       = 0xFC,
  PCA9685_REG_ALLLED_OFF_H       = 0xFD
};

err_t pca9685Init (uint8_t address);
err_t pca9685SetFrequency (uint16_t freqHz);
err_t pca9685SetPWM (uint16_t channel, uint16_t on, uint16_t off);

#ifdef __cplusplus
}
#endif 

#endif
