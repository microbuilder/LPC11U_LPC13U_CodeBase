/**************************************************************************/
/*!
    @file     pca9685.c
    @author   K. Townsend (microBuilder.eu)

    @brief    Drivers for NXP's PCA9685 12-bit 16-channel PWM Driver

    @section DESCRIPTION

    The PCA9685 is an I2C-bus controlled 16-channel LED controller
    optimized for LCD Red/Green/Blue/Amber (RGBA) color backlighting
    applications. Each LED output has its own 12-bit resolution (4096
    steps) fixed frequency individual PWM controller that operates at a
    programmable frequency from 40 Hz to 1000 Hz with a duty cycle that
    is adjustable from 0 % to 100 % to allow the LED to be set to a
    specific brightness value. All outputs are set to the same PWM
    frequency.

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2012, K. Townsend
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
#include <string.h>
#include "pca9685.h"
#include "core/gpio/gpio.h"
#include "core/delay/delay.h"

extern volatile uint8_t   I2CMasterBuffer[I2C_BUFSIZE];
extern volatile uint8_t   I2CSlaveBuffer[I2C_BUFSIZE];
extern volatile uint32_t  I2CReadLength, I2CWriteLength;

static bool _pca9685Initialised = false;
static uint8_t _pca9685Address = PCA9685_ADDRESS;

/**************************************************************************/
/*!
    @brief  Writes the specified number of bytes over I2C
*/
/**************************************************************************/
err_t pca9685WriteBytes(uint8_t reg, uint8_t *buffer, size_t length)
{
  uint32_t i;

  /* Try to avoid buffer overflow */
  ASSERT(length <= I2C_BUFSIZE - 2, ERROR_BUFFEROVERFLOW);

  /* Fill write buffer */
  for ( i = 2; i < length+2; i++ )
  {
    I2CMasterBuffer[i] = buffer[i-2];
  }

  /* Write transaction */
  I2CWriteLength = 2+length;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = _pca9685Address;
  I2CMasterBuffer[1] = reg;
  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads the specified number of bytes over I2C
*/
/**************************************************************************/
err_t pca9685ReadBytes(uint8_t reg, uint8_t *buffer, size_t length)
{
  uint32_t i;

  /* Try to avoid buffer overflow */
  ASSERT(length <= I2C_BUFSIZE, ERROR_BUFFEROVERFLOW);

  /* Read and write need to be handled in separate transactions or the
     PCA9685 increments the current register one step ahead of where
     we should be. */

  /* Write transaction */
  I2CWriteLength = 2;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = _pca9685Address;
  I2CMasterBuffer[1] = reg;
  i2cEngine();

  /* Read transaction */
  I2CWriteLength = 0;
  I2CReadLength = length;
  I2CMasterBuffer[0] = _pca9685Address | PCA9685_READBIT;
  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  /* Fill the buffer with the I2C response */
  for ( i = 0; i < length; i++ )
  {
    buffer[i] = I2CSlaveBuffer[i];
  }

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C
*/
/**************************************************************************/
err_t pca9685Write8 (uint8_t reg, uint8_t value)
{
  uint8_t buffer = value;
  return pca9685WriteBytes(reg, &buffer, 1);
}

/**************************************************************************/
/*!
    @brief  Reads a single byte over I2C
*/
/**************************************************************************/
err_t pca9685Read8(uint8_t reg, uint8_t *result)
{
  return pca9685ReadBytes(reg, result, 1);
}

/**************************************************************************/
/*!
    @brief  Initialises the I2C block

    @param address  The device I2C address (left-shifted 1 bit)
*/
/**************************************************************************/
err_t pca9685Init(uint8_t address)
{
  // Initialise I2C
  i2cInit(I2CMASTER);

  /* Ping the I2C device first to see if it exists! */
  ASSERT(!(i2cCheckAddress(_pca9685Address)), ERROR_I2C_DEVICENOTFOUND);

  ASSERT_STATUS(pca9685Write8(PCA9685_REG_MODE1, 0x00));

  _pca9685Initialised = true;

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Sets the PWM clock frequency (40-1000Hz)

    @param freqHz  Approximate frequency in Hz (40-1000)
*/
/**************************************************************************/
err_t pca9685SetFrequency(uint16_t freqHz)
{
  uint32_t prescaleValue;
  uint8_t oldMode, newMode;

  ASSERT(_pca9685Initialised, ERROR_DEVICENOTINITIALISED);

  if (freqHz < 40)
  {
    freqHz = 40;
  }

  if (freqHz > 1000)
  {
    freqHz = 1000;
  }

  // prescaleValue = round(25MHz / (4096*updateRate)) - 1
  prescaleValue = 25000000;   // 25 MHz
  prescaleValue /= 4096;      // 12-bit
  prescaleValue /= freqHz;
  prescaleValue -= 1;

  ASSERT_STATUS(pca9685Read8(PCA9685_REG_MODE1, &oldMode));
  newMode = (oldMode & 0x7F) | 0x10;

  // Go to sleep
  ASSERT_STATUS(pca9685Write8(PCA9685_REG_MODE1, newMode));

  // Set prescale
  ASSERT_STATUS(pca9685Write8(PCA9685_REG_PRESCALE, prescaleValue & 0xFF));

  // Wakeup
  ASSERT_STATUS(pca9685Write8(PCA9685_REG_MODE1, oldMode));
  delay(5);
  ASSERT_STATUS(pca9685Write8(PCA9685_REG_MODE1, oldMode | 0x80));

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Sets the PWM output of the specified channel

    @param channel  The channel number [0..15]
    @param on       The 12-bit start point (low to high transition)
    @param off      The 12-bit stop point (high to low transition)
*/
/**************************************************************************/
err_t pca9685SetPWM(uint16_t channel, uint16_t on, uint16_t off)
{
  ASSERT(_pca9685Initialised, ERROR_DEVICENOTINITIALISED);

  if (on > 0xFFF)
  {
    on = 0xFFF;
  }

  if (off < on)
  {
    off = on;
  }

  if (off > 0xFFF)
  {
    off = 0xFFF;
  }

  /* Set the on and off values */
  ASSERT_STATUS(pca9685Write8(PCA9685_REG_LED0_ON_L+4*channel, on & 0xFF));
  ASSERT_STATUS(pca9685Write8(PCA9685_REG_LED0_ON_H+4*channel, on >> 8));
  ASSERT_STATUS(pca9685Write8(PCA9685_REG_LED0_OFF_L+4*channel, off & 0xFF));
  ASSERT_STATUS(pca9685Write8(PCA9685_REG_LED0_OFF_H+4*channel, off >> 8));

  return ERROR_NONE;
}

