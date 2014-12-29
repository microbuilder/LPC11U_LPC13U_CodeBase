/**************************************************************************/
/*!
    @file     lsm303mag.c
    @author   K. Townsend (microBuilder.eu)
    @ingroup  Sensors

    @brief    Driver for the ST LSM303DLHC I2C magnetometer

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
#include "lsm303mag.h"
#include "core/delay/delay.h"
#include <string.h>

extern volatile uint8_t   I2CMasterBuffer[I2C_BUFSIZE];
extern volatile uint8_t   I2CSlaveBuffer[I2C_BUFSIZE];
extern volatile uint32_t  I2CReadLength, I2CWriteLength;

static bool               _lsm303magInitialised = false;
static int32_t            _lsm303magSensorID = 0;
static lsm303MagData_t    _lsm303magData;
static lsm303MagGain_t    _lsm303magGain = LSM303_MAGGAIN_1_3;
static float              _lsm303mag_Gauss_LSB_XY = 1100.0F; // Varies with gain
static float              _lsm303mag_Gauss_LSB_Z = 980.0F;   // Varies with gain

/**************************************************************************/
/*!
    @brief  Writes an unsigned 8 bit values over I2C
*/
/**************************************************************************/
err_t lsm303magWrite8(uint8_t addr, uint8_t reg, uint8_t value)
{
  I2CWriteLength = 3;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = addr;
  I2CMasterBuffer[1] = reg;
  I2CMasterBuffer[2] = (value);
  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads an unsigned 8 bit value over I2C
*/
/**************************************************************************/
err_t lsm303magRead8(uint8_t addr, uint8_t reg, uint8_t *value)
{
  /* Write transaction */
  I2CWriteLength = 2;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = addr;
  I2CMasterBuffer[1] = reg;
  i2cEngine();

  /* Read transaction */
  I2CWriteLength = 0;
  I2CReadLength = 1;
  I2CMasterBuffer[0] = addr | 0x01;
  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  /* Send received value back to the caller */
  *value = I2CSlaveBuffer[0];

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads three signed 16 bit values over I2C
*/
/**************************************************************************/
err_t lsm303magRead48(uint8_t addr, uint8_t reg, uint8_t buffer[6])
{
  /* Write transaction */
  I2CWriteLength = 2;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = addr;
  I2CMasterBuffer[1] = reg | (0x80);
  i2cEngine();

  /* Read transaction */
  I2CWriteLength = 0;
  I2CReadLength = 6;
  I2CMasterBuffer[0] = addr | 0x01;
  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  uint8_t i;
  for (i = 0; i < 6; i++)
  {
    buffer[i] = I2CSlaveBuffer[i];
  }

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Initialises the I2C block
*/
/**************************************************************************/
err_t lsm303magInit(void)
{
  // Initialise I2C
  i2cInit(I2CMASTER);

  /* Ping the I2C device first to see if it exists! */
  ASSERT(i2cCheckAddress(LSM303_ADDRESS_MAG), ERROR_I2C_DEVICENOTFOUND);

  /* Enable the magnetometer (continuous conversion) */
  ASSERT_STATUS(lsm303magWrite8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_MR_REG_M, 0x00));

  _lsm303magInitialised = true;

  /* Set default gain */
  ASSERT_STATUS(lsm303magSetGain(_lsm303magGain));

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Sets the gain on the magnetometer (controls sensitivity)
*/
/**************************************************************************/
err_t lsm303magSetGain(lsm303MagGain_t gain)
{
  if (!_lsm303magInitialised)
  {
    ASSERT_STATUS(lsm303magInit());
  }

  // Enable the device by setting the control bit to 0x03
  ASSERT_STATUS(lsm303magWrite8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRB_REG_M, (uint8_t)gain));

  _lsm303magGain = gain;

  switch(gain)
  {
    case LSM303_MAGGAIN_1_3:
      _lsm303mag_Gauss_LSB_XY = 1100;
      _lsm303mag_Gauss_LSB_Z = 980;
      break;
    case LSM303_MAGGAIN_1_9:
      _lsm303mag_Gauss_LSB_XY = 855;
      _lsm303mag_Gauss_LSB_Z = 760;
      break;
    case LSM303_MAGGAIN_2_5:
      _lsm303mag_Gauss_LSB_XY = 670;
      _lsm303mag_Gauss_LSB_Z = 600;
      break;
    case LSM303_MAGGAIN_4_0:
      _lsm303mag_Gauss_LSB_XY = 450;
      _lsm303mag_Gauss_LSB_Z = 400;
      break;
    case LSM303_MAGGAIN_4_7:
      _lsm303mag_Gauss_LSB_XY = 400;
      _lsm303mag_Gauss_LSB_Z = 255;
      break;
    case LSM303_MAGGAIN_5_6:
      _lsm303mag_Gauss_LSB_XY = 330;
      _lsm303mag_Gauss_LSB_Z = 295;
      break;
    case LSM303_MAGGAIN_8_1:
      _lsm303mag_Gauss_LSB_XY = 230;
      _lsm303mag_Gauss_LSB_Z = 205;
      break;
  }

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads the current magnetometer values
*/
/**************************************************************************/
err_t  lsm303magReadRaw(int16_t *x, int16_t *y, int16_t *z)
{
  uint8_t buffer[6] = { 0, 0, 0, 0, 0, 0 };

  if (!_lsm303magInitialised)
  {
    ASSERT_STATUS(lsm303magInit());
  }

  /* Read the magnetometer */
  ASSERT_STATUS(lsm303magRead48(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_OUT_X_H_M, buffer));

  /* Shift values to create properly formed integer (low byte first) */
  /* Note: Magnetometer is X/Z/Y, high byte then low */
  *x = (int16_t)(buffer[1] | (buffer[0] << 8));
  *z = (int16_t)(buffer[3] | (buffer[2] << 8));
  *y = (int16_t)(buffer[5] | (buffer[4] << 8));

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads the current magnetometer values
*/
/**************************************************************************/
err_t lsm303magRead(void)
{
  uint8_t buffer[6] = { 0, 0, 0, 0, 0, 0 };

  if (!_lsm303magInitialised)
  {
    ASSERT_STATUS(lsm303magInit());
  }

  /* Read the magnetometer */
  ASSERT_STATUS(lsm303magRead48(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_OUT_X_H_M, buffer));

  /* Shift values to create properly formed integer (low byte first) */
  /* Note: Magnetometer is X/Z/Y, high byte then low */
  _lsm303magData.x = (int16_t)(buffer[1] | (buffer[0] << 8));
  _lsm303magData.z = (int16_t)(buffer[3] | (buffer[2] << 8));
  _lsm303magData.y = (int16_t)(buffer[5] | (buffer[4] << 8));

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Provides the sensor_t data for this sensor
*/
/**************************************************************************/
void lsm303magGetSensor(sensor_t *sensor)
{
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "LSM303DLHC", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _lsm303magSensorID;
  sensor->type        = SENSOR_TYPE_MAGNETIC_FIELD;
  sensor->min_delay   = 0;
  sensor->max_value   = 1.0;                  // TBD
  sensor->min_value   = 1.0;                  // TBD
  sensor->resolution  = 1.0;                  // TBD
}

/**************************************************************************/
/*!
    @brief  Reads the sensor and returns the data as a sensors_event_t
*/
/**************************************************************************/
err_t lsm303magGetSensorEvent(sensors_event_t *event)
{
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _lsm303magSensorID;
  event->type      = SENSOR_TYPE_MAGNETIC_FIELD;
  event->timestamp = delayGetTicks();

  /* Convert units to micro-Tesla (1 Gauss = 1 micro-Tesla) */
  ASSERT_STATUS(lsm303magRead());
  event->magnetic.x = _lsm303magData.x / _lsm303mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
  event->magnetic.y = _lsm303magData.y / _lsm303mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
  event->magnetic.z = _lsm303magData.z / _lsm303mag_Gauss_LSB_Z * SENSORS_GAUSS_TO_MICROTESLA;

  return ERROR_NONE;
}

