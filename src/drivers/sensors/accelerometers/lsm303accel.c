/**************************************************************************/
/*!
    @file     lsm303accel.c
    @author   K. Townsend (microBuilder.eu)
    @ingroup  Sensors

    @brief    Driver for the ST LSM303DLHC I2C accelerometer

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
#include "lsm303accel.h"
#include "core/delay/delay.h"
#include <string.h>

extern volatile uint8_t   I2CMasterBuffer[I2C_BUFSIZE];
extern volatile uint8_t   I2CSlaveBuffer[I2C_BUFSIZE];
extern volatile uint32_t  I2CReadLength, I2CWriteLength;

static bool               _lsm303accelInitialised = false;
static int32_t            _lsm303accelSensorID = 0;
static lsm303AccelData_t  _lsm303accelData;
static float              _lsm303accel_MG_LSB = 0.001F;      // 1, 2, 4 or 12 mg per lsb

/**************************************************************************/
/*!
    @brief  Writes an unsigned 8 bit values over I2C
*/
/**************************************************************************/
err_t lsm303accelWrite8(uint8_t addr, uint8_t reg, uint8_t value)
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
err_t lsm303accelRead8(uint8_t addr, uint8_t reg, uint8_t *value)
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
err_t lsm303accelRead48(uint8_t addr, uint8_t reg, uint8_t buffer[6])
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
err_t lsm303accelInit(void)
{
  // Initialise I2C
  i2cInit(I2CMASTER);

  /* Ping the I2C device first to see if it exists! */
  ASSERT(i2cCheckAddress(LSM303_ADDRESS_ACCEL), ERROR_I2C_DEVICENOTFOUND);

  /* Enable the accelerometer (10Hz) */
  // ASSERT_STATUS(lsm303accelWrite8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x27));
  /* Enable the accelerometer (100Hz) */
  ASSERT_STATUS(lsm303accelWrite8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57));

  _lsm303accelInitialised = true;

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads the raw accelerometer values if you want to skip
            the sensor system abstraction layer and the float overhead
*/
/**************************************************************************/
err_t lsm303accelReadRaw(int16_t *x, int16_t *y, int16_t *z)
{
  uint8_t buffer[6] = { 0, 0, 0, 0, 0, 0 };

  if (!_lsm303accelInitialised)
  {
    ASSERT_STATUS(lsm303accelInit());
  }

  /* Read the accelerometer */
  ASSERT_STATUS(lsm303accelRead48(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_OUT_X_L_A, buffer));

  /* Shift values to create properly formed integer (low byte first) */
  /* Note: Accelerometer is X/Y/Z, low byte then high */
  *x = ((int16_t)(buffer[0] | (buffer[1] << 8))) >> 4;
  *y = ((int16_t)(buffer[2] | (buffer[3] << 8))) >> 4;
  *z = ((int16_t)(buffer[4] | (buffer[5] << 8))) >> 4;

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads the current accelerometer values
*/
/**************************************************************************/
err_t lsm303accelRead(void)
{
  uint8_t buffer[6] = { 0, 0, 0, 0, 0, 0 };

  if (!_lsm303accelInitialised)
  {
    ASSERT_STATUS(lsm303accelInit());
  }

  /* Read the accelerometer */
  ASSERT_STATUS(lsm303accelRead48(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_OUT_X_L_A, buffer));

  /* Shift values to create properly formed integer (low byte first) */
  /* Note: Accelerometer is X/Y/Z, low byte then high */
  _lsm303accelData.x = ((int16_t)(buffer[0] | (buffer[1] << 8))) >> 4;
  _lsm303accelData.y = ((int16_t)(buffer[2] | (buffer[3] << 8))) >> 4;
  _lsm303accelData.z = ((int16_t)(buffer[4] | (buffer[5] << 8))) >> 4;

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Provides the sensor_t data for this sensor
*/
/**************************************************************************/
void lsm303accelGetSensor(sensor_t *sensor)
{
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "LSM303DLHC", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _lsm303accelSensorID;
  sensor->type        = SENSOR_TYPE_ACCELEROMETER;
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
err_t lsm303accelGetSensorEvent(sensors_event_t *event)
{
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _lsm303accelSensorID;
  event->type      = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = delayGetTicks();

  /* Convert units to m/s^2 */
  ASSERT_STATUS(lsm303accelRead());
  event->acceleration.x = _lsm303accelData.x * _lsm303accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
  event->acceleration.y = _lsm303accelData.y * _lsm303accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
  event->acceleration.z = _lsm303accelData.z * _lsm303accel_MG_LSB * SENSORS_GRAVITY_STANDARD;

  return ERROR_NONE;
}
