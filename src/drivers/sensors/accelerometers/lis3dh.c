/**************************************************************************/
/*!
    @file     lis3dh.c
    @author   K. Townsend (microBuilder.eu)
    @ingroup  Sensors

    @brief    Driver for the ST LIS3DH I2C/SPI Accelerometer

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
#include "lis3dh.h"
#include "core/delay/delay.h"
#include <string.h>

#define LIS3DH_SENSITIVITY_2G  (0.001F)
#define LIS3DH_SENSITIVITY_4G  (0.002F)
#define LIS3DH_SENSITIVITY_8G  (0.004F)
#define LIS3DH_SENSITIVITY_16G (0.012F)

extern volatile uint8_t   I2CMasterBuffer[I2C_BUFSIZE];
extern volatile uint8_t   I2CSlaveBuffer[I2C_BUFSIZE];
extern volatile uint32_t  I2CReadLength, I2CWriteLength;

static bool    _lis3dhInitialised = false;
static uint8_t _lis3dhMeasurementRange = 2;        /* +/-2, 4, 8 or 16 g */
static int32_t _lis3dhSensorID = 0;

/**************************************************************************/
/*!
    @brief  Writes an unsigned 8 bit values over I2C
*/
/**************************************************************************/
err_t lis3dhWrite8 (uint8_t reg, uint8_t value)
{
  I2CWriteLength = 3;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = LIS3DH_ADDRESS;
  I2CMasterBuffer[1] = reg;
  I2CMasterBuffer[2] = (value);
  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads an unsigned 8 bit values over I2C
*/
/**************************************************************************/
err_t lis3dhRead8(uint8_t reg, uint8_t *value)
{
  /* Write transaction */
  I2CWriteLength = 2;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = LIS3DH_ADDRESS;
  I2CMasterBuffer[1] = reg;
  i2cEngine();

  /* Read transaction */
  I2CWriteLength = 0;
  I2CReadLength = 1;
  I2CMasterBuffer[0] = LIS3DH_ADDRESS | LIS3DH_READBIT;
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
err_t lis3dhRead48(uint8_t reg, int16_t *x, int16_t *y, int16_t *z)
{
  /* Write transaction */
  I2CWriteLength = 2;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = LIS3DH_ADDRESS;
  I2CMasterBuffer[1] = reg | (0x80);
  i2cEngine();

  /* Read transaction */
  I2CWriteLength = 0;
  I2CReadLength = 6;
  I2CMasterBuffer[0] = LIS3DH_ADDRESS | LIS3DH_READBIT;
  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  /* Shift values to create properly formed integer (low byte first) */
  *x = (int16_t)(I2CSlaveBuffer[1] << 8 | I2CSlaveBuffer[0]);
  *y = (int16_t)(I2CSlaveBuffer[3] << 8 | I2CSlaveBuffer[2]);
  *z = (int16_t)(I2CSlaveBuffer[5] << 8 | I2CSlaveBuffer[4]);

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Initialises the I2C block
*/
/**************************************************************************/
err_t lis3dhInit(void)
{
  /* Initialise I2C */
  i2cInit(I2CMASTER);

  /* Ping the I2C device first to see if it exists! */
  ASSERT(i2cCheckAddress(LIS3DH_ADDRESS), ERROR_I2C_DEVICENOTFOUND);

  /* Set data rate and power mode, and enable X/Y/Z */
  ASSERT_STATUS(lis3dhWrite8(LIS3DH_REGISTER_CTRL_REG1,
    LIS3DH_CTRL_REG1_DATARATE_50HZ |    /* Normal mode, 50Hz */
    LIS3DH_CTRL_REG1_XYZEN));           /* Enable X, Y and Z */

  /* Enable block update and set range to +/-2G */
  ASSERT_STATUS(lis3dhWrite8(LIS3DH_REGISTER_CTRL_REG4,
    LIS3DH_CTRL_REG4_BLOCKDATAUPDATE |  /* Enable block update */
    LIS3DH_CTRL_REG4_SCALE_2G));        /* +/-2G measurement range */

  /* Make sure the measurement range is updated here if you change it */
  _lis3dhMeasurementRange = 2;

  _lis3dhInitialised = true;

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Polls the device for a new X/Y/Z reading
*/
/**************************************************************************/
err_t lis3dhPoll(lis3dhData_t* data)
{
  uint8_t timeout = 0;
  uint8_t buffer = 0x00;

  if (!_lis3dhInitialised)
  {
    ASSERT_STATUS(lis3dhInit());
  }

  /* Check the status register until a new X/Y/Z sample is ready */
  do
  {
    ASSERT_STATUS(lis3dhRead8(LIS3DH_REGISTER_STATUS_REG_AUX, &buffer));
    ASSERT(timeout++ <= LIS3DH_POLL_TIMEOUT, ERROR_OPERATIONTIMEDOUT);
  } while (!(buffer & LIS3DH_STATUS_REG_ZYXDA));

  // /* Check if the data is new or not (data has changed) */
  // if (buffer & LIS3DH_STATUS_REG_ZYXOR)
  // {
  //   /* New data is available */
  // }

  /* For now, always read data even if it hasn't changed */
  ASSERT_STATUS(lis3dhRead48(LIS3DH_REGISTER_OUT_X_L, &(data->x), &(data->y), &(data->z)));

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Provides the sensor_t data for this sensor
*/
/**************************************************************************/
void lis3dhGetSensor(sensor_t *sensor)
{
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "LIS3DH", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _lis3dhSensorID;
  sensor->type        = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay   = 0;

  /* We need to do some calculations to determine resolution and maxRange in m/s^2 */
  sensor->max_value   = _lis3dhMeasurementRange * SENSORS_GRAVITY_STANDARD;
  sensor->min_value   = 0;
  switch (_lis3dhMeasurementRange)
  {
    case 16:
      sensor->resolution  = (_lis3dhMeasurementRange / 32767.0F) * LIS3DH_SENSITIVITY_16G * SENSORS_GRAVITY_STANDARD;
      break;
    case 8:
      sensor->resolution  = (_lis3dhMeasurementRange / 32767.0F) * LIS3DH_SENSITIVITY_8G * SENSORS_GRAVITY_STANDARD;
      break;
    case 4:
      sensor->resolution  = (_lis3dhMeasurementRange / 32767.0F) * LIS3DH_SENSITIVITY_4G * SENSORS_GRAVITY_STANDARD;
      break;
    case 2:
    default:
      sensor->resolution  = (_lis3dhMeasurementRange / 32767.0F) * LIS3DH_SENSITIVITY_2G * SENSORS_GRAVITY_STANDARD;
      break;
  }
}

/**************************************************************************/
/*!
    @brief  Reads the sensor and returns the data as a sensors_event_t
*/
/**************************************************************************/
err_t lis3dhGetSensorEvent(sensors_event_t *event)
{
  lis3dhData_t data;

  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _lis3dhSensorID;
  event->type      = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = delayGetTicks();

  /* Retrieve values from the sensor */
  ASSERT_STATUS(lis3dhPoll(&data));

  /* The LIS3DH returns a raw g value which needs to be adjusted by
   * sensitivity which is shown as mg/digit in the datasheet.  To convert
   * this to a normal g value, multiply by the appropriate sensitivity value
   * and then convert it to the m/s^2 value that sensors_event_t is expecting. */
  switch (_lis3dhMeasurementRange)
  {
    case 16:
      event->acceleration.x = data.x * LIS3DH_SENSITIVITY_16G * SENSORS_GRAVITY_STANDARD;
      event->acceleration.y = data.y * LIS3DH_SENSITIVITY_16G * SENSORS_GRAVITY_STANDARD;
      event->acceleration.z = data.z * LIS3DH_SENSITIVITY_16G * SENSORS_GRAVITY_STANDARD;
      break;
    case 8:
      event->acceleration.x = data.x * LIS3DH_SENSITIVITY_8G * SENSORS_GRAVITY_STANDARD;
      event->acceleration.y = data.y * LIS3DH_SENSITIVITY_8G * SENSORS_GRAVITY_STANDARD;
      event->acceleration.z = data.z * LIS3DH_SENSITIVITY_8G * SENSORS_GRAVITY_STANDARD;
      break;
    case 4:
      event->acceleration.x = data.x * LIS3DH_SENSITIVITY_4G * SENSORS_GRAVITY_STANDARD;
      event->acceleration.y = data.y * LIS3DH_SENSITIVITY_4G * SENSORS_GRAVITY_STANDARD;
      event->acceleration.z = data.z * LIS3DH_SENSITIVITY_4G * SENSORS_GRAVITY_STANDARD;
      break;
    case 2:
    default:
      event->acceleration.x = data.x * LIS3DH_SENSITIVITY_2G * SENSORS_GRAVITY_STANDARD;
      event->acceleration.y = data.y * LIS3DH_SENSITIVITY_2G * SENSORS_GRAVITY_STANDARD;
      event->acceleration.z = data.z * LIS3DH_SENSITIVITY_2G * SENSORS_GRAVITY_STANDARD;
      break;
  }

  return ERROR_NONE;
}
