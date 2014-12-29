/**************************************************************************/
/*!
    @file     adxl345.c
    @author   K. Townsend (microBuilder.eu)
    @ingroup  Sensors

    @brief    Driver for Analog Devices ADXL345 Accelerometer

    @details

    The ADXL345 is a digital accelerometer with 13-bit resolution, capable
    of measuring up to +/-16g.  This driver communicate using I2C.

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
#include "adxl345.h"
#include "core/delay/delay.h"
#include <string.h>

#define ADXL345_MG2G_MULTIPLIER (0.004)  // 4mg per lsb

extern volatile uint8_t   I2CMasterBuffer[I2C_BUFSIZE];
extern volatile uint8_t   I2CSlaveBuffer[I2C_BUFSIZE];
extern volatile uint32_t  I2CReadLength, I2CWriteLength;

static bool    _adxl345Initialised = false;
static int32_t _adxl345SensorID = 0;

/**************************************************************************/
/*!
    @brief  Sends a single command byte over I2C
*/
/**************************************************************************/
static err_t adxl345Write8 (uint8_t reg, uint8_t value)
{
  I2CWriteLength = 3;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = ADXL345_ADDRESS;
  I2CMasterBuffer[1] = reg;
  I2CMasterBuffer[2] = value;
  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/
static err_t adxl345Read8(uint8_t reg, uint8_t *value)
{
  I2CWriteLength = 2;
  I2CReadLength = 1;
  I2CMasterBuffer[0] = ADXL345_ADDRESS;
  I2CMasterBuffer[1] = reg;
  /* Append address w/read bit */
  I2CMasterBuffer[2] = ADXL345_ADDRESS | ADXL345_READBIT;
  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  /* Shift values to create properly formed integer */
  *value = I2CSlaveBuffer[0];

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Initialises the I2C block
*/
/**************************************************************************/
err_t adxl345Init(void)
{
  uint8_t devid = 0x00;

  /* Initialise I2C */
  i2cInit(I2CMASTER);

  /* Ping the I2C device first to see if it exists! */
  ASSERT(i2cCheckAddress(ADXL345_ADDRESS), ERROR_I2C_DEVICENOTFOUND);

  /* Check device ID register to see if everything is properly connected */
  ASSERT_STATUS(adxl345Read8(ADXL345_REG_DEVID, &devid));
  if (devid != 0xE5)
  {
    return ERROR_I2C_DEVICENOTFOUND;
  }

  /* Enable measurements */
  ASSERT_STATUS(adxl345Write8(ADXL345_REG_POWER_CTL, 0x08));

  _adxl345Initialised = true;

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Gets the latest X/Y/Z values
*/
/**************************************************************************/
err_t adxl345GetXYZ(int16_t *x, int16_t *y, int16_t *z)
{
  int32_t i2cState;

  if (!_adxl345Initialised)
  {
    ASSERT_STATUS(adxl345Init());
  }

  I2CWriteLength = 2;
  I2CReadLength = 6;
  I2CMasterBuffer[0] = ADXL345_ADDRESS;
  I2CMasterBuffer[1] = ADXL345_REG_DATAX0;
  I2CMasterBuffer[2] = ADXL345_ADDRESS | ADXL345_READBIT;
  i2cState = i2cEngine();

  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cState);

  /* Shift values to create properly formed integer */
  *x = (I2CSlaveBuffer[1] << 8) | (I2CSlaveBuffer[0]);
  *y = (I2CSlaveBuffer[3] << 8) | (I2CSlaveBuffer[2]);
  *z = (I2CSlaveBuffer[5] << 8) | (I2CSlaveBuffer[4]);

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
err_t adxl345SetRange(adxl345_range_t range)
{
  uint8_t format;

  /* Red the data format register to preserve bits */
  ASSERT_STATUS(adxl345Read8(ADXL345_REG_DATA_FORMAT, &format));

  /* Update the data rate */
  format &= ~0x0F;
  format |= range;

  /* Make sure that the FULL-RES bit is enabled for range scaling */
  format |= 0x08;

  /* Write the register back to the IC */
  ASSERT_STATUS(adxl345Write8(ADXL345_REG_DATA_FORMAT, format));

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
err_t adxl345GetRange(adxl345_range_t *range)
{
  uint8_t results;
  ASSERT_STATUS(adxl345Read8(ADXL345_REG_DATA_FORMAT, &results));
  *range = (adxl345_range_t)(results & 0x03);

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Sets the data rate for the ADXL345 (controls power consumption)
*/
/**************************************************************************/
err_t adxl345SetDataRate(adxl345_dataRate_t dataRate)
{
  /* Note: The LOW_POWER bits are currently ignored and we always keep
     the device in 'normal' mode */
  ASSERT_STATUS(adxl345Write8(ADXL345_REG_BW_RATE, dataRate));

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer
*/
/**************************************************************************/
err_t adxl345GetDataRate(adxl345_dataRate_t *dataRate)
{
  uint8_t results;
  ASSERT_STATUS(adxl345Read8(ADXL345_REG_BW_RATE, &results));
  *dataRate = (adxl345_dataRate_t)(results & 0x0F);

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Provides the sensor_t data for this sensor
*/
/**************************************************************************/
void adxl345GetSensor(sensor_t *sensor)
{
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "ADXL345", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _adxl345SensorID;
  sensor->type        = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay   = 0;
  sensor->max_value   = -156.9064F; /* -16g = 156.9064 m/s^2  */
  sensor->min_value   = 156.9064F;  /*  16g = 156.9064 m/s^2  */
  sensor->resolution  = 0.03923F;   /*  4mg = 0.0392266 m/s^2 */
}

/**************************************************************************/
/*!
    @brief  Reads the sensor and returns the data as a sensors_event_t
*/
/**************************************************************************/
err_t adxl345GetSensorEvent(sensors_event_t *event)
{
  int16_t x, y, z;

  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _adxl345SensorID;
  event->type      = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = delayGetTicks();

  /* Retrieve values from the sensor */
  ASSERT_STATUS(adxl345GetXYZ(&x, &y, &z));

  /* The ADXL345 returns a raw value where each lsb represents 4mg.  To
   * convert this to a normal g value, multiply by 0.004 and then convert
   * it to the m/s^2 value that sensors_event_t is expecting. */
  event->acceleration.x = x * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  event->acceleration.y = y * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  event->acceleration.z = z * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;

  return ERROR_NONE;
}
