/**************************************************************************/
/*!
    @file     bmp085.c
    @author   K. Townsend (microBuilder.eu)
    @ingroup  Sensors

    @brief    Driver for the Bosch BMP085 barometric pressure sensor

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
#include "bmp085.h"
#include "core/delay/delay.h"
#include <string.h>
#include <math.h>

extern volatile uint8_t   I2CMasterBuffer[I2C_BUFSIZE];
extern volatile uint8_t   I2CSlaveBuffer[I2C_BUFSIZE];
extern volatile uint32_t  I2CReadLength, I2CWriteLength;

static bool               _bmp085Initialised = false;
static int32_t            _bmp085SensorID = 0;
static uint8_t            _bmp085Mode = BMP085_MODE_HIGHRES;
static bmp085_calib_data  _bmp085_coeffs;

#define BMP085_USE_DATASHEET_VALS (0) /* Set to 1 for sanity check */

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C
*/
/**************************************************************************/
err_t bmp085WriteCommand(uint8_t reg, uint8_t value)
{
  I2CWriteLength = 3;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = BMP085_ADDRESS;
  I2CMasterBuffer[1] = reg;
  I2CMasterBuffer[2] = value;
  ASSERT_I2C_STATUS(i2cEngine());
  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
err_t bmp085Read8(uint8_t reg, uint8_t *value)
{
  I2CWriteLength = 2;
  I2CReadLength = 1;
  I2CMasterBuffer[0] = BMP085_ADDRESS;
  I2CMasterBuffer[1] = reg;
  I2CMasterBuffer[2] = BMP085_ADDRESS | BMP085_READBIT;
  ASSERT_I2C_STATUS(i2cEngine());

  *value = I2CSlaveBuffer[0];

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit value over I2C
*/
/**************************************************************************/
err_t bmp085Read16(uint8_t reg, uint16_t *value)
{
  I2CWriteLength = 2;
  I2CReadLength = 2;
  I2CMasterBuffer[0] = BMP085_ADDRESS;
  I2CMasterBuffer[1] = reg;
  I2CMasterBuffer[2] = BMP085_ADDRESS | BMP085_READBIT;
  ASSERT_I2C_STATUS(i2cEngine());

  *value = I2CSlaveBuffer[0] << 8 | I2CSlaveBuffer[1];

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads a signed 16 bit value over I2C
*/
/**************************************************************************/
err_t bmp085ReadS16(uint16_t reg, int16_t *value)
{
  uint16_t i;
  ASSERT_STATUS(bmp085Read16(reg, &i));
  *value = (int16_t)i;

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads the factory-set coefficients
*/
/**************************************************************************/
err_t bmp085ReadCoefficients(void)
{
  #if BMP085_USE_DATASHEET_VALS
    _bmp085_coeffs.ac1 = 408;
    _bmp085_coeffs.ac2 = -72;
    _bmp085_coeffs.ac3 = -14383;
    _bmp085_coeffs.ac4 = 32741;
    _bmp085_coeffs.ac5 = 32757;
    _bmp085_coeffs.ac6 = 23153;
    _bmp085_coeffs.b1  = 6190;
    _bmp085_coeffs.b2  = 4;
    _bmp085_coeffs.mb  = -32768;
    _bmp085_coeffs.mc  = -8711;
    _bmp085_coeffs.md  = 2868;
    _bmp085Mode        = 0;
  #else
    ASSERT_STATUS(bmp085ReadS16(BMP085_REGISTER_CAL_AC1, &_bmp085_coeffs.ac1));
    ASSERT_STATUS(bmp085ReadS16(BMP085_REGISTER_CAL_AC2, &_bmp085_coeffs.ac2));
    ASSERT_STATUS(bmp085ReadS16(BMP085_REGISTER_CAL_AC3, &_bmp085_coeffs.ac3));
    ASSERT_STATUS(bmp085Read16(BMP085_REGISTER_CAL_AC4, &_bmp085_coeffs.ac4));
    ASSERT_STATUS(bmp085Read16(BMP085_REGISTER_CAL_AC5, &_bmp085_coeffs.ac5));
    ASSERT_STATUS(bmp085Read16(BMP085_REGISTER_CAL_AC6, &_bmp085_coeffs.ac6));
    ASSERT_STATUS(bmp085ReadS16(BMP085_REGISTER_CAL_B1, &_bmp085_coeffs.b1));
    ASSERT_STATUS(bmp085ReadS16(BMP085_REGISTER_CAL_B2, &_bmp085_coeffs.b2));
    ASSERT_STATUS(bmp085ReadS16(BMP085_REGISTER_CAL_MB, &_bmp085_coeffs.mb));
    ASSERT_STATUS(bmp085ReadS16(BMP085_REGISTER_CAL_MC, &_bmp085_coeffs.mc));
    ASSERT_STATUS(bmp085ReadS16(BMP085_REGISTER_CAL_MD, &_bmp085_coeffs.md));
  #endif

  return ERROR_NONE;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
err_t bmp085ReadRawTemperature(int32_t *temperature)
{
  #if BMP085_USE_DATASHEET_VALS
    *temperature = 27898;
  #else
    uint16_t t;
    ASSERT_STATUS(bmp085WriteCommand(BMP085_REGISTER_CONTROL, BMP085_REGISTER_READTEMPCMD));
    delay(5);
    ASSERT_STATUS(bmp085Read16(BMP085_REGISTER_TEMPDATA, &t));
    *temperature = t;
  #endif

  return ERROR_NONE;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
err_t bmp085ReadRawPressure(int32_t *pressure)
{
  #if BMP085_USE_DATASHEET_VALS
    *pressure = 23843;
  #else
    uint8_t  p8;
    uint16_t p16;
    int32_t  p32;

    ASSERT_STATUS(bmp085WriteCommand(BMP085_REGISTER_CONTROL, BMP085_REGISTER_READPRESSURECMD + (_bmp085Mode << 6)));
    switch(_bmp085Mode)
    {
      case BMP085_MODE_ULTRALOWPOWER:
        delay(5);
        break;
      case BMP085_MODE_STANDARD:
        delay(8);
        break;
      case BMP085_MODE_HIGHRES:
        delay(14);
        break;
      case BMP085_MODE_ULTRAHIGHRES:
      default:
        delay(26);
        break;
    }

    ASSERT_STATUS(bmp085Read16(BMP085_REGISTER_PRESSUREDATA, &p16));
    p32 = (uint32_t)p16 << 8;
    ASSERT_STATUS(bmp085Read8(BMP085_REGISTER_PRESSUREDATA+2, &p8));
    p32 += p8;
    p32 >>= (8 - _bmp085Mode);

    *pressure = p32;
  #endif

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Initialises the I2C block
*/
/**************************************************************************/
err_t bmp085Init(bmp085_mode_t mode)
{
  /* Mode boundary check */
  if ((mode > BMP085_MODE_ULTRAHIGHRES) || (mode < 0))
  {
    mode = BMP085_MODE_ULTRAHIGHRES;
  }

  /* Initialise I2C */
  i2cInit(I2CMASTER);

  /* Ping the I2C device first to see if it exists! */
  ASSERT(i2cCheckAddress(BMP085_ADDRESS), ERROR_I2C_DEVICENOTFOUND);

  /* Make sure we have the right device */
  uint8_t id;
  ASSERT_STATUS(bmp085Read8(BMP085_REGISTER_CHIPID, &id));
  ASSERT(id == 0x55, ERROR_I2C_DEVICENOTFOUND);

  /* Set the mode indicator */
  _bmp085Mode = mode;

  /* Coefficients need to be read once */
  ASSERT_STATUS(bmp085ReadCoefficients());

  _bmp085Initialised = true;

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Gets the compensated pressure level in kPa
*/
/**************************************************************************/
err_t bmp085GetPressure(float *pressure)
{
  int32_t  ut = 0, up = 0, compp = 0;
  int32_t  x1, x2, b5, b6, x3, b3, p;
  uint32_t b4, b7;

  /* Make sure the coefficients have been read, etc. */
  if (!_bmp085Initialised)
  {
    ASSERT_STATUS(bmp085Init(BMP085_MODE_STANDARD));
  }

  /* Get the raw pressure and temperature values */
  ASSERT_STATUS(bmp085ReadRawTemperature(&ut));
  ASSERT_STATUS(bmp085ReadRawPressure(&up));

  /* Temperature compensation */
  x1 = (ut - _bmp085_coeffs.ac6) * _bmp085_coeffs.ac5 >> 15;
  x2 = (_bmp085_coeffs.mc << 11) / (x1 + _bmp085_coeffs.md);
  b5 = x1 + x2;

  /* Pressure compensation */
  b6 = b5 - 4000;
  x1 = (_bmp085_coeffs.b2 * ((b6 * b6) >> 12)) >> 11;
  x2 = (_bmp085_coeffs.ac2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = (((((int32_t) _bmp085_coeffs.ac1) * 4 + x3) << _bmp085Mode) + 2) >> 2;
  x1 = (_bmp085_coeffs.ac3 * b6) >> 13;
  x2 = (_bmp085_coeffs.b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (_bmp085_coeffs.ac4 * (uint32_t) (x3 + 32768)) >> 15;
  b7 = ((uint32_t) (up - b3) * (50000 >> _bmp085Mode));

  if (b7 < 0x80000000)
  {
    p = (b7 << 1) / b4;
  }
  else
  {
    p = (b7 / b4) << 1;
  }

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  compp = p + ((x1 + x2 + 3791) >> 4);

  /* Assign compensated pressure value */
  *pressure = compp;

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads the temperatures in degrees Celsius
*/
/**************************************************************************/
err_t bmp085GetTemperature(float *temp)
{
  int32_t ut, x1, x2, b5;
  float t;

  if (!_bmp085Initialised)
  {
    ASSERT_STATUS(bmp085Init(BMP085_MODE_STANDARD));
  }

  ASSERT_STATUS(bmp085ReadRawTemperature(&ut));

  x1 = (ut - (int32_t)_bmp085_coeffs.ac6) * ((int32_t)_bmp085_coeffs.ac5) / pow(2,15);
  x2 = ((int32_t)_bmp085_coeffs.mc * pow(2,11)) / (x1+(int32_t)_bmp085_coeffs.md);
  b5 = x1 + x2;
  t = (b5+8)/pow(2,4);
  t /= 10;

  *temp = t;

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Provides the sensor_t data for this sensor
*/
/**************************************************************************/
void bmp085GetSensor(sensor_t *sensor)
{
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "BMP085", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _bmp085SensorID;
  sensor->type        = SENSOR_TYPE_PRESSURE;
  sensor->min_delay   = 0;
  sensor->max_value   = 300.0F;               // 300..1100 hPa
  sensor->min_value   = 1100.0F;
  sensor->resolution  = 0.01F;                // Datasheet states 0.01 hPa resolution
}

/**************************************************************************/
/*!
    @brief  Reads the sensor and returns the data as a sensors_event_t
*/
/**************************************************************************/
err_t bmp085GetSensorEvent(sensors_event_t *event)
{
  float pressure_kPa;

  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _bmp085SensorID;
  event->type      = SENSOR_TYPE_PRESSURE;
  event->timestamp = delayGetTicks();
  ASSERT_STATUS(bmp085GetPressure(&pressure_kPa));
  event->pressure = pressure_kPa / 100.0F; /* kPa to hPa */

  return ERROR_NONE;
}
