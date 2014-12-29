/**************************************************************************/
/*!
    @file     mpl115a2.c
    @author   K. Townsend (microBuilder.eu)
    @ingroup  Sensors

    @brief    Driver for the Freescale MPL115A2 I2C Digital Barometer

    @details

    The MPL115A2 is an I2C absolute pressure sensor, employing a MEMS
    pressure sensor with a conditioning IC to provide accurate pressure
    measurements from 50 to 115 kPa.

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
#include "mpl115a2.h"
#include "core/delay/delay.h"
#include <string.h>

extern volatile uint8_t   I2CMasterBuffer[I2C_BUFSIZE];
extern volatile uint8_t   I2CSlaveBuffer[I2C_BUFSIZE];
extern volatile uint32_t  I2CReadLength, I2CWriteLength;

static float _mpl115a2_a0;
static float _mpl115a2_b1;
static float _mpl115a2_b2;
static float _mpl115a2_c12;

static bool    _mpl115a2Initialised = false;
static int32_t _mpl115a2SensorID = 0;

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
err_t mpl115a2ReadPressureTemp(uint16_t *pressure, uint16_t *temp)
{
  I2CWriteLength = 3;
  I2CReadLength = 1;
  I2CMasterBuffer[0] = MPL115A2_ADDRESS;
  I2CMasterBuffer[1] = MPL115A2_REGISTER_STARTCONVERSION;
  I2CMasterBuffer[2] = 0x00;  // Why is this necessary to get results?
  i2cEngine();

  /* Wait a bit for the conversion to complete (3ms max) */
  delay(4);

  I2CWriteLength = 2;
  I2CReadLength = 4;
  I2CMasterBuffer[0] = MPL115A2_ADDRESS;
  I2CMasterBuffer[1] = MPL115A2_REGISTER_PRESSURE_MSB;
  I2CMasterBuffer[2] = MPL115A2_ADDRESS | MPL115A2_READBIT;
  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  /* Shift values to create properly formed integers */
  *pressure = ((I2CSlaveBuffer[0] << 8) | (I2CSlaveBuffer[1])) >> 6;
  *temp = ((I2CSlaveBuffer[2] << 8) | (I2CSlaveBuffer[3])) >> 6;

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads the factory-set coefficients
*/
/**************************************************************************/
err_t mpl115a2ReadCoefficients(void)
{
  int16_t a0coeff;
  int16_t b1coeff;
  int16_t b2coeff;
  int16_t c12coeff;

  I2CWriteLength = 2;
  I2CReadLength = 8;
  I2CMasterBuffer[0] = MPL115A2_ADDRESS;
  I2CMasterBuffer[1] = MPL115A2_REGISTER_A0_COEFF_MSB;
  I2CMasterBuffer[2] = MPL115A2_ADDRESS | MPL115A2_READBIT;
  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  a0coeff = (I2CSlaveBuffer[0] << 8 ) | I2CSlaveBuffer[1];
  b1coeff = (I2CSlaveBuffer[2] << 8 ) | I2CSlaveBuffer[3];
  b2coeff = (I2CSlaveBuffer[4] << 8 ) | I2CSlaveBuffer[5];
  c12coeff = ((I2CSlaveBuffer[6] << 8 ) | I2CSlaveBuffer[7]) >> 2;

  _mpl115a2_a0 = (float)a0coeff / 8;
  _mpl115a2_b1 = (float)b1coeff / 8192;
  _mpl115a2_b2 = (float)b2coeff / 16384;
  _mpl115a2_c12 = (float)c12coeff / 4194304;

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Initialises the I2C block
*/
/**************************************************************************/
err_t mpl115a2Init(void)
{
  /* Initialise I2C */
  i2cInit(I2CMASTER);

  /* Ping the I2C device first to see if it exists! */
  ASSERT(i2cCheckAddress(MPL115A2_ADDRESS), ERROR_I2C_DEVICENOTFOUND);

  /* Coefficients need to be read once */
  ASSERT_STATUS(mpl115a2ReadCoefficients());

  _mpl115a2Initialised = true;

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Gets the compensated pressure level in kPa
*/
/**************************************************************************/
err_t mpl115a2GetPressure(float *pressure)
{
  uint16_t  Padc, Tadc;
  float     Pcomp;

  /* Make sure the coefficients have been read, etc. */
  if (!_mpl115a2Initialised)
  {
    ASSERT_STATUS(mpl115a2Init());
  }

  /* Get raw pressure and temperature settings */
  ASSERT_STATUS(mpl115a2ReadPressureTemp(&Padc, &Tadc));

  /* See datasheet p.6 for evaluation sequence */
  Pcomp = _mpl115a2_a0 + (_mpl115a2_b1 + _mpl115a2_c12 * Tadc ) * Padc + _mpl115a2_b2 * Tadc;

  /* Return pressure as floating point value */
  *pressure = ((65.0F / 1023.0F)*(float)Pcomp) + 50;

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Provides the sensor_t data for this sensor
*/
/**************************************************************************/
void mpl115a2GetSensor(sensor_t *sensor)
{
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "MLP115A2", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _mpl115a2SensorID;
  sensor->type        = SENSOR_TYPE_PRESSURE;
  sensor->min_delay   = 0;
  sensor->max_value   = 1150.0F;    //  115 kPa = 1150 hPa
  sensor->min_value   = 500.0F;     //   50 kPa =  500 hPa
  sensor->resolution  = 1.5F;       // 0.15 kPa
}

/**************************************************************************/
/*!
    @brief  Reads the sensor and returns the data as a sensors_event_t
*/
/**************************************************************************/
err_t mpl115a2GetSensorEvent(sensors_event_t *event)
{
  float pressure_kPa;

  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _mpl115a2SensorID;
  event->type      = SENSOR_TYPE_PRESSURE;
  event->timestamp = delayGetTicks();

  /* Retrieve values from the sensor */
  ASSERT_STATUS(mpl115a2GetPressure(&pressure_kPa));

  /* The MPL115A2 returns a value from 50..115 kPa using a 10-bit range.
   * To convert this to the hPa value sensor_event_t is expecitng simply
   * multiply by 10. */
  event->pressure = pressure_kPa * 10;

  return ERROR_NONE;
}
