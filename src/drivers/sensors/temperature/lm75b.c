/**************************************************************************/
/*!
    @file     lm75b.c
    @author   K. Townsend (microBuilder.eu)
    @ingroup  Sensors

    @brief    Driver for NXP's LM75B temperature sensor

    @details

    Driver for NXP's LM75B I2C temperature sensor.  This temperature
    sensor has an accuracy of 0.125°C, and returns a temperature value
    in degrees celsius where each unit is equal to 0.125°C.  For example,
    if the temperature reading is 198, it means that the temperature in
    degree celsius is: 198 / 8 = 24.75°C.

    @code
    #include "boards/board.h"
    #include "drivers/sensors/temperature/lm75b.h"

    int main(void)
    {
      boardInit();

      int32_t temp = 0;

      // Initialise the LM75B
      lm75bInit();

      while (1)
      {
        // Get the current temperature (in 0.125°C units)
        lm75bGetTemperature(&temp);

        // Multiply value by 125 for fixed-point math (0.125°C per unit)
        temp *= 125;

        // Use modulus operator to display decimal value
        printf("Current Temperature: %d.%d C\n", temp / 1000, temp % 1000);

        // Alternatively, you could also use floating point math, though
        // this will result in larger compiled code if you add in floating
        // point support for printf, etc.
        //
        // float tempFloat = 0.0F;
        // lm75bGetTemperature(&temp);
        // tempFloat = (float)temp / 8.0F;
      }
    }
    @endcode

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, K. Townsend (microBuilder.eu)
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
#include "lm75b.h"
#include "core/delay/delay.h"
#include <string.h>

extern volatile uint8_t   I2CMasterBuffer[I2C_BUFSIZE];
extern volatile uint8_t   I2CSlaveBuffer[I2C_BUFSIZE];
extern volatile uint32_t  I2CReadLength, I2CWriteLength;

static bool    _lm75bInitialised = false;
static int32_t _lm75bSensorID = 0;

/**************************************************************************/
/*!
    @brief  Writes an 8 bit values over I2C
*/
/**************************************************************************/
err_t lm75bWrite8 (uint8_t reg, uint32_t value)
{
  I2CWriteLength = 3;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = LM75B_ADDRESS;             // I2C device address
  I2CMasterBuffer[1] = reg;                       // Command register
  I2CMasterBuffer[2] = (value & 0xFF);            // Value to write
  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/
err_t lm75bRead16(uint8_t reg, int32_t *value)
{
  I2CWriteLength = 2;
  I2CReadLength = 2;
  I2CMasterBuffer[0] = LM75B_ADDRESS;
  I2CMasterBuffer[1] = reg;
  I2CMasterBuffer[2] = LM75B_ADDRESS | LM75B_READBIT;
  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  // Shift values to create properly formed integer
  *value = ((I2CSlaveBuffer[0] << 8) | I2CSlaveBuffer[1]) >> 5;

  //  Sign extend negative numbers
  if (I2CSlaveBuffer[0] & 0x80)
  {
    // Negative number
    *value |= 0xFFFFFC00;
  }

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Writes the supplied 8-bit value to the LM75B config register
*/
/**************************************************************************/
err_t lm75bConfigWrite (uint8_t configValue)
{
  if (!_lm75bInitialised)
  {
    ASSERT_STATUS(lm75bInit());
  }

  ASSERT_STATUS(lm75bWrite8(LM75B_REGISTER_CONFIGURATION, configValue));

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Initialises the I2C block
*/
/**************************************************************************/
err_t lm75bInit(void)
{
  /* Initialise I2C */
  i2cInit(I2CMASTER);

  /* Ping the I2C device first to see if it exists! */
  ASSERT(i2cCheckAddress(LM75B_ADDRESS), ERROR_I2C_DEVICENOTFOUND);

  _lm75bInitialised = true;

  /* Set device to shutdown mode by default (saves power) */
  ASSERT_STATUS(lm75bConfigWrite (LM75B_CONFIG_SHUTDOWN_SHUTDOWN));

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads the current temperature from the LM75B

    @note   This method will assign a signed 32-bit value (int32) to 'temp',
            where each unit represents +/- 0.125°C.  To convert the numeric
            value to degrees celsius, you must divide the value of 'temp'
            by 8.  This conversion is not done automatically, since you may
            or may not want to use floating point math for the calculations.
*/
/**************************************************************************/
err_t lm75bGetTemperature (int32_t *temp)
{
  if (!_lm75bInitialised)
  {
    ASSERT_STATUS(lm75bInit());
  }

  /* Turn device on */
  ASSERT_STATUS(lm75bConfigWrite (LM75B_CONFIG_SHUTDOWN_POWERON));

  /* Read temperature */
  ASSERT_STATUS(lm75bRead16 (LM75B_REGISTER_TEMPERATURE, temp));

  /* Shut device back down */
  ASSERT_STATUS(lm75bConfigWrite (LM75B_CONFIG_SHUTDOWN_SHUTDOWN));

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Provides the sensor_t data for this sensor
*/
/**************************************************************************/
void lm75bGetSensor(sensor_t *sensor)
{
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "LM75B", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version         = 1;
  sensor->sensor_id       = _lm75bSensorID;
  sensor->type            = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  sensor->min_delay       = 0;
  sensor->max_value       = 125.0F;
  sensor->min_value       = -55.0F;
  sensor->resolution      = 0.125F;
}

/**************************************************************************/
/*!
    @brief  Reads the sensor and returns the data as a sensors_event_t
*/
/**************************************************************************/
err_t lm75bGetSensorEvent(sensors_event_t *event)
{
  int32_t temp;

  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _lm75bSensorID;
  event->type      = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  event->timestamp = delayGetTicks();

  /* Retrieve values from the sensor */
  ASSERT_STATUS(lm75bGetTemperature(&temp));

  event->temperature = temp * 0.125F;  /* 0.125 per lsb */

  return ERROR_NONE;
}
