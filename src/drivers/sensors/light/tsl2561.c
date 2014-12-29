/**************************************************************************/
/*!
    @file     tsl2561.c
    @author   K. Townsend (microBuilder.eu)
    @ingroup  Sensors

    @brief    Driver for the TAOS TSL2561 I2C digital luminosity sensor

    @details

    The TSL2561 is a 16-bit digital luminosity sensor the approximates
    the human eye's response to light.  It contains one broadband
    photodiode that measures visible plus infrared light (channel 0)
    and one infrared photodiode (channel 1).

    @code
    #include "drivers/sensors/tsl2561/tsl2561.h"
    ...
    uint16_t broadband, ir;
    uint32_t lux;

    // Initialise luminosity sensor
    if (tsl2561Init())
    {
      printf("No response on the I2C bus from the TSL2561 ... check address/connections%s", CFG_PRINTF_NEWLINE);
    }
    else
    {
      // Optional ... default setting is 13ms with no gain
      // Set timing to 101ms with no gain
      tsl2561SetGain(TSL2561_GAIN_1X);
      tsl2561SetIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);

      // Check luminosity level and calculate lux
      tsl2561GetLuminosity(&broadband, &ir);
      lux = tsl2561CalculateLux(broadband, ir);
      printf("Broadband: %u, IR: %u, Lux: %d %s", broadband, ir, lux, CFG_PRINTF_NEWLINE);
    }

    @endcode

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2012, K. Townsend (microBuilder.eu)
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
#include "tsl2561.h"
#include "core/delay/delay.h"
#include <string.h>

extern volatile uint8_t   I2CMasterBuffer[I2C_BUFSIZE];
extern volatile uint8_t   I2CSlaveBuffer[I2C_BUFSIZE];
extern volatile uint32_t  I2CReadLength, I2CWriteLength;

static bool                     _tsl2561Initialised = false;
static bool                     _tsl2561AutoGain = false;
static tsl2561IntegrationTime_t _tsl2561IntegrationTime = TSL2561_INTEGRATIONTIME_13MS;
static tsl2561Gain_t            _tsl2561Gain = TSL2561_GAIN_1X;
static int32_t                  _tsl2561SensorID = 0;

/**************************************************************************/
/*!
    @brief  Sends a single command byte over I2C
*/
/**************************************************************************/
err_t tsl2561WriteCmd (uint8_t cmd)
{
  I2CWriteLength = 2;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = TSL2561_ADDRESS;
  I2CMasterBuffer[1] = cmd;
  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Writes an 8 bit values over I2C
*/
/**************************************************************************/
err_t tsl2561Write8 (uint8_t reg, uint32_t value)
{
  I2CWriteLength = 3;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = TSL2561_ADDRESS;
  I2CMasterBuffer[1] = reg;
  I2CMasterBuffer[2] = (value & 0xFF);
  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/
err_t tsl2561Read16(uint8_t reg, uint16_t *value)
{
  /* Write transaction */
  I2CWriteLength = 2;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = TSL2561_ADDRESS;
  I2CMasterBuffer[1] = reg;
  i2cEngine();

  /* Read transaction */
  I2CWriteLength = 0;
  I2CReadLength = 2;
  I2CMasterBuffer[0] = TSL2561_ADDRESS | TSL2561_READBIT;
  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  /* Shift values to create properly formed integer (low byte first) */
  *value = (I2CSlaveBuffer[0] | (I2CSlaveBuffer[1] << 8));

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Enables the device
*/
/**************************************************************************/
err_t tsl2561Enable(void)
{
  /* Enable the device by setting the control bit to 0x03 */
  return tsl2561Write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWERON);
}

/**************************************************************************/
/*!
    @brief  Disables the device (putting it in lower power sleep mode)
*/
/**************************************************************************/
err_t tsl2561Disable(void)
{
  /* Turn the device off to save power */
  return tsl2561Write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWEROFF);
}

/**************************************************************************/
/*!
    @brief  Private function to read luminosity on both channels
*/
/**************************************************************************/
err_t tsl2561GetData (uint16_t *broadband, uint16_t *ir)
{
  /* Enable the device by setting the control bit to 0x03 */
  ASSERT_STATUS(tsl2561Enable());

  /* Wait x ms for ADC to complete */
  switch (_tsl2561IntegrationTime)
  {
    case TSL2561_INTEGRATIONTIME_13MS:
      delay(14);
      break;
    case TSL2561_INTEGRATIONTIME_101MS:
      delay(102);
      break;
    default:
      delay(403);
      break;
  }

  /* Reads a two byte value from channel 0 (visible + infrared) */
  ASSERT_STATUS(tsl2561Read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW, broadband));

  /* Reads a two byte value from channel 1 (infrared) */
  ASSERT_STATUS(tsl2561Read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW, ir));

  /* Turn the device off to save power */
  ASSERT_STATUS(tsl2561Disable());

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Initialises the I2C block
*/
/**************************************************************************/
err_t tsl2561Init(void)
{
  /* Initialise I2C */
  i2cInit(I2CMASTER);

  /* Ping the I2C device first to see if it exists! */
  ASSERT(i2cCheckAddress(TSL2561_ADDRESS), ERROR_I2C_DEVICENOTFOUND);

  /* Note: by default, the device is in power down mode on bootup */
  _tsl2561Initialised = true;

  /* Set default integration time and gain */
  ASSERT_STATUS(tsl2561SetIntegrationTime(_tsl2561IntegrationTime));
  ASSERT_STATUS(tsl2561SetGain(_tsl2561Gain));

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Enables or disables the auto-gain settings when reading
            data from the sensor
*/
/**************************************************************************/
void tsl2561EnableAutoGain(bool enable)
{
   _tsl2561AutoGain = enable ? true : false;
}

/**************************************************************************/
/*!
    @brief  Sets the integration time for the sensor.
*/
/**************************************************************************/
err_t tsl2561SetIntegrationTime(tsl2561IntegrationTime_t time)
{
  if (!_tsl2561Initialised)
  {
    ASSERT_STATUS(tsl2561Init());
  }

  /* Enable the device by setting the control bit to 0x03 */
  ASSERT_STATUS(tsl2561Enable());

  /* Update the timing register */
  ASSERT_STATUS(tsl2561Write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING, time | _tsl2561Gain));

  /* Update value placeholders */
  _tsl2561IntegrationTime = time;

  /* Turn the device off to save power */
  ASSERT_STATUS(tsl2561Disable());

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Sets the integration time for the sensor.
*/
/**************************************************************************/
err_t tsl2561SetGain(tsl2561Gain_t gain)
{
  if (!_tsl2561Initialised)
  {
    ASSERT_STATUS(tsl2561Init());
  }

  /* Enable the device by setting the control bit to 0x03 */
  ASSERT_STATUS(tsl2561Enable());

  /* Update the timing register */
  ASSERT_STATUS(tsl2561Write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING, _tsl2561IntegrationTime | gain));

  /* Update value placeholders */
  _tsl2561Gain = gain;

  /* Turn the device off to save power */
  ASSERT_STATUS(tsl2561Disable());

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Gets the broadband (mixed lighting) and IR only values from
            the TSL2561, adjusting gain if auto-gain is enabled
*/
/**************************************************************************/
err_t tsl2561GetLuminosity (uint16_t *broadband, uint16_t *ir)
{
  bool valid = false;

  if (!_tsl2561Initialised)
  {
    ASSERT_STATUS(tsl2561Init());
  }

  /* If Auto gain disabled get a single reading and continue */
  if(!_tsl2561AutoGain)
  {
    return tsl2561GetData (broadband, ir);
  }

  /* Read data until we find a valid range */
  bool _agcCheck = false;
  do
  {
    uint16_t _b, _ir;
    uint16_t _hi, _lo;
    tsl2561IntegrationTime_t _it = _tsl2561IntegrationTime;

    /* Get the hi/low threshold for the current integration time */
    switch(_it)
    {
      case TSL2561_INTEGRATIONTIME_13MS:
        _hi = TSL2561_AGC_THI_13MS;
        _lo = TSL2561_AGC_TLO_13MS;
        break;
      case TSL2561_INTEGRATIONTIME_101MS:
        _hi = TSL2561_AGC_THI_101MS;
        _lo = TSL2561_AGC_TLO_101MS;
        break;
      default:
        _hi = TSL2561_AGC_THI_402MS;
        _lo = TSL2561_AGC_TLO_402MS;
        break;
    }

    ASSERT_STATUS(tsl2561GetData(&_b, &_ir));

    /* Run an auto-gain check if we haven't already done so ... */
    if (!_agcCheck)
    {
      if ((_b < _lo) && (_tsl2561Gain == TSL2561_GAIN_1X))
      {
        /* Increase the gain and try again */
        ASSERT_STATUS(tsl2561SetGain(TSL2561_GAIN_16X));
        /* Drop the previous conversion results */
        ASSERT_STATUS(tsl2561GetData(&_b, &_ir));
        /* Set a flag to indicate we've adjusted the gain */
        _agcCheck = true;
      }
      else if ((_b > _hi) && (_tsl2561Gain == TSL2561_GAIN_16X))
      {
        /* Drop gain to 1x and try again */
        ASSERT_STATUS(tsl2561SetGain(TSL2561_GAIN_1X));
        /* Drop the previous conversion results */
        ASSERT_STATUS(tsl2561GetData(&_b, &_ir));
        /* Set a flag to indicate we've adjusted the gain */
        _agcCheck = true;
      }
      else
      {
        /* Nothing to look at here, keep moving ....
           Reading is either valid, or we're already at the chips limits */
        *broadband = _b;
        *ir = _ir;
        valid = true;
      }
    }
    else
    {
      /* If we've already adjusted the gain once, just return the new results.
         This avoids endless loops where a value is at one extreme pre-gain,
         and the the other extreme post-gain */
      *broadband = _b;
      *ir = _ir;
      valid = true;
    }
  } while (!valid);

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Calculates LUX from the supplied ch0 (broadband) and ch1
            (IR) readings
*/
/**************************************************************************/
uint32_t tsl2561CalculateLux(uint16_t ch0, uint16_t ch1)
{
  unsigned long chScale;
  unsigned long channel1;
  unsigned long channel0;
  uint32_t lux;
  unsigned long temp;
  unsigned long ratio1;
  unsigned long ratio;
  unsigned int b, m;

  /* Make sure the sensor isn't saturated! */
  uint16_t clipThreshold;
  switch (_tsl2561IntegrationTime)
  {
    case TSL2561_INTEGRATIONTIME_13MS:
      /* Ti 13ms, max value is 5047 */
      clipThreshold = TSL2561_CLIPPING_13MS;
      break;
    case TSL2561_INTEGRATIONTIME_101MS:
      /* Ti 13ms, max value is 37177 */
      clipThreshold = TSL2561_CLIPPING_101MS;
      break;
    default:
      /* Ti 13ms, max value is 65535 */
      clipThreshold = TSL2561_CLIPPING_402MS;
      break;
  }

  /* Return 0 is sensor is saturated */
  if ((ch0 > clipThreshold) || (ch1 > clipThreshold))
  {
    return 0;
  }

  /* Scale factor for integration time */
  switch (_tsl2561IntegrationTime)
  {
    case TSL2561_INTEGRATIONTIME_13MS:
      chScale = TSL2561_LUX_CHSCALE_TINT0;
      break;
    case TSL2561_INTEGRATIONTIME_101MS:
      chScale = TSL2561_LUX_CHSCALE_TINT1;
      break;
    default: // No scaling ... integration time = 402ms
      chScale = (1 << TSL2561_LUX_CHSCALE);
      break;
  }

  /* Scale factor for gain (1x or 16x) */
  if (!_tsl2561Gain) chScale = chScale << 4;

  /* Scale the channel values */
  channel0 = (ch0 * chScale) >> TSL2561_LUX_CHSCALE;
  channel1 = (ch1 * chScale) >> TSL2561_LUX_CHSCALE;

  /* Find the ratio of the channel values (Channel1/Channel0) */
  ratio1 = 0;
  if (channel0 != 0) ratio1 = (channel1 << (TSL2561_LUX_RATIOSCALE+1)) / channel0;

  /* Round the ratio values */
  ratio = (ratio1 + 1) >> 1;

#ifdef TSL2561_PACKAGE_CS
  if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1C))
    {b=TSL2561_LUX_B1C; m=TSL2561_LUX_M1C;}
  else if (ratio <= TSL2561_LUX_K2C)
    {b=TSL2561_LUX_B2C; m=TSL2561_LUX_M2C;}
  else if (ratio <= TSL2561_LUX_K3C)
    {b=TSL2561_LUX_B3C; m=TSL2561_LUX_M3C;}
  else if (ratio <= TSL2561_LUX_K4C)
    {b=TSL2561_LUX_B4C; m=TSL2561_LUX_M4C;}
  else if (ratio <= TSL2561_LUX_K5C)
    {b=TSL2561_LUX_B5C; m=TSL2561_LUX_M5C;}
  else if (ratio <= TSL2561_LUX_K6C)
    {b=TSL2561_LUX_B6C; m=TSL2561_LUX_M6C;}
  else if (ratio <= TSL2561_LUX_K7C)
    {b=TSL2561_LUX_B7C; m=TSL2561_LUX_M7C;}
  else if (ratio > TSL2561_LUX_K8C)
    {b=TSL2561_LUX_B8C; m=TSL2561_LUX_M8C;}
#else
  if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1T))
    {b=TSL2561_LUX_B1T; m=TSL2561_LUX_M1T;}
  else if (ratio <= TSL2561_LUX_K2T)
    {b=TSL2561_LUX_B2T; m=TSL2561_LUX_M2T;}
  else if (ratio <= TSL2561_LUX_K3T)
    {b=TSL2561_LUX_B3T; m=TSL2561_LUX_M3T;}
  else if (ratio <= TSL2561_LUX_K4T)
    {b=TSL2561_LUX_B4T; m=TSL2561_LUX_M4T;}
  else if (ratio <= TSL2561_LUX_K5T)
    {b=TSL2561_LUX_B5T; m=TSL2561_LUX_M5T;}
  else if (ratio <= TSL2561_LUX_K6T)
    {b=TSL2561_LUX_B6T; m=TSL2561_LUX_M6T;}
  else if (ratio <= TSL2561_LUX_K7T)
    {b=TSL2561_LUX_B7T; m=TSL2561_LUX_M7T;}
  else if (ratio > TSL2561_LUX_K8T)
    {b=TSL2561_LUX_B8T; m=TSL2561_LUX_M8T;}
#endif

  temp = ((channel0 * b) - (channel1 * m));

  /* Round lsb (2^(LUX_SCALE-1)) */
  temp += (1 << (TSL2561_LUX_LUXSCALE-1));

  /* Strip off fractional portion */
  lux = temp >> TSL2561_LUX_LUXSCALE;

  return lux;
}

/**************************************************************************/
/*!
    @brief  Provides the sensor_t data for this sensor
*/
/**************************************************************************/
void tsl2561GetSensor(sensor_t *sensor)
{
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Note: Max lux can't be provided accurately because the max */
  /* value changes depending on the light source.  Fluorescent  */
  /* lights will have a higher max lux that can be calculated   */
  /* by the visible + IR sensors than incandescent lights, etc. */

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "TSL2561", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _tsl2561SensorID;
  sensor->type        = SENSOR_TYPE_LIGHT;
  sensor->min_delay   = 0;
  sensor->max_value   = 17000.0;  /* Based on trial and error ... confirm! */
  sensor->min_value   = 0.0;
  sensor->resolution  = 1.0;
}

/**************************************************************************/
/*!
    @brief  Reads the sensor and returns the data as a sensors_event_t
*/
/**************************************************************************/
err_t tsl2561GetSensorEvent(sensors_event_t *event)
{
  uint16_t broadband, ir;

  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _tsl2561SensorID;
  event->type      = SENSOR_TYPE_LIGHT;
  event->timestamp = delayGetTicks();

  /* Calculate the actual lux value */
  ASSERT_STATUS(tsl2561GetLuminosity(&broadband, &ir));
  event->light = tsl2561CalculateLux(broadband, ir);

  return ERROR_NONE;
}
