/**************************************************************************/
/*!
    @file     tcs34725.c
    @author   K. Townsend (microBuilder.eu)
    @ingroup  Sensors
    @brief    Driver for the TAOS TCS34725 I2C digital RGB/color sensor
    @license  BSD (see license.txt)
*/
/**************************************************************************/
#include <string.h>
#include <math.h>

#include "projectconfig.h"
#include "tcs34725.h"
#include "core/delay/delay.h"

extern volatile uint8_t  I2CMasterBuffer[I2C_BUFSIZE];
extern volatile uint8_t  I2CSlaveBuffer[I2C_BUFSIZE];
extern volatile uint32_t I2CReadLength, I2CWriteLength;

static bool                       _tcs34725Initialised = false;
static int32_t                    _tcs34725SensorID = 0;
static tcs34725Gain_t             _tcs34725Gain = TCS34725_GAIN_1X;
static tcs34725IntegrationTime_t  _tcs34725IntegrationTime = TCS34725_INTEGRATIONTIME_2_4MS;

/**************************************************************************/
/*!
    @brief  Writes an 8 bit values over I2C
*/
/**************************************************************************/
err_t tcs34725Write8 (uint8_t reg, uint8_t value)
{
  I2CWriteLength = 3;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = TCS34725_ADDRESS;
  I2CMasterBuffer[1] = TCS34725_COMMAND_BIT | reg;
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
err_t tcs34725Read8(uint8_t reg, uint8_t *value)
{
  /* Write transaction */
  I2CWriteLength = 2;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = TCS34725_ADDRESS;
  I2CMasterBuffer[1] = TCS34725_COMMAND_BIT | reg;
  i2cEngine();

  /* Read transaction */
  I2CWriteLength = 0;
  I2CReadLength = 1;
  I2CMasterBuffer[0] = TCS34725_ADDRESS | TCS34725_READBIT;
  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  *value = I2CSlaveBuffer[0];

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/
err_t tcs34725Read16(uint8_t reg, uint16_t *value)
{
  /* Write transaction */
  I2CWriteLength = 2;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = TCS34725_ADDRESS;
  I2CMasterBuffer[1] = TCS34725_COMMAND_BIT | reg;
  i2cEngine();

  /* Read transaction */
  I2CWriteLength = 0;
  I2CReadLength = 2;
  I2CMasterBuffer[0] = TCS34725_ADDRESS | TCS34725_READBIT;
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
err_t tcs34725Enable(void)
{
  ASSERT_STATUS(tcs34725Write8(TCS34725_ENABLE, TCS34725_ENABLE_PON));
  delay(3);
  ASSERT_STATUS(tcs34725Write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Disables the device (putting it in lower power sleep mode)
*/
/**************************************************************************/
err_t tcs34725Disable(void)
{
  /* Turn the device off to save power */
  uint8_t reg = 0;
  ASSERT_STATUS(tcs34725Read8(TCS34725_ENABLE, &reg));
  ASSERT_STATUS(tcs34725Write8(TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN)));

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Initialises the I2C block
*/
/**************************************************************************/
err_t tcs34725Init(void)
{
  uint8_t id = 0;

  /* Initialise I2C */
  i2cInit(I2CMASTER);

  /* Ping the I2C device first to see if it exists! */
  ASSERT(i2cCheckAddress(TCS34725_ADDRESS), ERROR_I2C_DEVICENOTFOUND);

  /* Make sure we have the right IC (0x44 = TCS34725 and TCS34721) */
  ASSERT_STATUS(tcs34725Read8(TCS34725_ID, &id));
  ASSERT(id == 0x44, ERROR_DEVICENOTINITIALISED);

  /* Enable the device */
  ASSERT_STATUS(tcs34725Enable());

  /* Ready to go ... set the initialised flag */
  _tcs34725Initialised = true;

  /* This needs to take place after the initialisation flag! */
  ASSERT_STATUS(tcs34725SetIntegrationTime(TCS34725_INTEGRATIONTIME_2_4MS));
  ASSERT_STATUS(tcs34725SetGain(TCS34725_GAIN_60X));

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Sets the integration time to the specified value
*/
/**************************************************************************/
err_t tcs34725SetIntegrationTime(tcs34725IntegrationTime_t it)
{
  if (!_tcs34725Initialised)
  {
    ASSERT_STATUS(tcs34725Init());
  }

  ASSERT_STATUS(tcs34725Write8(TCS34725_ATIME, it));
  _tcs34725IntegrationTime = it;

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Sets gain to the specified value
*/
/**************************************************************************/
err_t tcs34725SetGain(tcs34725Gain_t gain)
{
  if (!_tcs34725Initialised)
  {
    ASSERT_STATUS(tcs34725Init());
  }

  ASSERT_STATUS(tcs34725Write8(TCS34725_CONTROL, gain));
  _tcs34725Gain = gain;

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads the raw red, green, blue and clear channel values
*/
/**************************************************************************/
err_t tcs34725GetRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  if (!_tcs34725Initialised)
  {
    ASSERT_STATUS(tcs34725Init());
  }

  /* ToDo: Insert a blocky delay until the data is ready! */
  ASSERT_STATUS(tcs34725Read16(TCS34725_CDATAL, c));
  ASSERT_STATUS(tcs34725Read16(TCS34725_RDATAL, r));
  ASSERT_STATUS(tcs34725Read16(TCS34725_GDATAL, g));
  ASSERT_STATUS(tcs34725Read16(TCS34725_BDATAL, b));

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Converts the raw R/G/B values to color temperature in degrees
            Kelvin
*/
/**************************************************************************/
uint16_t tcs34725CalculateColorTemperature(uint16_t r, uint16_t g, uint16_t b)
{
  float X, Y, Z;      /* RGB to XYZ correlation      */
  float xc, yc;       /* Chromaticity co-ordinates   */
  float n;            /* McCamy's formula            */
  float cct;

  /* 1. Map RGB values to their XYZ counterparts.    */
  /* Based on 6500K fluorescent, 3000K fluorescent   */
  /* and 60W incandescent values for a wide range.   */
  /* Note: Y = Illuminance or lux                    */
  X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
  Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
  Z = (-0.68202F * r) + (0.77073F * g) + ( 0.56332F * b);

  /* 2. Calculate the chromaticity co-ordinates      */
  xc = (X) / (X + Y + Z);
  yc = (Y) / (X + Y + Z);

  /* 3. Use McCamy's formula to determine the CCT    */
  n = (xc - 0.3320F) / (0.1858F - yc);

  /* Calculate the final CCT */
  cct = (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

  /* Return the results in degrees Kelvin */
  return (uint16_t)cct;
}

/**************************************************************************/
/*!
    @brief  Converts the raw R/G/B values to color temperature in degrees
            Kelvin
*/
/**************************************************************************/
uint16_t tcs34725CalculateLux(uint16_t r, uint16_t g, uint16_t b)
{
  float illuminance;

  /* This only uses RGB ... how can we integrate clear or calculate lux */
  /* based exclusively on clear since this might be more reliable?      */
  illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

  return (uint16_t)illuminance;
}

/**************************************************************************/
/*!
    @brief  Provides the sensor_t data for this sensor
*/
/**************************************************************************/
void tcs34725GetSensor(sensor_t *sensor)
{
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "TCS34725", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _tcs34725SensorID;
  sensor->type        = SENSOR_TYPE_COLOR;
  sensor->min_delay   = 0;
  sensor->max_value   = 0.0;
  sensor->min_value   = 1.0;
  sensor->resolution  = 1.0;
}

/**************************************************************************/
/*!
    @brief  Reads the sensor and returns the data as a sensors_event_t
*/
/**************************************************************************/
err_t tcs34725GetSensorEvent(sensors_event_t *event)
{
  float maxCount;
  uint16_t r, g, b, c;

  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _tcs34725SensorID;
  event->type      = SENSOR_TYPE_COLOR;
  event->timestamp = delayGetTicks();

  /* Get the raw RGB values */
  ASSERT_STATUS(tcs34725GetRawData(&r, &g, &b, &c));

  /* Convert RGB values to floats */
  maxCount = (256 - _tcs34725IntegrationTime) * 1024;
  event->color.r = r / maxCount;
  event->color.g = g / maxCount;
  event->color.b = b / maxCount;

  /* Convert to a 24-bit ARGB value */
  event->color.rgba = (uint8_t)(event->color.r * 0xFF) << 16|
                      (uint8_t)(event->color.g * 0xFF) << 8 |
                      (uint8_t)(event->color.b * 0xFF) |
                      0xFF000000;

  return ERROR_NONE;
}
