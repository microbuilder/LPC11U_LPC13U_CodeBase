/**************************************************************************/
/*!
    @file     l3gd20.c
    @author   K. Townsend (microBuilder.eu)
    @ingroup  Sensors

    @brief    Driver for the ST L3GD20 I2C Gyroscope

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
#include "l3gd20.h"
#include "core/delay/delay.h"
#include <string.h>

#define L3GD20_SENSITIVITY_250DPS  (0.00875F)   // Roughly 22/256 for fixed point match
#define L3GD20_SENSITIVITY_500DPS  (0.0175F)    // Roughly 45/256
#define L3GD20_SENSITIVITY_2000DPS (0.070F)     // Roughly 18/256

extern volatile uint8_t   I2CMasterBuffer[I2C_BUFSIZE];
extern volatile uint8_t   I2CSlaveBuffer[I2C_BUFSIZE];
extern volatile uint32_t  I2CReadLength, I2CWriteLength;

static bool     _l3gd20Initialised = false;
static uint16_t _l3gd20MeasurementRange = 250;  // 250, 500, or 2000
static int32_t  _l3gd20SensorID = 0;

/**************************************************************************/
/*!
    @brief  Writes an unsigned 8 bit value over I2C
*/
/**************************************************************************/
err_t l3gd20Write8 (uint8_t reg, uint8_t value)
{
  I2CWriteLength = 3;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = L3GD20_ADDRESS;
  I2CMasterBuffer[1] = reg;
  I2CMasterBuffer[2] = value;
  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads an unsigned 8 bit value over I2C
*/
/**************************************************************************/
err_t l3gd20Read8(uint8_t reg, uint8_t *value)
{
  /* Write transaction */
  I2CWriteLength = 2;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = L3GD20_ADDRESS;
  I2CMasterBuffer[1] = reg;
  i2cEngine();

  /* Read transaction */
  I2CWriteLength = 0;
  I2CReadLength = 1;
  I2CMasterBuffer[0] = L3GD20_ADDRESS | L3GD20_READBIT;
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
err_t l3gd20Read48(uint8_t reg, int16_t *x, int16_t *y, int16_t *z)
{
  /* Write transaction */
  I2CWriteLength = 2;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = L3GD20_ADDRESS;
  I2CMasterBuffer[1] = reg | (0x80);
  i2cEngine();

  /* Read transaction */
  I2CWriteLength = 0;
  I2CReadLength = 6;
  I2CMasterBuffer[0] = L3GD20_ADDRESS | L3GD20_READBIT;
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

    @section EXAMPLE

    @code
    if (lis3dhInit())
    {
      printf("LIS3DH: %s%s", STRING(STRINGS_TEXT_No_response_on_the_I2C_bus), CFG_PRINTF_NEWLINE);
    }
    else
    {
      int16_t x, y, z;
      while(1)
      {
        lis3dhPoll(&x, &y, &z);
        printf("X: %i, Y: %i, Z: %i %s", x, y, z, CFG_PRINTF_NEWLINE);
      }
    }
    @endcode
*/
/**************************************************************************/
err_t l3gd20Init(void)
{
  uint8_t id;

  /* Initialise I2C */
  i2cInit(I2CMASTER);

  /* Ping the I2C device first to see if it exists! */
  ASSERT(i2cCheckAddress(L3GD20_ADDRESS), ERROR_I2C_DEVICENOTFOUND);

  /* Check the ID register to make sure everything is correct */
  ASSERT_STATUS(l3gd20Read8(L3GD20_REGISTER_WHO_AM_I, &id));

  /* Check the address against the expected value, etc. */
  ASSERT(id == L3GD20_ID, ERROR_I2C_DEVICENOTFOUND);

  /* Set CTRL_REG1 (0x20)
     ====================================================================
     BIT  Symbol    Description                                   Default
     ---  ------    --------------------------------------------- -------
     7-6  DR1/0     Output data rate                                   00
     5-4  BW1/0     Bandwidth selection                                00
       3  PD        0 = Power-down mode, 1 = normal/sleep mode          0
       2  ZEN       Z-axis enable (0 = disabled, 1 = enabled)           1
       1  YEN       Y-axis enable (0 = disabled, 1 = enabled)           1
       0  XEN       X-axis enable (0 = disabled, 1 = enabled)           1 */

  /* Switch to normal mode and enable all three channels */
  ASSERT_STATUS(l3gd20Write8(L3GD20_REGISTER_CTRL_REG1, 0x0F));

  /* Set CTRL_REG2 (0x21)
     ====================================================================
     BIT  Symbol    Description                                   Default
     ---  ------    --------------------------------------------- -------
     5-4  HPM1/0    High-pass filter mode selection                    00
     3-0  HPCF3..0  High-pass filter cutoff frequency selection      0000 */

  /* Set CTRL_REG3 (0x22)
     ====================================================================
     BIT  Symbol    Description                                   Default
     ---  ------    --------------------------------------------- -------
       7  I1_Int1   Interrupt enable on INT1 (0=disable,1=enable)       0
       6  I1_Boot   Boot status on INT1 (0=disable,1=enable)            0
       5  H-Lactive Interrupt active config on INT1 (0=high,1=low)      0
       4  PP_OD     Push-Pull/Open-Drain (0=PP, 1=OD)                   0
       3  I2_DRDY   Data ready on DRDY/INT2 (0=disable,1=enable)        0
       2  I2_WTM    FIFO wtrmrk int on DRDY/INT2 (0=dsbl,1=enbl)        0
       1  I2_ORun   FIFO overrun int on DRDY/INT2 (0=dsbl,1=enbl)       0
       0  I2_Empty  FIFI empty int on DRDY/INT2 (0=dsbl,1=enbl)         0 */

  /* Set CTRL_REG4 (0x23)
     ====================================================================
     BIT  Symbol    Description                                   Default
     ---  ------    --------------------------------------------- -------
       7  BDU       Block Data Update (0=continuous, 1=LSB/MSB)         0
       6  BLE       Big/Little-Endian (0=Data LSB, 1=Data MSB)          0
     5-4  FS1/0     Full scale selection                               00
                    00 = 250 dps
                    01 = 500 dps
                    10 = 2000 dps
                    11 = 2000 dps
       0  SIM       SPI Mode (0=4-wire, 1=3-wire)                       0 */

  /* Enable BDU for more accurate measurements */
  ASSERT_STATUS(l3gd20Write8(L3GD20_REGISTER_CTRL_REG4, 0x1 << 7));

  /* Make sure to update the measurement range if you change it here! */
  _l3gd20MeasurementRange = 250;

  /* Set CTRL_REG5 (0x24)
     ====================================================================
     BIT  Symbol    Description                                   Default
     ---  ------    --------------------------------------------- -------
       7  BOOT      Reboot memory content (0=normal, 1=reboot)          0
       6  FIFO_EN   FIFO enable (0=FIFO disable, 1=enable)              0
       4  HPen      High-pass filter enable (0=disable,1=enable)        0
     3-2  INT1_SEL  INT1 Selection config                              00
     1-0  OUT_SEL   Out selection config                               00 */

  _l3gd20Initialised = true;

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Polls the device for a new X/Y/Z reading
*/
/**************************************************************************/
err_t l3gd20Poll(l3gd20Data_t* data)
{
  uint8_t timeout = 0;
  uint8_t buffer;

  if (!_l3gd20Initialised)
  {
    ASSERT_STATUS(l3gd20Init());
  }

  /* Wait for a new X/Y/Z sample */
  do
  {
    ASSERT_STATUS(l3gd20Read8(L3GD20_REGISTER_STATUS_REG, &buffer));
    ASSERT(timeout++ <= L3GD20_POLL_TIMEOUT, ERROR_OPERATIONTIMEDOUT);
  } while (!(buffer & (1<<3)));

  /* Read data (in degrees per second) */
  ASSERT_STATUS(l3gd20Read48(L3GD20_REGISTER_OUT_X_L | (1<<7), &(data->x), &(data->y), &(data->z)));

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Provides the sensor_t data for this sensor
*/
/**************************************************************************/
void l3gd20GetSensor(sensor_t *sensor)
{
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "L3GD20", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _l3gd20SensorID;
  sensor->type        = SENSOR_TYPE_GYROSCOPE;
  sensor->min_delay   = 0;

  /* We need to do some calculations to determine resolution and maxRange in rad/s */
  sensor->max_value   = _l3gd20MeasurementRange * SENSORS_DPS_TO_RADS;
  sensor->min_value   = 0;
  switch (_l3gd20MeasurementRange)
  {
    case 2000:
      sensor->resolution  = (_l3gd20MeasurementRange / 32767.0F) * L3GD20_SENSITIVITY_2000DPS * SENSORS_DPS_TO_RADS;
      break;
    case 500:
      sensor->resolution  = (_l3gd20MeasurementRange / 32767.0F) * L3GD20_SENSITIVITY_500DPS * SENSORS_DPS_TO_RADS;
      break;
    case 250:
    default:
      sensor->resolution  = (_l3gd20MeasurementRange / 32767.0F) * L3GD20_SENSITIVITY_250DPS * SENSORS_DPS_TO_RADS;
      break;
  }
}

/**************************************************************************/
/*!
    @brief  Reads the sensor and returns the data as a sensors_event_t
*/
/**************************************************************************/
err_t l3gd20GetSensorEvent(sensors_event_t *event)
{
  l3gd20Data_t data;

  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = sizeof(sensors_event_t);
  event->sensor_id = _l3gd20SensorID;
  event->type      = SENSOR_TYPE_GYROSCOPE;
  event->timestamp = delayGetTicks();

  /* Retrieve values from the sensor */
  ASSERT_STATUS(l3gd20Poll(&data));

  /* The L3GD20 returns degrees per second, adjusted by sensitivity which
   * is shown as mdps/digit in the datasheet.  To convert this to proper
   * degrees/s multiply by the appropriate sensitivity value and then
   * convert it to the rad/s value that sensors_event_t is expecting. */
  switch (_l3gd20MeasurementRange)
  {
    case (2000):
      event->gyro.x = data.x * L3GD20_SENSITIVITY_2000DPS * SENSORS_DPS_TO_RADS;
      event->gyro.y = data.y * L3GD20_SENSITIVITY_2000DPS * SENSORS_DPS_TO_RADS;
      event->gyro.z = data.z * L3GD20_SENSITIVITY_2000DPS * SENSORS_DPS_TO_RADS;
      break;
    case (500):
      event->gyro.x = data.x * L3GD20_SENSITIVITY_500DPS * SENSORS_DPS_TO_RADS;
      event->gyro.y = data.y * L3GD20_SENSITIVITY_500DPS * SENSORS_DPS_TO_RADS;
      event->gyro.z = data.z * L3GD20_SENSITIVITY_500DPS * SENSORS_DPS_TO_RADS;
      break;
    case (250):
    default:
      event->gyro.x = data.x * L3GD20_SENSITIVITY_250DPS * SENSORS_DPS_TO_RADS;
      event->gyro.y = data.y * L3GD20_SENSITIVITY_250DPS * SENSORS_DPS_TO_RADS;
      event->gyro.z = data.z * L3GD20_SENSITIVITY_250DPS * SENSORS_DPS_TO_RADS;
      break;
  }

  return ERROR_NONE;
}
