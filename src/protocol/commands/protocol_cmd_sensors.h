/**************************************************************************/
/*!
    @file     protocol_cmd_sensors.h
    @author   K. Townsend (microBuilder.eu)

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

#ifndef __PROTOCOL_CMD_SENSORS_H__
#define __PROTOCOL_CMD_SENSORS_H__

#ifdef __cplusplus
 extern "C" {
#endif

/**************************************************************************/
/*!
    SENSOR Keys (indicates what specific system sensor we want)

    Key Range     Sensor Type           'sensors_type_t' Type (value)
    ------------  --------------------  -----------------------------------
    0x0001..00FF  RESERVED
    0x0100..01FF  Accelerometers        SENSOR_TYPE_ACCELEROMETER (1)
    0x0200..02FF  Magnetometers         SENSOR_TYPE_MAGNETIC_FIELD (2)
    0x0300..03FF  Orientation           SENSOR_TYPE_ORIENTATION (3)
    0x0400..04FF  Gyroscopes            SENSOR_TYPE_GYROSCOPE (4)
    0x0500..05FF  Light Sensors         SENSOR_TYPE_LIGHT (5)
    0x0600..06FF  Pressure Sensors      SENSOR_TYPE_PRESSURE (6)
    0x0700..07FF  Color                 SENSOR_TYPE_COLOR (7)
    0x0800..08FF  Proximity             SENSOR_TYPE_PROXIMITY (8)
    0x0C00..0CFF  Humidity              SENSOR_TYPE_RELATIVE_HUMIDITY (9)
    0x0D00..0DFF  Ambient Temperature   SENSOR_TYPE_AMBIENT_TEMPERATURE (10)
    0x0E00..0EFF  Voltage               SENSOR_TYPE_VOLTAGE (11)
    0x0F00..0FFF  Current               SENSOR_TYPE_CURRENT (12)
*/
/**************************************************************************/
typedef enum
{
  PROT_CMD_SENSORS_KEY_FIRST                  = 0x0000,

  /*=======================================================================
    Accelerometers                                         0x0100 .. 0x01FF
    -----------------------------------------------------------------------*/
    PROT_CMD_SENSORS_KEY_ACCEL_DEFAULT        = 0x0100,   /**< Tries to read the default accelerometer */
    PROT_CMD_SENSORS_KEY_ACCEL_LSM303         = 0x0101,   /**< LSM303DMHC 3-Axis Accelerometer */
  /*=======================================================================*/


  /*=======================================================================
    Magnetometers                                          0x0200 .. 0x02FF
    -----------------------------------------------------------------------*/
    PROT_CMD_SENSORS_KEY_MAG_DEFAULT          = 0x0200,   /**< Tries to read the default magnetometer */
    PROT_CMD_SENSORS_KEY_MAG_LSM303           = 0x0201,   /**< LSM303DLHC 3-Axis Magnetometer */
  /*=======================================================================*/


  /*=======================================================================
    Orientation                                            0x0300 .. 0x03FF
    -----------------------------------------------------------------------*/
    PROT_CMD_SENSORS_KEY_ORIENT_DEFAULT       = 0x0300,   /**< Tries to read the default orientation sensor */
  /*=======================================================================*/


  /*=======================================================================
    Gyroscopes                                             0x0400 .. 0x04FF
    -----------------------------------------------------------------------*/
    PROT_CMD_SENSORS_KEY_GYRO_DEFAULT         = 0x0400,   /**< Tries to read the default gyroscope */
  /*=======================================================================*/


  /*=======================================================================
    Light Sensors                                          0x0500 .. 0x05FF
    -----------------------------------------------------------------------*/
    PROT_CMD_SENSORS_KEY_LIGHT_DEFAULT        = 0x0500,   /**< Tries to read the default light sensor */
  /*=======================================================================*/


  /*=======================================================================
    Pressure Sensors                                       0x0600 .. 0x06FF
    -----------------------------------------------------------------------*/
    PROT_CMD_SENSORS_KEY_PRESS_DEFAULT        = 0x0600,   /**< Tries to read the default pressure sensor */
  /*=======================================================================*/


  /*=======================================================================
    Color                                                  0x0700 .. 0x07FF
    -----------------------------------------------------------------------*/
    PROT_CMD_SENSORS_KEY_COLOR_DEFAULT        = 0x0700,   /**< Tries to read the default color sensor */
  /*=======================================================================*/


  /*=======================================================================
    Proximity                                              0x0800 .. 0x08FF
    -----------------------------------------------------------------------*/
    PROT_CMD_SENSORS_KEY_PROX_DEFAULT         = 0x0800,   /**< Tries to read the default proximity sensor */
  /*=======================================================================*/


  /*=======================================================================
    Humidity                                               0x0900 .. 0x09FF
    -----------------------------------------------------------------------*/
    PROT_CMD_SENSORS_KEY_HUMIDITY_DEFAULT     = 0x0C00,   /**< Tries to read the default humidity sensor */
  /*=======================================================================*/


  /*=======================================================================
    Ambient Temperature                                    0x0A00 .. 0x0AFF
    -----------------------------------------------------------------------*/
    PROT_CMD_SENSORS_KEY_AMBTEMP_DEFAULT      = 0x0D00,   /**< Tries to read the default ambient temperature sensor */
  /*=======================================================================*/


  /*=======================================================================
    Voltage                                                0x0B00 .. 0x0BFF
    -----------------------------------------------------------------------*/
    PROT_CMD_SENSORS_KEY_VOLT_DEFAULT         = 0x0E00,   /**< Tries to read the default voltage sensor */
  /*=======================================================================*/


  /*=======================================================================
    Current                                                0x0C00 .. 0x0CFF
    -----------------------------------------------------------------------*/
    PROT_CMD_SENSORS_KEY_CURRENT_DEFAULT      = 0x0F00,   /**< Tries to read the default current sensor */
  /*=======================================================================*/

  PROT_CMD_SENSORS_KEY_LAST
} prot_cmd_sensors_key_t;

#ifdef __cplusplus
 }
#endif

#endif /* __PROTOCOL_CMD_SENSORS_H__ */

/** @} */
