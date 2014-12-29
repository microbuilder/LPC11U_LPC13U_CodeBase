/**************************************************************************/
/*!
    @file     adxl345.h
    @author   K. Townsend (microBuilder.eu)
    @ingroup  Sensors

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
#ifndef _ADXL345_H_
#define _ADXL345_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"
#include "core/i2c/i2c.h"
#include "drivers/sensors/sensors.h"

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define ADXL345_ADDRESS                 (0x53<<1) // Assumes ALT address pin low ... use 0x3A with ALT high
    #define ADXL345_READBIT                 (0x01)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    #define ADXL345_REG_DEVID               (0x00)    // Device ID
    #define ADXL345_REG_THRESH_TAP          (0x1D)    // Tap threshold
    #define ADXL345_REG_OFSX                (0x1E)    // X-axis offset
    #define ADXL345_REG_OFSY                (0x1F)    // Y-axis offset
    #define ADXL345_REG_OFSZ                (0x20)    // Z-axis offset
    #define ADXL345_REG_DUR                 (0x21)    // Tap duration
    #define ADXL345_REG_LATENT              (0x22)    // Tap latency
    #define ADXL345_REG_WINDOW              (0x23)    // Tap window
    #define ADXL345_REG_THRESH_ACT          (0x24)    // Activity threshold
    #define ADXL345_REG_THRESH_INACT        (0x25)    // Inactivity threshold
    #define ADXL345_REG_TIME_INACT          (0x26)    // Inactivity time
    #define ADXL345_REG_ACT_INACT_CTL       (0x27)    // Axis enable control for activity and inactivity detection
    #define ADXL345_REG_THRESH_FF           (0x28)    // Free-fall threshold
    #define ADXL345_REG_TIME_FF             (0x29)    // Free-fall time
    #define ADXL345_REG_TAP_AXES            (0x2A)    // Axis control for single/double tap
    #define ADXL345_REG_ACT_TAP_STATUS      (0x2B)    // Source for single/double tap
    #define ADXL345_REG_BW_RATE             (0x2C)    // Data rate and power mode control
    #define ADXL345_REG_POWER_CTL           (0x2D)    // Power-saving features control
    #define ADXL345_REG_INT_ENABLE          (0x2E)    // Interrupt enable control
    #define ADXL345_REG_INT_MAP             (0x2F)    // Interrupt mapping control
    #define ADXL345_REG_INT_SOURCE          (0x30)    // Source of interrupts
    #define ADXL345_REG_DATA_FORMAT         (0x31)    // Data format control
    #define ADXL345_REG_DATAX0              (0x32)    // X-axis data 0
    #define ADXL345_REG_DATAX1              (0x33)    // X-axis data 1
    #define ADXL345_REG_DATAY0              (0x34)    // Y-axis data 0
    #define ADXL345_REG_DATAY1              (0x35)    // Y-axis data 1
    #define ADXL345_REG_DATAZ0              (0x36)    // Z-axis data 0
    #define ADXL345_REG_DATAZ1              (0x37)    // Z-axis data 1
    #define ADXL345_REG_FIFO_CTL            (0x38)    // FIFO control
    #define ADXL345_REG_FIFO_STATUS         (0x39)    // FIFO status
/*=========================================================================*/

/* Used with register 0x2C (ADXL345_REG_BW_RATE) to set bandwidth */
typedef enum
{
  ADXL345_DATARATE_3200_HZ    = BIN8(1111), // 1600Hz Bandwidth   140µA IDD
  ADXL345_DATARATE_1600_HZ    = BIN8(1110), //  800Hz Bandwidth    90µA IDD
  ADXL345_DATARATE_800_HZ     = BIN8(1101), //  400Hz Bandwidth   140µA IDD
  ADXL345_DATARATE_400_HZ     = BIN8(1100), //  200Hz Bandwidth   140µA IDD
  ADXL345_DATARATE_200_HZ     = BIN8(1011), //  100Hz Bandwidth   140µA IDD
  ADXL345_DATARATE_100_HZ     = BIN8(1010), //   50Hz Bandwidth   140µA IDD (Default)
  ADXL345_DATARATE_50_HZ      = BIN8(1001), //   25Hz Bandwidth    90µA IDD
  ADXL345_DATARATE_25_HZ      = BIN8(1000), // 12.5Hz Bandwidth    60µA IDD
  ADXL345_DATARATE_12_5_HZ    = BIN8(0111), // 6.25Hz Bandwidth    50µA IDD
  ADXL345_DATARATE_6_25HZ     = BIN8(0110), // 3.13Hz Bandwidth    45µA IDD
  ADXL345_DATARATE_3_13_HZ    = BIN8(0101), // 1.56Hz Bandwidth    40µA IDD
  ADXL345_DATARATE_1_56_HZ    = BIN8(0100), // 0.78Hz Bandwidth    34µA IDD
  ADXL345_DATARATE_0_78_HZ    = BIN8(0011), // 0.39Hz Bandwidth    23µA IDD
  ADXL345_DATARATE_0_39_HZ    = BIN8(0010), // 0.20Hz Bandwidth    23µA IDD
  ADXL345_DATARATE_0_20_HZ    = BIN8(0001), // 0.10Hz Bandwidth    23µA IDD
  ADXL345_DATARATE_0_10_HZ    = BIN8(0000)  // 0.05Hz Bandwidth    23µA IDD
} adxl345_dataRate_t;

/* Used with register 0x31 (ADXL345_REG_DATA_FORMAT) to set g range */
typedef enum
{
  ADXL345_RANGE_16_G          = 0x03,   // +/- 16g
  ADXL345_RANGE_8_G           = 0x02,   // +/- 8g
  ADXL345_RANGE_4_G           = 0x01,   // +/- 4g
  ADXL345_RANGE_2_G           = 0x00    // +/- 2g  (Default)
} adxl345_range_t;

err_t adxl345Init(void);
err_t adxl345GetXYZ(int16_t *x, int16_t *y, int16_t *z);
err_t adxl345SetRange(adxl345_range_t range);
err_t adxl345GetRange(adxl345_range_t *range);
err_t adxl345SetDataRate(adxl345_dataRate_t dataRate);
err_t adxl345GetDataRate(adxl345_dataRate_t *dataRate);
void    adxl345GetSensor(sensor_t *sensor);
err_t adxl345GetSensorEvent(sensors_event_t *event);

#ifdef __cplusplus
}
#endif

#endif
