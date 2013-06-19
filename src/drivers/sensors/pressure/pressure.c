/**************************************************************************/
/*!
    @file     pressure.c
    @author   K. Townsend (microBuilder.eu)
    @ingroup  Sensors

    @brief    Helpers functions for working with pressure sensors

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

#include <math.h>
#include "drivers/sensors/sensors.h"
#include "pressure.h"

/**************************************************************************/
/*!
    Calculates the altitude (in meters) from the specified atmospheric
    pressure (in hPa), sea-level pressure (in hPa), and temperature (in °C)

    @param  seaLevel      Sea-level pressure in hPa
    @param  atmospheric   Atmospheric pressure in hPa
    @param  temp          Temperature in degrees Celsius
*/
/**************************************************************************/
float pressureToAltitude(float seaLevel, float atmospheric, float temp)
{
  /* Hyposometric formula:                      */
  /*                                            */
  /*     ((P0/P)^(1/5.257) - 1) * (T + 273.15)  */
  /* h = -------------------------------------  */
  /*                   0.0065                   */
  /*                                            */
  /* where: h   = height (in meters)            */
  /*        P0  = sea-level pressure (in hPa)   */
  /*        P   = atmospheric pressure (in hPa) */
  /*        T   = temperature (in °C)           */

  return (((float)pow((seaLevel/atmospheric), 0.190223F) - 1.0F)
         * (temp + 273.15F)) / 0.0065F;
}

/**************************************************************************/
/*!
    Calculates the sea-level pressure (in hPa) based on the current
    altitude (in meters), atmospheric pressure (in hPa), and temperature
    (in °C)

    @param  altitude      altitude in meters
    @param  atmospheric   Atmospheric pressure in hPa
    @param  temp          Temperature in degrees Celsius
*/
/**************************************************************************/
float pressureSeaLevelFromAltitude(float altitude, float atmospheric, float temp)
{
  /* Sea-level pressure:                        */
  /*                                            */
  /*                   0.0065*h                 */
  /* P0 = P * (1 - ----------------- ) ^ -5.257 */
  /*               T+0.0065*h+273.15            */
  /*                                            */
  /* where: P0  = sea-level pressure (in hPa)   */
  /*        P   = atmospheric pressure (in hPa) */
  /*        h   = altitude (in meters)          */
  /*        T   = Temperature (in °C)           */

  return atmospheric * (float)pow((1.0F - (0.0065 * altitude) /
          (temp + 0.0065 * altitude + 273.15F)), -5.257F);
}

/**************************************************************************/
/*!
    Calculates the temperature (in °C) at the destination altitude based
    on the current seal-level pressure (in hPa), altitude
    (in meters) and temperature (in C)

    @param  currTemp        Temperature at current altitude (in °C)
    @param  currAltitude    Current altitude (in meters)
    @param  destAltitude    Destination altitude (in meters)
*/
/**************************************************************************/
float pressureTempAtDestination(float currTemp, float currAltitude, float destAltitude)
{
  /* Temperature at destination:                */
  /*                                            */
  /* T = Ta - 0.0065(h - ha)                    */
  /*                                            */
  /* where: T   = Temp at destination (in °C)   */
  /*        Ta  = Temp at current location (°C) */
  /*        ha  = Current altitude (in meters)  */
  /*        h   = Target altitude (in meters)   */

  return currTemp - 0.0065F * (destAltitude - currAltitude);
}

/**************************************************************************/
/*!
    Calculates the atmospheric pressure (in hPa) at the destination
    altitude based on the current seal-level pressure (in hPa), altitude
    (in meters) and temperature (in C)

    @param  seaLevel        Sea-level pressure (in hPa)
    @param  destTemp        Temperature at the destination altitude (°C)
    @param  destAltitude    Destination altitude (in meters)

    @note Normally you will need to calculate the temperature at
          destination with pressureTempAtDestination() before running
          this function!
*/
/**************************************************************************/
float pressureAtDestination(float seaLevel, float destTemp, float destAltitude)
{
  /* Atmospheric pressure at destination:       */
  /*                                            */
  /*                0.0065 * h                  */
  /* P = P0 (1 - -----------------) ^ 5.257     */
  /*             T+0.0065*h+273.15              */
  /*                                            */
  /* where: P   = Pressure at destination (hPa) */
  /*        P0  = Sea-level pressure (hPa)      */
  /*        h   = Destination altitude (meters) */
  /*        T   = Destination temperature (°C)  */

  return seaLevel * (float)pow(1.0F - (0.0065F * destAltitude) /
          (destTemp + 0.0065F * destAltitude + 273.15F), 5.257F);
}

// ToDo: Calculate vertical speed over fixed delay
// float vertical_speed = ( pressureToAltitude(sea level pressure, pressure at t0, temp) – pressureToAltitude(sea level pressure, pressure at t1, temp) / ( t1 – t0 );