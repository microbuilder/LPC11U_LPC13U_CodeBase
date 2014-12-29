/**************************************************************************/
/*!
    @file     sensors.c
    @author   K. Townsend (microBuilder.eu)

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
#include "sensors.h"
#include <string.h>
#include <math.h>

/**************************************************************************/
/*!
    @brief  Serializes a sensor_t object as a byte array

    @code

    sensor_t sensor;

    // Get the sensor_t data
    mpl115a2GetSensor(&sensor);

    // Create a serialisation buffer
    uint8_t buf[sizeof(event)];

    // Serialize sensor data into a byte array
    sensorsSerializeSensor(buf, &sensor);

    // Broadcast the event data over the air
    msgSend(0xFFFF, MSG_MESSAGETYPE_SENSORDETAILS, buf, sizeof(sensor));

    @endcode
*/
/**************************************************************************/
size_t sensorsSerializeSensor(uint8_t *buffer, const sensor_t *sensor)
{
   size_t i = 0;

   memcpy(&buffer[i], &sensor->name, sizeof sensor->name);
   i += sizeof sensor->name;
   memcpy(&buffer[i], &sensor->version, sizeof sensor->version);
   i += sizeof sensor->version;
   memcpy(&buffer[i], &sensor->sensor_id, sizeof sensor->sensor_id);
   i += sizeof sensor->sensor_id;
   memcpy(&buffer[i], &sensor->type, sizeof sensor->type);
   i += sizeof sensor->type;
   memcpy(&buffer[i], &sensor->max_value, sizeof sensor->max_value);
   i += sizeof sensor->max_value;
   memcpy(&buffer[i], &sensor->min_value, sizeof sensor->min_value);
   i += sizeof sensor->min_value;
   memcpy(&buffer[i], &sensor->resolution, sizeof sensor->resolution);
   i += sizeof sensor->resolution;
   memcpy(&buffer[i], &sensor->min_delay, sizeof sensor->min_delay);
   i += sizeof sensor->min_delay;

   return i;
}

/**************************************************************************/
/*!
    @brief  Serializes a sensors_event_t object as a byte array

    @code

    err_t error;
    sensors_event_t event;

    error = mpl115a2GetSensorEvent(&event);
    if (!error)
    {
      // Serialization buffer
      uint8_t msgbuf[sizeof(event)];

      // Serialize event data into a byte array
      sensorsSerializeSensorsEvent(msgbuf, &event);

      // Broadcast the event data over the air
      msgSend(0xFFFF, MSG_MESSAGETYPE_SENSOREVENT, msgbuf, sizeof(event));
    }

    @endcode
*/
/**************************************************************************/
size_t sensorsSerializeSensorsEvent(uint8_t *buffer,
    const sensors_event_t *event)
{
   size_t i = 0;

   memcpy(&buffer[i], &event->version, sizeof event->version);
   i += sizeof event->version;
   memcpy(&buffer[i], &event->sensor_id, sizeof event->sensor_id);
   i += sizeof event->sensor_id;
   memcpy(&buffer[i], &event->type, sizeof event->type);
   i += sizeof event->type;
   memcpy(&buffer[i], &event->reserved0, sizeof event->reserved0);
   i += sizeof event->reserved0;
   memcpy(&buffer[i], &event->timestamp, sizeof event->timestamp);
   i += sizeof event->timestamp;
   memcpy(&buffer[i], &event->data, sizeof event->data);
   i += sizeof event->data;

   return i;
}

/**************************************************************************/
/*!
    Places the sensor details in a text buffer for data logging purposes
*/
/**************************************************************************/
size_t sensorsLogSensor(char *buffer, const size_t len, const sensor_t *sensor)
{
  size_t written;

  /* Clear the buffer */
  memset(buffer, 0x00, len);

  written = snprintf(buffer, len, "%d,%s,%d,%d,%f,%f,%f%s",
        (int)sensor->sensor_id,
        sensor->name,
        (int)sensor->type,
        (int)sensor->version,
        sensor->max_value,
        sensor->min_value,
        sensor->resolution,
        CFG_PRINTF_NEWLINE);

  return written;
}

/**************************************************************************/
/*!
    @brief  Places the sensor event in a text buffer for data logging
            purposes, with the following comma-separated format:
            SENSOR ID, SENSOR TYPE, TIMESTAMP (ms), D0, D1, D2, D3

    @code

    // Read 1000 samples from an accelerometer and log them to an SD card
    // using 'drivers/storage/logger.c' (CFG_SDCARD must be enabled)

    err_t error;
    sensors_event_t event;
    char buffer[128];
    size_t len;
    volatile size_t count = 0;

    // Initialise the accelerometer
    error = lsm303accelInit();

    if (!error)
    {
      // Initialise the logger using the file 'sensors.txt'
      error = loggerInit("sensors.txt", LOGGER_FILEACTION_ALWAYSCREATE);

      if (!error)
      {
        // Read 1000 sensor events and log them to the SD card
        while(count < 1000)
        {
          // Increment the sample counter
          count++;

          // Get fresh sensor data to log
          error = lsm303accelGetSensorEvent(&event);

          // Try to log the data to disk
          if (!error)
          {
            // Fill 'buffer' with the data, returning the string length
            len = sensorsLogSensorsEvent(buffer, 128, &event);

            // Write the buffer out to the log file
            error = loggerWrite(buffer, len);
            if (error)
            {
              // Something went wrong writing to the file ...
              // This will cause us to exit the loop, and the file
              // will be closed further down
              count = 1000;
            }
          }
        }
      }

      // Make sure the file is closed and we unmount the drive
      loggerClose();
    }

    @endcode
*/
/**************************************************************************/
size_t sensorsLogSensorsEvent(char *buffer, const size_t len,
    const sensors_event_t *event)
{
  size_t written;

  /* Clear the buffer */
  memset(buffer, 0x00, len);

  written = snprintf(buffer, len, "%d,%d,%d,%f,%f,%f,%f%s",
    (int)event->sensor_id,
    (int)event->type,
    (int)event->timestamp,
    event->data[0],
    event->data[1],
    event->data[2],
    event->data[3],
    CFG_PRINTF_NEWLINE);

  return written;
}
