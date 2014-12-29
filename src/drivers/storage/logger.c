/**************************************************************************/
/*!
    @file     logger.c
    @brief    Logs data to a file
    @author   K. Townsend (microBuilder.eu)

    @section Example

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

#include <string.h>
#include "logger.h"

#define LOGGER_LOCALFILE (0) /* Create a file on the HD via the J-Link */
#define LOGGER_FATFSFILE (1) /* Store a file on the SD card via FatFS  */

// Write local files using crossworks debug library (CW Debug only)
#if LOGGER_LOCALFILE
  #ifdef __CROSSWORKS_ARM
    #include <cross_studio_io.h>
    DEBUG_FILE * loggerLocalFile;
  #endif
#endif

// Write files to SD using FatFS
#if defined CFG_SDCARD && LOGGER_FATFSFILE
  #include "drivers/storage/fatfs/diskio.h"
  #include "drivers/storage/fatfs/ff.h"
  static FATFS Fatfs[1];
  static FIL loggerSDFile;
#endif

char * loggerFName;
static bool loggerInitialised = FALSE;

/**************************************************************************/
/*!
    @brief Initialises a new text file for data logging

    @param  filename  Full path and filename for the log file
    @param  action    LOGGER_FILEACTION_APPEND to create a file if it
                      doesn't already exist or append to an existing file,
                      or LOGGER_FILEACTION_ALWAYSCREATE to always create
                      a new file, overwriting any previously existing file
                      with the same name.

    @note   Possible errors are:

            - ERROR_FATFS_NODISK
            - ERROR_FATFS_INITFAILED
            - ERROR_FATFS_FAILEDTOMOUNTDRIVE
            - ERROR_FATFS_UNABLETOCREATEFILE
            - ERROR_NONE

    @code

    err_t error;
    error = loggerInit("/folder/datalog.csv", LOGGER_FILEACTION_APPEND);

    if (!error)
    {
      // Start logging data with loggerWrite() ...
    }

    // Make sure we close the file to commit any unwritten data!
    loggerClose();

    @endcode
*/
/**************************************************************************/
err_t loggerInit(char *filename, logger_fileaction_t action)
{
  loggerFName = filename;

  #if LOGGER_LOCALFILE
    #ifdef __CROSSWORKS_ARM
      switch (action)
      {
        case (LOGGER_FILEACTION_ALWAYSCREATE):
          // Always create a new file
          loggerLocalFile = debug_fopen(loggerFName, "wt");
          break;
        default:
          // Append data to existing files
          loggerLocalFile = debug_fopen(loggerFName, "at");
          break;
      }
    #endif
  #endif

  #if defined CFG_SDCARD && LOGGER_FATFSFILE
    DSTATUS stat = disk_status(0);

    // Make sure an SD card is present
    if (stat & STA_NODISK)
    {
      return ERROR_FATFS_NODISK;
    }

    // Check if the SD card has already been initialised
    if (stat & STA_NOINIT)
    {
      // Try to initialise it here
      if(disk_initialize(0) & (STA_NOINIT | STA_NODISK))
      {
        return ERROR_FATFS_INITFAILED;
      }
    }

    // SD card successfully initialised
    if (stat == 0)
    {
      BYTE res;
      // Try to mount the fat file system
      res = f_mount(0, &Fatfs[0]);
      if (res != FR_OK)
      {
        return ERROR_FATFS_FAILEDTOMOUNTDRIVE;
      }
      if (res == FR_OK)
      {
        switch (action)
        {
          case (LOGGER_FILEACTION_ALWAYSCREATE):
            // Create a file (overwriting any existing file!)
            if(f_open(&loggerSDFile, loggerFName, FA_READ | FA_WRITE | FA_CREATE_ALWAYS)!=FR_OK)
            {
              return ERROR_FATFS_UNABLETOCREATEFILE;
            }
            break;
          default:
            // Append to an existing file is present, otherwise create one
            if(f_open(&loggerSDFile, loggerFName, FA_READ | FA_WRITE | FA_OPEN_ALWAYS)!=FR_OK)
            {
              return ERROR_FATFS_UNABLETOCREATEFILE;
            }
            // Move to end of the file if we are appending data
            f_lseek(&loggerSDFile, (&loggerSDFile)->fsize);
            break;
        }
      }
    }
  #endif

  loggerInitialised = TRUE;
  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief Appends the supplied buffer to the log file created in
           loggerInit()
*/
/**************************************************************************/
err_t loggerWrite(const uint8_t * buffer, uint32_t len)
{
  if (!loggerInitialised)
  {
    return ERROR_DEVICENOTINITIALISED;
  }

  #if LOGGER_LOCALFILE
    #ifdef __CROSSWORKS_ARM
      int written;
      // Open file for text append
      written = debug_fwrite(buffer, len, 1, loggerLocalFile);
      if (written != len)
      {
        return ERROR_FATFS_WRITEFAILED;
      }
    #endif
  #endif

  #if defined CFG_SDCARD && LOGGER_FATFSFILE
    BYTE res;
    unsigned int bytesWritten;
    DSTATUS stat = disk_status(0);

    // Make sure the SD card is still present
    if(stat & STA_NODISK)
    {
      return ERROR_FATFS_NODISK;
    }

    // Write the data to the log file
    res = f_write(&loggerSDFile, buffer, len, &bytesWritten);
    if (res != FR_OK)
    {
      return ERROR_FATFS_WRITEFAILED;
    }
  #endif

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief Writes an uncommitted data and closes the file
*/
/**************************************************************************/
err_t loggerClose(void)
{
  #if LOGGER_LOCALFILE
    #ifdef __CROSSWORKS_ARM
      debug_fclose(loggerLocalFile);
    #endif
  #endif

  #if defined CFG_SDCARD && LOGGER_FATFSFILE
    f_sync(&loggerSDFile);    // Sync any uncommitted data
    f_close(&loggerSDFile);   // Close the file
    f_mount(0, 0);            // Unmount the file system
  #endif

  return ERROR_NONE;
}
