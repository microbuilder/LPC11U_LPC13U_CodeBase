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
#include <string.h>
#include "logger.h"
#include "core/delay/delay.h"

#define LOGGER_LOCALFILE (0)
#define LOGGER_FATFSFILE (1)

// Write local files using crossworks debug library (CW Debug only)
#if LOGGER_LOCALFILE
  #include <cross_studio_io.h>
  DEBUG_FILE * loggerLocalFile;
#endif

// Write files to SD using FatFS
#if defined CFG_SDCARD
  #include "drivers/storage/fatfs/diskio.h"
  #include "drivers/storage/fatfs/ff.h"
  static FILINFO Finfo;
  static FATFS Fatfs[1];
  static FIL loggerSDFile;
#endif

char * loggerFName;
static bool loggerInitialised = FALSE;

/**************************************************************************/
/*!

*/
/**************************************************************************/
logger_error_t loggerWrite(const uint8_t * buffer, uint32_t len)
{
    if (!loggerInitialised)
    {
      return LOGGER_ERROR_NOTINITIALISED;
    }

    #if LOGGER_LOCALFILE
      // Open file for text append
      loggerLocalFile = debug_fopen(loggerFName, "at");
      debug_fwrite(buffer, len, 1, loggerLocalFile);
      debug_fclose(loggerLocalFile);
    #endif

    #if defined CFG_SDCARD && LOGGER_FATFSFILE
      // Open file
      if(f_open(&loggerSDFile, loggerFName, FA_READ | FA_WRITE | FA_OPEN_EXISTING)!=FR_OK)
      {
        return LOGGER_ERROR_FATFS_UNABLETOOPENFILE;
      }
      // Move to end of the file to append data
      f_lseek(&loggerSDFile, (&loggerSDFile)->fsize);
      // Write data
      unsigned int bytesWritten;
      f_write(&loggerSDFile, buffer, len, &bytesWritten);
      f_sync(&loggerSDFile);
      f_close(&loggerSDFile);
    #endif

    return LOGGER_OK;
}

/**************************************************************************/
/*!
    @brief Initialises a new libpcap binary file and writes the header
*/
/**************************************************************************/
logger_error_t loggerInit(char *filename)
{
  loggerFName = filename;

  // Create a new file
  #if LOGGER_LOCALFILE
    loggerLocalFile = debug_fopen(loggerFName, "wt");
  #endif
  #if defined CFG_SDCARD && LOGGER_FATFSFILE
    DSTATUS stat;
    stat = disk_initialize(0);
    if (stat & STA_NOINIT)
    {
      return LOGGER_ERROR_FATFS_INITFAILED;
    }
    if (stat & STA_NODISK)
    {
      return LOGGER_ERROR_FATFS_NODISK;
    }
    if (stat == 0)
    {
      // SD card sucessfully initialised
      DSTATUS stat;
      DWORD p2;
      WORD w1;
      BYTE res, b1;
      DIR dir;
      // Try to mount drive
      res = f_mount(0, &Fatfs[0]);
      if (res != FR_OK)
      {
        return LOGGER_ERROR_FATFS_FAILEDTOMOUNTDRIVE;
      }
      if (res == FR_OK)
      {
        // Create a file (overwriting any existing file!)
        if(f_open(&loggerSDFile, loggerFName, FA_READ | FA_WRITE | FA_CREATE_ALWAYS)!=FR_OK)
        {
          return LOGGER_ERROR_FATFS_UNABLETOCREATEFILE;
        }
      }
    }
  #endif

  // Close the file (not a great idea to keep it open permanently)
  #if LOGGER_LOCALFILE
    debug_fclose(loggerLocalFile);
  #endif
  #if defined CFG_SDCARD && LOGGER_FATFSFILE
    f_close(&loggerSDFile);
    // ToDo: This will leave the driver mounted ... when to call "f_mount(0,0)"?
  #endif

  loggerInitialised = TRUE;
  return LOGGER_OK;
}
