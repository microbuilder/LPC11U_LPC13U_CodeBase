/**************************************************************************/
/*!
    @file     retarget.c

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2014, Adafruit Industries (adafruit.com)
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
#include "retarget.h"

static char const* const retarget_filename_tbl[] =
{
    RETARGET_LOOKUP_TABLE(RETARGET_XPAND_FILENAME)
};

retarget_func_t const retarget_func_tbl[] =
{
    RETARGET_LOOKUP_TABLE(RETARGET_XPAND_FUNCTABLE)
};

enum
{
  RETARGET_FILECOUNT = sizeof(retarget_func_tbl) / sizeof(retarget_func_t)
};

/******************************************************************************/
/*!
    @brief  Syscall to write data to an FILE target

    @param  file fileno for the target
    @param  buf buffer that stores data
    @param  len number of byte we want to write

    @return number of byte actually written
*/
/******************************************************************************/
int _write(int file, char *ptr, int len)
{
  // unknown target or target does not support output
  if ( !(file < RETARGET_FILECOUNT && retarget_func_tbl[file].write) ) return 0;

  for(int i=0; i<len; i++)
  {
    if ( EOF == retarget_func_tbl[file].write(*ptr++) ) return i;
  }

  return len;
}

/******************************************************************************/
/*!
    @brief  Syscall to get read from an FILE target

    @param  file fileno for the target
    @param  buf buffer to store read data
    @param  len number of byte we want to read

    @return number of byte actually read
*/
/******************************************************************************/
int _read (int file, char *buf, int len)
{
  // unknown target or target does not support output
  if ( !(file < RETARGET_FILECOUNT && retarget_func_tbl[file].read) ) return EOF;
  if (len == 0 || buf == NULL) return EOF;

  int count=0;
  int ch;

  do
  {
    if ( (ch = retarget_func_tbl[file].read()) != EOF )
    {
      *buf++ = (char) ch;
      count++;
    }

  } while( (count < len) && (ch != EOF) );

  return (count == 0) ? EOF : count;
}

/******************************************************************************/
/*!
    @brief    Syscall ultimated called by fopen to get the file descriptor
    (fileno) for the FILE* pointer

    @param    name  filename of the target
    @param    flags file's flag
    @param    mode  openning mode
    @return   Unique integer for a filename (FILENO), a.k.a file descriptor
*/
/******************************************************************************/
int _open(const char *name, int flags, int mode)
{
  (void) flags; (void) mode;

  for(int fileno = FILENO_NONSTD_START; fileno<RETARGET_FILECOUNT; fileno++)
  {
    if ( !strcmp(name, retarget_filename_tbl[fileno]) )
    {
      return fileno;
    }
  }

  return -1;
}

/******************************************************************************/
/*!
    @brief  Get character from the stream without remove it at an offset

    @param  stream target's stream we want to peek
    @param  index relative offset to the head of the received buffer.
*/
/******************************************************************************/
int fpeekAt(FILE *stream, uint16_t index)
{
  int file = stream->_file;
  if ( !(file < RETARGET_FILECOUNT && retarget_func_tbl[file].peekAt) ) return EOF;

  return retarget_func_tbl[file].peekAt(index);
}
