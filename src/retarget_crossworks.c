/**************************************************************************/
/*!
    @file     retarget_crossworks.c

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

#ifdef __CROSSWORKS_ARM

#include <__cross_studio_io.h>

int _write(int file, char *ptr, int len);
int _read (int file, char *buf, int len);
int _open(const char *name, int flags, int mode);

#define RETARGET_XPAND_ENUM_FILENO(filename, target_putc, target_getc, target_peek) \
  FILENO_##filename,

enum
{
  RETARGET_LOOKUP_TABLE(RETARGET_XPAND_ENUM_FILENO)
  FILENO_COUNT
};

#define RETARGET_XPAND_FILEPOOL(filename, target_putc, target_getc, target_peek) \
  [FILENO_##filename] = { ._file = FILENO_##filename },

static FILE retarget_filepool[FILENO_COUNT] =
{
  RETARGET_LOOKUP_TABLE(RETARGET_XPAND_FILEPOOL)
};

FILE *stdin  = &retarget_filepool[STDIN_FILENO];
FILE *stdout = &retarget_filepool[STDOUT_FILENO];
FILE *stderr = &retarget_filepool[STDERR_FILENO];

/******************************************************************************/
/*!
    @brief write a character to stdout
*/
/******************************************************************************/
int __putchar(int ch)
{
  char byte = (char) ch;
  return _write(STDOUT_FILENO, &byte, 1);
}

/******************************************************************************/
/*!
    @brief read a character from stdin
*/
/******************************************************************************/
int __getchar()
{
  char ch;
  return (EOF == _read(STDIN_FILENO, &ch, 1)) ? EOF : (int) ch;
}

/******************************************************************************/
/*!

*/
/******************************************************************************/
void setbuf(FILE *f, char *b)
{
  // set buffer size for target.
}

/******************************************************************************/
/*!
    @brief  Writes a character ('c') to the specified stream.
*/
/******************************************************************************/
int fputc(int c, FILE *f)
{
  char ch = (char) c;
  return _write(f->_file, &ch, 1);
}

/******************************************************************************/
/*!
    @brief  Writes a character ('c') to the specified stream.
*/
/******************************************************************************/
int putc(int c, FILE *f)
{
  return fputc(c, f);
}

/******************************************************************************/
/*!
    @brief  Writes an array of characters ('s') to the specified stream.
            The function terminates after reaching the terminating null
            character ('\0').
*/
/******************************************************************************/
int fputs(const char *s, FILE *f)
{
  return _write(f->_file, (char*) s, strlen(s));
}

/******************************************************************************/
/*!
    @brief  Prints formatted output to the specified file stream
*/
/******************************************************************************/
int fprintf(FILE *f, const char * fmt, ...)
{
  char buf[256], *p;
  int len;

  va_list ap;
  va_start(ap, fmt);

  len = vsnprintf(buf, sizeof(buf), fmt, ap);
  _write(f->_file, buf, strlen(buf));

  va_end(ap);

  return len;
}

/******************************************************************************/
/*!
    @brief  Reads a character from the specified file stream
*/
/******************************************************************************/
int getc(FILE *f)
{
  char ch;
  return (EOF == _read(f->_file, &ch, 1)) ? EOF : (int) ch;
}

/******************************************************************************/
/*!
    @brief  Opens the named stream 'path' with the access rights specified
            by 'mode'.

    @arg    mode
            Can be equal to one of the following values (where b
            stands for binary):

            r   rb      Open for reading (the file must exists).
                        Starts at beginning.
            w   wb      Opens for writing (creates the file if necessary,
                        deleted content and overwrites file if it exists).
                        Starts at beginning.
            a   ab      Open for appending (creates file if it doesn't exist).
                        Starts and end.
            r+  rb+ r+b Open for reading and writing (the file must exist).
                        Starts at beginning.
            w+  wb+ w+b Open for reading and writing. If file exists delete
                        content and overwrite the file, otherwise create a
                        new empty file.
                        Starts at beginning.
            a+  ab+ a+b Opens for reading and writing (append if file exists)
                        Starts at end.
*/
/******************************************************************************/
FILE *fopen(const char *path, const char *mode)
{
  (void) mode;
  int fileno = _open(path, 0, 0);
  return (fileno < 0) ? NULL : &retarget_filepool[fileno];
}

/******************************************************************************/
/*!
    @brief  Closes the stream specified by file_pointer.

    @return The return value is an integer with the following meaning:

            0 (zero): The stream was closed successfully
            EOF: An error occured
*/
/******************************************************************************/
int fclose(FILE *file_pointer)
{
  return EOF;
}

/******************************************************************************/
/*!
    @brief  Copies 'nmemb' items of data of size 'size' from the named input
            'stream' into an array pointed to by 'ptr'.

    @return The number of items actually read. If 'nmemb' is zero no action
            is taken and the function will return 0.
*/
/******************************************************************************/
size_t fread(void *ptr, size_t size, size_t nmemb, FILE *stream)
{
  int count = _read(stream->_file, (char*) ptr, size*nmemb);
  return (EOF == count) ? 0 : count;
}

/******************************************************************************/
/*!
    @brief  Writes an array of 'count' elements to the current position in
            'stream'.  For each element, it will write 'size' bytes. The
            position indicator of the stream will be advanced by the number
            of bytes written successfully.

    @return The number of elements written successfully. The return value will
            be equal to 'count' if the write completed successfully. In case of
            a write error, the return value will be less than 'count'.
*/
/******************************************************************************/
size_t fwrite(const void *array, size_t size, size_t count, FILE *stream)
{
  return (size_t) _write(stream->_file, (char *) array, size*count);
}

/******************************************************************************/
/*!
    @brief  Flush a single file, or (if file_pointer is NULL) all files.
*/
/******************************************************************************/
int fflush(FILE *file_pointer)
{
  return -1;
}

#endif /* __CROSSWORKS_ARM */
