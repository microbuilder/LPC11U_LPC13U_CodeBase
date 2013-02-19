/**************************************************************************/
/*!
    @file     cmd_dbg_memrd.c
    @author   K. Townsend (microBuilder.eu)

    @brief    Reads the contents of memory at a specific address.0
    @ingroup  CLI

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
#include <stdio.h>

#include "projectconfig.h"
#include "cli/cli.h"
#include "cli/commands.h"       // Generic helper functions

/**************************************************************************/
/*!
    Command handler
*/
/**************************************************************************/
void cmd_dbg_memrd(uint8_t argc, char **argv)
{
  uint32_t startAddr;
  int32_t  i, s;
  int32_t  len = 4;
  int32_t  size = 4;

  /* Warning: This command is extremely dangerous and can easily
     lead to a hardfault ... use with care! */

  getNumberU32(argv[0], &startAddr);
  if (argc > 1) getNumber(argv[1], &len);
  if (argc == 3) getNumber(argv[2], &size);

  /* Make sure size is something sane */
  if ((size < 1) || (size > 8))
  {
    printf("%s %d%s", STRING(LOCALISATION_TEXT_Invalid_argument), (int)size, CFG_PRINTF_NEWLINE);
    return;
  }

  s = 0;
  for (i = 0; i<len; i++)
  {
    /* Get the aligned word address (M0 can't do unaligned access!) */
    uint32_t mem = *(uint32_t*)(startAddr+i - ((startAddr+i) % 4));

    s++;
    printf("%02X", (unsigned int)((mem >> (8*(3-((startAddr+i) % 4)))) & 0xFF));
    if (s == size)
    {
      printf(" ");
      s = 0;
    }
  }
}
