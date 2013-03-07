/**************************************************************************/
/*!
    @defgroup CLI Command Line Interface

    @brief    Easily extensible command-line interface that can handle
	          data from UART, USB CDC, or a variety of other IO sources.
*/
/**************************************************************************/

/**************************************************************************/
/*! 
    @file     cli.h
    @author   K. Townsend (microBuilder.eu)

	@brief    Core functions called to process CLI data and route request
	          to the appropriate command handler.
    @ingroup  CLI

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2012, K. Townsend
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

#ifndef __CLI_H__ 
#define __CLI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

typedef struct
{
  char *command;
  uint8_t minArgs;
  uint8_t maxArgs;
  uint8_t hidden;
  void (*func)(uint8_t argc, char **argv);
  const char *description;
  const char *parameters;
} cli_t;

void cliPoll(void);
void cliRx(uint8_t c);
void cliParse(char *cmd);
void cliInit(void);
void cliReadLine(uint8_t *str, uint16_t *strLen);

#ifdef __cplusplus
}
#endif 

#endif
