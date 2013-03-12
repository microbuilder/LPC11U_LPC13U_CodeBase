/**************************************************************************/
/*!
    @file     ansi.h
    @author   K. Townsend (microBuilder.eu)

    @brief    ANSI codes for text-formating in the CLI
    @ingroup  CLI

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2012 K. Townsend
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

#ifndef __ANSI_H__
#define __ANSI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

#define ANSI_GOTOXY(x, y) do { printf("%c[%d;%df",0x1B,y,x); } while(0)

// See: http://www.inwap.com/pdp10/ansicode.txt

/* Ex: printf(ANSICODES_GRAPHICS_STYLE_BOLD
               ANSICODES_GRAPHICS_FORECOLOR_BLACK
               ANSICODES_GRAPHICS_STYLE_NEGATIVE
               "Bold Black Inverted Text"
               ANSICODES_GRAPHICS_CLEARALL "\r\n"); */


#define ANSICODES_GRAPHICS_CLEARALL               "\33[0m"
#define ANSICODES_GRAPHICS_STYLE_BOLD             "\33[1m"
#define ANSICODES_GRAPHICS_STYLE_DIM              "\33[2m"
#define ANSICODES_GRAPHICS_STYLE_ITALIC           "\33[3m"
#define ANSICODES_GRAPHICS_STYLE_UNDERSCORE       "\33[4m"
#define ANSICODES_GRAPHICS_STYLE_SLOWBLINK        "\33[5m"
#define ANSICODES_GRAPHICS_STYLE_FASTBLINK        "\33[6m"
#define ANSICODES_GRAPHICS_STYLE_NEGATIVE         "\33[7m"
#define ANSICODES_GRAPHICS_STYLE_CONCEALED        "\33[8m"   // Do not display character
#define ANSICODES_GRAPHICS_STYLE_CANCELBOLDDIM    "\33[22m"
#define ANSICODES_GRAPHICS_STYLE_CANCELUNDERLINE  "\33[24m"
#define ANSICODES_GRAPHICS_STYLE_CANCELBLINK      "\33[25m"
#define ANSICODES_GRAPHICS_STYLE_CANCELNEGATIVE   "\33[27m"
#define ANSICODES_GRAPHICS_FORECOLOR_BLACK        "\33[30m"
#define ANSICODES_GRAPHICS_FORECOLOR_RED          "\33[31m"
#define ANSICODES_GRAPHICS_FORECOLOR_GREEN        "\33[32m"
#define ANSICODES_GRAPHICS_FORECOLOR_YELLOW       "\33[33m"
#define ANSICODES_GRAPHICS_FORECOLOR_BLUE         "\33[34m"
#define ANSICODES_GRAPHICS_FORECOLOR_MAGENTA      "\33[35m"
#define ANSICODES_GRAPHICS_FORECOLOR_CYAN         "\33[36m"
#define ANSICODES_GRAPHICS_FORECOLOR_WHITE        "\33[37m"
#define ANSICODES_GRAPHICS_BACKCOLOR_BLACK        "\33[40m"
#define ANSICODES_GRAPHICS_BACKCOLOR_RED          "\33[41m"
#define ANSICODES_GRAPHICS_BACKCOLOR_GREEN        "\33[42m"
#define ANSICODES_GRAPHICS_BACKCOLOR_YELLOW       "\33[43m"
#define ANSICODES_GRAPHICS_BACKCOLOR_BLUE         "\33[44m"
#define ANSICODES_GRAPHICS_BACKCOLOR_MAGENTA      "\33[45m"
#define ANSICODES_GRAPHICS_BACKCOLOR_CYAN         "\33[46m"
#define ANSICODES_GRAPHICS_BACKCOLOR_WHITE        "\33[47m"

#ifdef __cplusplus
}
#endif

#endif
