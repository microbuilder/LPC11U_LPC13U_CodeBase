/**************************************************************************/
/*!
    @file     ht16k33.h
    @author   K. Townsend (microBuilder.eu)

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
#ifndef __JT16K33_H__
#define __HT16K33_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

/*=========================================================================
    I2C Address
    ---------------------------------------------------------------------*/
    #define HT16K33_I2C_ADDRESS             (0xE0)    // = 0111 000x
    #define HT16K33_I2C_READWRITE           (0x01)
/*=========================================================================*/

// Registers
enum
{
  HT16K33_REGISTER_DISPLAY_SETUP        = 0x80,
  HT16K33_REGISTER_SYSTEM_SETUP         = 0x20,
  HT16K33_REGISTER_DIMMING              = 0xE0
};

// Blink Rate
typedef enum
{
  HT16K33_BLINKRATE_OFF       = 0x00,
  HT16K33_BLINKRATE_2HZ       = 0x01,
  HT16K33_BLINKRATE_1HZ       = 0x02,
  HT16K33_BLINKRATE_HALFHZ    = 0x03
} ht16k33BlinkRate_t;

// Hexadecimal number table for 7-segment displays
static const uint8_t _ht16k33_numbertable[] =
{
  0x3F, /* 0 */
  0x06, /* 1 */
  0x5B, /* 2 */
  0x4F, /* 3 */
  0x66, /* 4 */
  0x6D, /* 5 */
  0x7D, /* 6 */
  0x07, /* 7 */
  0x7F, /* 8 */
  0x6F, /* 9 */
  0x77, /* A */
  0x7C, /* B */
  0x39, /* C */
  0x5E, /* D */
  0x79, /* E */
  0x71, /* F */
};

// Function prototypes
err_t ht16k33Init ( void );
err_t ht16k33SetBrightness ( uint8_t brightness );
err_t ht16k33SetBlinkRate ( ht16k33BlinkRate_t blinkRate );
err_t ht16k33WriteDisplay ( void );
void    ht16k33Clear ( void );

#ifdef __cplusplus
}
#endif 

#endif
