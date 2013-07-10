/**************************************************************************/
/*!
    @file pn532_gpio.h

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
#ifndef __PN532_GPIO_H__
#define __PN532_GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

#define PN532_GPIO_RESPONSELENGTH           (16)
#define PN532_GPIO_VALIDATIONBIT            (0x80)

typedef enum pn532_gpio_e
{
  PN532_GPIO_P30 = 0,
  PN532_GPIO_P31,
  PN532_GPIO_P32,
  PN532_GPIO_P33,
  PN532_GPIO_P34,
  PN532_GPIO_P35
} pn532_gpio_t;

pn532_error_t pn532_gpio_WriteGPIO (uint8_t pinState);
pn532_error_t pn532_gpio_ReadGPIO (uint8_t * pinState);

#ifdef __cplusplus
}
#endif 

#endif
