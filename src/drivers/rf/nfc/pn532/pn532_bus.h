/**************************************************************************/
/*!
    @file pn532_bus.h

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
#ifndef __PN532_BUS_H__
#define __PN532_BUS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"
#include "pn532.h"

#if defined(CFG_ENABLE_UART)
  // #define PN532_BUS_UART
#endif

#if defined(CFG_ENABLE_I2C)
  #define PN532_BUS_I2C
#endif

#if defined PN532_BUS_UART && defined PN532_BUS_I2C
  #error "Only one target can be defined for the PN532 (PN532_BUS_I2C or PN532_BUS_UART)"
#endif

#define PN532_NORMAL_FRAME__DATA_MAX_LEN      (254)
#define PN532_NORMAL_FRAME__OVERHEAD          (8)
#define PN532_EXTENDED_FRAME__DATA_MAX_LEN    (264)
#define PN532_EXTENDED_FRAME__OVERHEAD        (11)
#define PN532_BUFFER_LEN                      (PN532_EXTENDED_FRAME__DATA_MAX_LEN + PN532_EXTENDED_FRAME__OVERHEAD)

#define PN532_UART_BAUDRATE                   (115200)

#define PN532_I2C_ADDRESS                     (0x48)
#define PN532_I2C_READBIT                     (0x01)
#define PN532_I2C_READYTIMEOUT                (20)    // Max number of attempts to read Ready bit (see UM 5-Nov-2007 Section 6.2.4)

// Generic interface for the different serial buses available on the PN532
err_t       pn532_bus_HWInit(void);
pn532_error_t pn532_bus_SendCommand(const byte_t * pbtData, const size_t szData);
pn532_error_t pn532_bus_ReadResponse(byte_t * pbtResponse, size_t * pszRxLen);
pn532_error_t pn532_bus_Wakeup(void);

#ifdef __cplusplus
}
#endif 

#endif
