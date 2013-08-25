/**************************************************************************/
/*!
    @file     ili9340.h
    @author   K. Townsend (microBuilder.eu)

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
#ifndef __ILI9340_H__
#define __ILI9340_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"
#include "drivers/displays/graphic/lcd.h"

/**************************************************************************
    ILI9340 2.2" LCD - http://www.adafruit.com/products/1480
    -----------------------------------------------------------------------
    Pin   Function        Notes
    ===   ==============  ===============================
      1   BL              GND
      2   SCK             See board config file
      3   MISO            See board config file
      4   MOSI            See board config file
      5   CS
      6   SDCS            See board config file (CFG_SDCARD)
      7   RST
      8   D/C
      9   VIN             5V (Adafruit board has 3V3 VREG)
     10   GND             GND

 **************************************************************************/

/* Comment this line out if BL on the LCD is tied to GND (not PWM or GPIO) */
#define ILI9340_USEBL

/* Comment this line out if RST on the LCD is tied to RST on the MCU */
#define ILI9340_USERESET

/* Control pins */
#define ILI9340_PORT        (1)
#define ILI9340_BL_PIN      (25)
#define ILI9340_CS_PIN      (27)
#define ILI9340_RST_PIN     (28)
#define ILI9340_DC_PIN      (26)

/* Macros for control line state */
#define CLR_BL      do { LPC_GPIO->CLR[ILI9340_PORT] = (1 << ILI9340_BL_PIN); } while(0)
#define SET_BL      do { LPC_GPIO->SET[ILI9340_PORT] = (1 << ILI9340_BL_PIN); } while(0)
#define CLR_CS      do { LPC_GPIO->CLR[ILI9340_PORT] = (1 << ILI9340_CS_PIN); } while(0)
#define SET_CS      do { LPC_GPIO->SET[ILI9340_PORT] = (1 << ILI9340_CS_PIN); } while(0)
#define CLR_RST     do { LPC_GPIO->CLR[ILI9340_PORT] = (1 << ILI9340_RST_PIN); } while(0)
#define SET_RST     do { LPC_GPIO->SET[ILI9340_PORT] = (1 << ILI9340_RST_PIN); } while(0)
#define CLR_DC      do { LPC_GPIO->CLR[ILI9340_PORT] = (1 << ILI9340_DC_PIN); } while(0)
#define SET_DC      do { LPC_GPIO->SET[ILI9340_PORT] = (1 << ILI9340_DC_PIN); } while(0)

/* ILI9340 Commands */
#define ILI9340_NOP         (0x00)
#define ILI9340_SWRESET     (0x01)
#define ILI9340_RDDID       (0x04)
#define ILI9340_RDDST       (0x09)
#define ILI9340_SLPIN       (0x10)
#define ILI9340_SLPOUT      (0x11)
#define ILI9340_PTLON       (0x12)
#define ILI9340_NORON       (0x13)
#define ILI9340_RDMODE      (0x0A)
#define ILI9340_RDMADCTL    (0x0B)
#define ILI9340_RDPIXFMT    (0x0C)
#define ILI9340_RDIMGFMT    (0x0A)
#define ILI9340_RDSELFDIAG  (0x0F)
#define ILI9340_INVOFF      (0x20)
#define ILI9340_INVON       (0x21)
#define ILI9340_GAMMASET    (0x26)
#define ILI9340_DISPOFF     (0x28)
#define ILI9340_DISPON      (0x29)
#define ILI9340_CASET       (0x2A)
#define ILI9340_PASET       (0x2B)
#define ILI9340_RAMWR       (0x2C)
#define ILI9340_RAMRD       (0x2E)
#define ILI9340_PTLAR       (0x30)

#define ILI9340_MADCTL      (0x36)
#define ILI9340_MADCTL_MY   (0x80)
#define ILI9340_MADCTL_MX   (0x40)
#define ILI9340_MADCTL_MV   (0x20)
#define ILI9340_MADCTL_ML   (0x10)
#define ILI9340_MADCTL_RGB  (0x00)
#define ILI9340_MADCTL_BGR  (0x08)
#define ILI9340_MADCTL_MH   (0x04)

#define ILI9340_PIXFMT      (0x3A)
#define ILI9340_FRMCTR1     (0xB1)
#define ILI9340_FRMCTR2     (0xB2)
#define ILI9340_FRMCTR3     (0xB3)
#define ILI9340_INVCTR      (0xB4)
#define ILI9340_DFUNCTR     (0xB6)
#define ILI9340_PWCTR1      (0xC0)
#define ILI9340_PWCTR2      (0xC1)
#define ILI9340_PWCTR3      (0xC2)
#define ILI9340_PWCTR4      (0xC3)
#define ILI9340_PWCTR5      (0xC4)
#define ILI9340_VMCTR1      (0xC5)
#define ILI9340_VMCTR2      (0xC7)
#define ILI9340_RDID1       (0xDA)
#define ILI9340_RDID2       (0xDB)
#define ILI9340_RDID3       (0xDC)
#define ILI9340_RDID4       (0xDD)
#define ILI9340_GMCTRP1     (0xE0)
#define ILI9340_GMCTRN1     (0xE1)

#ifdef __cplusplus
}
#endif

#endif
