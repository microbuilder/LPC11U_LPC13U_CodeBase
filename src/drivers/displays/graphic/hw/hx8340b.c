/**************************************************************************/
/*!
    @file     hx8340b.c
    @author   K. Townsend (microBuilder.eu)

    @section  DESCRIPTION

    Driver for HX8340B 176x220 pixel TFT LCD displays.

    This driver uses a bit-banged SPI interface and a 16-bit RGB565
    colour palette.

    @section  LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2012 Kevin Townsend
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
#include "hx8340b.h"
#include "core/delay/delay.h"
#include "core/gpio/gpio.h"

static lcdOrientation_t lcdOrientation = LCD_ORIENTATION_PORTRAIT;
static lcdProperties_t hx8340bProperties = { 176, 220, false, false, false, true, true };

/*************************************************/
/* Private Methods                               */
/*************************************************/

/*************************************************/
void hx8340bWriteCmd(uint8_t command)
{
  CLR_CS;

  CLR_SDI;
  CLR_SCL;
  SET_SCL;

  uint8_t i = 0;
  for (i=0; i<8; i++)
  {
    if (command & 0x80)
    {
      SET_SDI;
    }
    else
    {
      CLR_SDI;
    }
    CLR_SCL;
    __NOP();
    __NOP();
    __NOP();
    command <<= 1;
    SET_SCL;
  }
  SET_CS;
}

/*************************************************/
void hx8340bWriteData(uint8_t data)
{
  CLR_CS;

  SET_SDI;
  CLR_SCL;
  SET_SCL;

  uint8_t i = 0;
  for (i=0; i<8; i++)
  {
    if (data & 0x80)
    {
      SET_SDI;
    }
    else
    {
      CLR_SDI;
    }
    CLR_SCL;
    __NOP();
    __NOP();
    __NOP();
    data <<= 1;
    SET_SCL;
  }
  SET_CS;
}

/*************************************************/
void hx8340bWriteData16(uint16_t data)
{
  hx8340bWriteData((data>>8) & 0xFF);
  hx8340bWriteData(data & 0xFF);
}

/*************************************************/
void hx8340bWriteRegister(uint8_t reg, uint8_t value)
{
  hx8340bWriteCmd(reg);
  hx8340bWriteCmd(value);
}

/*************************************************/
void hx8340bInitDisplay(void)
{
  hx8340bWriteCmd(HX8340B_N_SETEXTCMD);
  hx8340bWriteData(0xFF);
  hx8340bWriteData(0x83);
  hx8340bWriteData(0x40);

  hx8340bWriteCmd(HX8340B_N_SPLOUT);
  delay(100);

  hx8340bWriteCmd(0xCA);                  // Undocumented register?
  hx8340bWriteData(0x70);
  hx8340bWriteData(0x00);
  hx8340bWriteData(0xD9);
  hx8340bWriteData(0x01);
  hx8340bWriteData(0x11);

  hx8340bWriteCmd(0xC9);                  // Undocumented register?
  hx8340bWriteData(0x90);
  hx8340bWriteData(0x49);
  hx8340bWriteData(0x10);
  hx8340bWriteData(0x28);
  hx8340bWriteData(0x28);
  hx8340bWriteData(0x10);
  hx8340bWriteData(0x00);
  hx8340bWriteData(0x06);
  delay(20);

  hx8340bWriteCmd(HX8340B_N_SETGAMMAP);
  hx8340bWriteData(0x60);
  hx8340bWriteData(0x71);
  hx8340bWriteData(0x01);
  hx8340bWriteData(0x0E);
  hx8340bWriteData(0x05);
  hx8340bWriteData(0x02);
  hx8340bWriteData(0x09);
  hx8340bWriteData(0x31);
  hx8340bWriteData(0x0A);

  hx8340bWriteCmd(HX8340B_N_SETGAMMAN);
  hx8340bWriteData(0x67);
  hx8340bWriteData(0x30);
  hx8340bWriteData(0x61);
  hx8340bWriteData(0x17);
  hx8340bWriteData(0x48);
  hx8340bWriteData(0x07);
  hx8340bWriteData(0x05);
  hx8340bWriteData(0x33);
  delay(10);

  hx8340bWriteCmd(HX8340B_N_SETPWCTR5);
  hx8340bWriteData(0x35);
  hx8340bWriteData(0x20);
  hx8340bWriteData(0x45);

  hx8340bWriteCmd(HX8340B_N_SETPWCTR4);
  hx8340bWriteData(0x33);
  hx8340bWriteData(0x25);
  hx8340bWriteData(0x4c);
  delay(10);

  hx8340bWriteCmd(HX8340B_N_COLMOD);      // Color Mode
  hx8340bWriteData(0x05);                 // 0x05 = 16bpp, 0x06 = 18bpp

  hx8340bWriteCmd(HX8340B_N_DISPON);
  delay(10);

  hx8340bWriteCmd(HX8340B_N_CASET);
  hx8340bWriteData(0x00);
  hx8340bWriteData(0x00);
  hx8340bWriteData(0x00);
  hx8340bWriteData(0xaf);                 // 175

  hx8340bWriteCmd(HX8340B_N_PASET);
  hx8340bWriteData(0x00);
  hx8340bWriteData(0x00);
  hx8340bWriteData(0x00);
  hx8340bWriteData(0xdb);                // 219

  hx8340bWriteCmd(HX8340B_N_RAMWR);
}

/*************************************************/
void hx8340bHome(void)
{
   hx8340bWriteCmd(HX8340B_N_CASET);
   hx8340bWriteData(0x00);
   hx8340bWriteData(0x00);
   hx8340bWriteData(0x00);
   hx8340bWriteData(0xaf);
   hx8340bWriteCmd(HX8340B_N_PASET);
   hx8340bWriteData(0x00);
   hx8340bWriteData(0x00);
   hx8340bWriteData(0x00);
   hx8340bWriteData(0xdb);
}

/*************************************************/
static inline void hx8340bSetPosition(uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1)
{
  hx8340bWriteCmd(HX8340B_N_CASET);
  hx8340bWriteData(x0>>8);
  hx8340bWriteData(x0);
  hx8340bWriteData(x1>>8);
  hx8340bWriteData(x1);

  hx8340bWriteCmd(HX8340B_N_PASET);
  hx8340bWriteData(y0>>8);
  hx8340bWriteData(y0);
  hx8340bWriteData(y1>>8);
  hx8340bWriteData(y1);

  hx8340bWriteCmd(HX8340B_N_RAMWR);
}

/*************************************************/
/* Public Methods                                */
/*************************************************/

/*************************************************/
void lcdInit(void)
{
  // Set control pins to output
  LPC_GPIO->DIR[HX8340B_PORT] |=  (1 << HX8340B_SDI_PIN);
  LPC_GPIO->DIR[HX8340B_PORT] |=  (1 << HX8340B_SCL_PIN);
  LPC_GPIO->DIR[HX8340B_PORT] |=  (1 << HX8340B_CS_PIN);
  LPC_GPIO->DIR[HX8340B_PORT] |=  (1 << HX8340B_BL_PIN);
  #ifdef HX8340B_USERESET
  LPC_GPIO->DIR[HX8340B_PORT] |=  (1 << HX8340B_RES_PIN);
  #endif

  // Set pins low by default (except reset)
  CLR_SDI;
  CLR_SCL;
  CLR_CS;
  CLR_BL;
  #ifdef HX8340B_USERESET
    SET_RES;
  #endif

  // Turn backlight on
  lcdBacklight(TRUE);

  // Reset display
  #ifdef HX8340B_USERESET
    SET_RES;
    delay(100);
    CLR_RES;
    delay(50);
    SET_RES;
    delay(50);
  #endif

  // Run LCD init sequence
  hx8340bInitDisplay();

  // Fill black
  lcdFillRGB(COLOR_BLACK);
}

/*************************************************/
void lcdBacklight(bool state)
{
  // Set the backlight
  // Note: Depending on the type of transistor used
  // to control the backlight, you made need to invert
  // the values below
  if (state)
    // CLR_BL;
    SET_BL;
  else
    // SET_BL;
    CLR_BL;
}

/*************************************************/
void lcdTest(void)
{
  lcdFillRGB(COLOR_GREEN);
}

/*************************************************/
void lcdFillRGB(uint16_t color)
{
  uint8_t i,j;
  for (i=0;i<220;i++)
  {
    for (j=0;j<176;j++)
    {
      hx8340bWriteData16(color);
    }
  }
}

/*************************************************/
void lcdDrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
  hx8340bSetPosition(x, y, x+1, y+1);
  hx8340bWriteData16(color);
}

/**************************************************************************/
/*!
    @brief  Draws an array of consecutive RGB565 pixels (much
            faster than addressing each pixel individually)
*/
/**************************************************************************/
void lcdDrawPixels(uint16_t x, uint16_t y, uint16_t *data, uint32_t len)
{
  // ToDo: Optimise this function ... currently only a placeholder
  uint32_t i = 0;
  do
  {
    lcdDrawPixel(x+i, y, data[i]);
    i++;
  } while (i<len);
}

/*************************************************/
void lcdDrawHLine(uint16_t x0, uint16_t x1, uint16_t y, uint16_t color)
{
  // Allows for slightly better performance than setting individual pixels
  uint16_t x, pixels;

  if (x1 < x0)
  {
    // Switch x1 and x0
    x = x1;
    x1 = x0;
    x0 = x;
  }

  // Check limits
  if (x1 >= lcdGetWidth())
  {
    x1 = lcdGetWidth() - 1;
  }
  if (x0 >= lcdGetWidth())
  {
    x0 = lcdGetWidth() - 1;
  }

  hx8340bSetPosition(x0, y, x1, y+1);
  for (pixels = 0; pixels < x1 - x0 + 1; pixels++)
  {
    hx8340bWriteData16(color);
  }
}

/*************************************************/
void lcdDrawVLine(uint16_t x, uint16_t y0, uint16_t y1, uint16_t color)
{
  // Allows for slightly better performance than setting individual pixels
  uint16_t y, pixels;

  if (y1 < y0)
  {
    // Switch y1 and y0
    y = y1;
    y1 = y0;
    y0 = y;
  }

  // Check limits
  if (y1 >= lcdGetHeight())
  {
    y1 = lcdGetHeight() - 1;
  }
  if (y0 >= lcdGetHeight())
  {
    y0 = lcdGetHeight() - 1;
  }

  for (pixels = 0; pixels < y1 - y0 + 1; pixels++)
  {
    hx8340bSetPosition(x, y0+pixels, x+1, y0+pixels+1);
    hx8340bWriteData16(color);
  }
}

/*************************************************/
uint16_t lcdGetPixel(uint16_t x, uint16_t y)
{
  // ToDo
  return 0;
}

/*************************************************/
void lcdSetOrientation(lcdOrientation_t orientation)
{
  // ToDo
}

/*************************************************/
lcdOrientation_t lcdGetOrientation(void)
{
  return lcdOrientation;
}

/*************************************************/
uint16_t lcdGetWidth(void)
{
  return hx8340bProperties.width;
}

/*************************************************/
uint16_t lcdGetHeight(void)
{
  return hx8340bProperties.height;
}

/*************************************************/
void lcdScroll(int16_t pixels, uint16_t fillColor)
{
  // ToDo
}

/*************************************************/
uint16_t lcdGetControllerID(void)
{
  return 0x8340;
}

/*************************************************/
lcdProperties_t lcdGetProperties(void)
{
  return hx8340bProperties;
}
