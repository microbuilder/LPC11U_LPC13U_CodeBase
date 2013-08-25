/**************************************************************************/
/*!
    @file     ili9340.c
    @author   K. Townsend (microBuilder.eu)

    @section  DESCRIPTION

    Driver for ILI9340 based 240x320 pixel TFT LCD displays.

    This driver uses a HW SPI interface and a 16-bit RGB565 colour palette.

    @section  LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013 Kevin Townsend (microBuilder.eu)
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
#include "ili9340.h"
#include "core/delay/delay.h"
#include "core/gpio/gpio.h"
#include "core/ssp0/ssp0.h"

static lcdOrientation_t lcdOrientation = LCD_ORIENTATION_PORTRAIT;
static lcdProperties_t ili9340Properties = { .width       = 240,
                                             .height      = 320,
                                             .touchscreen = false,
                                             .orientation = false,
                                             .hwscrolling = false,
                                             .fastHLine   = false,
                                             .fastVLine   = false };

/*************************************************/
/* Private Methods                               */
/*************************************************/

/*************************************************/
void ili9340WriteCmd(uint8_t command)
{
  uint8_t buf[1] = { command };
  CLR_DC;
  CLR_CS;
  ssp0Send(buf, 1);
  SET_CS;
}

/*************************************************/
void ili9340WriteData(uint8_t data)
{
  uint8_t buf[1] = { data };
  SET_DC;
  CLR_CS;
  ssp0Send(buf, 1);
  SET_CS;
}

/*************************************************/
void ili9340InitDisplay(void)
{
  ili9340WriteCmd(0xEF);
  ili9340WriteData(0x03);
  ili9340WriteData(0x80);
  ili9340WriteData(0x02);

  ili9340WriteCmd(0xCF);
  ili9340WriteData(0x00);
  ili9340WriteData(0XC1);
  ili9340WriteData(0X30);

  ili9340WriteCmd(0xED);
  ili9340WriteData(0x64);
  ili9340WriteData(0x03);
  ili9340WriteData(0X12);
  ili9340WriteData(0X81);

  ili9340WriteCmd(0xE8);
  ili9340WriteData(0x85);
  ili9340WriteData(0x00);
  ili9340WriteData(0x78);

  ili9340WriteCmd(0xCB);
  ili9340WriteData(0x39);
  ili9340WriteData(0x2C);
  ili9340WriteData(0x00);
  ili9340WriteData(0x34);
  ili9340WriteData(0x02);

  ili9340WriteCmd(0xF7);
  ili9340WriteData(0x20);

  ili9340WriteCmd(0xEA);
  ili9340WriteData(0x00);
  ili9340WriteData(0x00);

  ili9340WriteCmd(ILI9340_PWCTR1);    //Power control
  ili9340WriteData(0x23);   //VRH[5:0]

  ili9340WriteCmd(ILI9340_PWCTR2);    //Power control
  ili9340WriteData(0x10);   //SAP[2:0];BT[3:0]

  ili9340WriteCmd(ILI9340_VMCTR1);    //VCM control
  ili9340WriteData(0x3e); //对比度调节
  ili9340WriteData(0x28);

  ili9340WriteCmd(ILI9340_VMCTR2);    //VCM control2
  ili9340WriteData(0x86);  //--

  ili9340WriteCmd(ILI9340_MADCTL);    // Memory Access Control
  ili9340WriteData(ILI9340_MADCTL_MX | ILI9340_MADCTL_BGR);

  ili9340WriteCmd(ILI9340_PIXFMT);
  ili9340WriteData(0x55);

  ili9340WriteCmd(ILI9340_FRMCTR1);
  ili9340WriteData(0x00);
  ili9340WriteData(0x18);

  ili9340WriteCmd(ILI9340_DFUNCTR);    // Display Function Control
  ili9340WriteData(0x08);
  ili9340WriteData(0x82);
  ili9340WriteData(0x27);

  ili9340WriteCmd(0xF2);    // 3Gamma Function Disable
  ili9340WriteData(0x00);

  ili9340WriteCmd(ILI9340_GAMMASET);    //Gamma curve selected
  ili9340WriteData(0x01);

  ili9340WriteCmd(ILI9340_GMCTRP1);    //Set Gamma
  ili9340WriteData(0x0F);
  ili9340WriteData(0x31);
  ili9340WriteData(0x2B);
  ili9340WriteData(0x0C);
  ili9340WriteData(0x0E);
  ili9340WriteData(0x08);
  ili9340WriteData(0x4E);
  ili9340WriteData(0xF1);
  ili9340WriteData(0x37);
  ili9340WriteData(0x07);
  ili9340WriteData(0x10);
  ili9340WriteData(0x03);
  ili9340WriteData(0x0E);
  ili9340WriteData(0x09);
  ili9340WriteData(0x00);

  ili9340WriteCmd(ILI9340_GMCTRN1);    //Set Gamma
  ili9340WriteData(0x00);
  ili9340WriteData(0x0E);
  ili9340WriteData(0x14);
  ili9340WriteData(0x03);
  ili9340WriteData(0x11);
  ili9340WriteData(0x07);
  ili9340WriteData(0x31);
  ili9340WriteData(0xC1);
  ili9340WriteData(0x48);
  ili9340WriteData(0x08);
  ili9340WriteData(0x0F);
  ili9340WriteData(0x0C);
  ili9340WriteData(0x31);
  ili9340WriteData(0x36);
  ili9340WriteData(0x0F);

  ili9340WriteCmd(ILI9340_SLPOUT);    //Exit Sleep
  delay(120);
  ili9340WriteCmd(ILI9340_DISPON);    //Display on
}

/*************************************************/
static void ili9340SetWindow(uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1)
{
  ili9340WriteCmd(ILI9340_CASET); // Column addr set
  ili9340WriteData(x0 >> 8);
  ili9340WriteData(x0 & 0xFF);     // XSTART
  ili9340WriteData(x1 >> 8);
  ili9340WriteData(x1 & 0xFF);     // XEND

  ili9340WriteCmd(ILI9340_PASET); // Row addr set
  ili9340WriteData(y0>>8);
  ili9340WriteData(y0);     // YSTART
  ili9340WriteData(y1>>8);
  ili9340WriteData(y1);     // YEND

  ili9340WriteCmd(ILI9340_RAMWR); // write to RAM
}

/*************************************************/
/* Public Methods                                */
/*************************************************/

/*************************************************/
void lcdInit(void)
{
  /* Set control pins to output */
  #ifdef ILI9340_USEBL
    LPC_GPIO->DIR[ILI9340_PORT] |=  (1 << ILI9340_BL_PIN);
  #endif
  #ifdef ILI9340_USERESET
    LPC_GPIO->DIR[ILI9340_PORT] |=  (1 << ILI9340_RST_PIN);
  #endif
  LPC_GPIO->DIR[ILI9340_PORT] |=  (1 << ILI9340_CS_PIN);
  LPC_GPIO->DIR[ILI9340_PORT] |=  (1 << ILI9340_DC_PIN);

  /* Set pins low by default (except reset) */
  #ifdef ILI9340_USEBL
    CLR_BL;
  #endif
  #ifdef ILI9340_USERESET
    SET_RST;
  #endif
  SET_CS;
  CLR_DC;

  /* ToDo: Setup SPI0 */
  ssp0Init();
  ssp0ClockFast();

  /* Turn backlight on */
  #ifdef ILI9340_USEBL
    lcdBacklight(TRUE);
  #endif

  /* Reset display */
  #ifdef ILI9340_USERESET
    SET_RST;
    delay(100);
    CLR_RST;
    delay(50);
    SET_RST;
    delay(50);
  #endif

  /* Run LCD init sequence */
  ili9340InitDisplay();
  ili9340SetWindow(0, 0, 239, 319);

  /* Fill black */
  lcdFillRGB(COLOR_RED);
}

/*************************************************/
void lcdBacklight(bool state)
{
  #ifdef ILI9340_USEBL
    if (state)
      SET_BL;
    else
      CLR_BL;
  #endif
}

/*************************************************/
void lcdTest(void)
{
  lcdFillRGB(COLOR_GREEN);
}

/*************************************************/
void lcdFillRGB(uint16_t color)
{
  uint16_t i,j;
  uint8_t buf[2] = { color >> 8, color & 0xFF };
  SET_DC;
  for (i=0;i<240;i++)
  {
    for (j=0;j<320;j++)
    {
      CLR_CS;
      ssp0Send(buf, 2);
      SET_CS;
    }
  }
}

/*************************************************/
void lcdDrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
  uint8_t buf[2] = { color >> 8, color & 0xFF };
  ili9340SetWindow(x,y,x+1,y+1);
  SET_DC;
  CLR_CS;
  ssp0Send(buf, 2);
  SET_CS;
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
}

/*************************************************/
void lcdDrawVLine(uint16_t x0, uint16_t x1, uint16_t y, uint16_t color)
{
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
  return ili9340Properties.width;
}

/*************************************************/
uint16_t lcdGetHeight(void)
{
  return ili9340Properties.height;
}

/*************************************************/
void lcdScroll(int16_t pixels, uint16_t fillColor)
{
  // ToDo
}

/*************************************************/
uint16_t lcdGetControllerID(void)
{
  return 0x9340;
}

/*************************************************/
lcdProperties_t lcdGetProperties(void)
{
  return ili9340Properties;
}
