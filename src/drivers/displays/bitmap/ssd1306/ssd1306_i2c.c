/**************************************************************************/
/*!
    @file     ssd1306_i2c.c
    @author   K. Townsend (microBuilder.eu)

    @section DESCRIPTION

    I2C Driver for 128x64 OLED display based on the SSD1306 controller.

    This driver is based on the SSD1306 Library from Limor Fried
    (Adafruit Industries) at: https://github.com/adafruit/SSD1306

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
#include "projectconfig.h"
#include <string.h>
#include "ssd1306_i2c.h"
#include "core/i2c/i2c.h"
#include "core/delay/delay.h"
#include "drivers/displays/smallfonts.h"

#define DELAY(mS)     do { delay(mS); } while(0);

uint8_t _ssd1306buffer[SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT / 8];

extern volatile uint8_t   I2CMasterBuffer[I2C_BUFSIZE];
extern volatile uint8_t   I2CSlaveBuffer[I2C_BUFSIZE];
extern volatile uint32_t  I2CReadLength, I2CWriteLength;

/**************************************************************************/
/* Private Methods                                                        */
/**************************************************************************/

/**************************************************************************/
/*!
    @brief Sends a command via I2C

    @param[in]  byte
                The byte to send
*/
/**************************************************************************/
err_t ssd1306SendCommand(uint8_t byte)
{
  uint8_t control = 0x00;   /* Co = 0, D/C = 0 */

  /* Send the specified bytes */
  I2CWriteLength = 3;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = SSD1306_I2C_ADDRESS;
  I2CMasterBuffer[1] = control;
  I2CMasterBuffer[2] = byte;
  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief Sends data via I2C

    @param  data
            The byte to send
*/
/**************************************************************************/
err_t ssd1306SendData(uint8_t data)
{
  uint8_t control = 0x40;   /* Co = 0, D/C = 1 */

  /* Send the specified bytes */
  I2CWriteLength = 3;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = SSD1306_I2C_ADDRESS;
  I2CMasterBuffer[1] = control;
  I2CMasterBuffer[2] = data;
  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Draws a single graphic character using the supplied font
*/
/**************************************************************************/
void ssd1306DrawChar(uint16_t x, uint16_t y, uint8_t c, struct FONT_DEF font)
{
  uint8_t col;
  uint8_t column[font.u8Width];
  uint16_t xoffset, yoffset;

  /* Make sure we are not exceeding the display limits
     This also gets checked in ssd1306DrawPixel, but if we start
     outside the limits we can avoid some unecessary work at the outset */
  if ((x > SSD1306_LCDWIDTH) || (y > SSD1306_LCDHEIGHT))
    return;

  /* Check if the requested character is available */
  if ((c >= font.u8FirstChar) && (c <= font.u8LastChar))
  {
    /* Retrieve appropriate columns from font data */
    for (col = 0; col < font.u8Width; col++)
    {
      column[col] = font.au8FontTable[((c - font.u8FirstChar) * font.u8Width) + col];    /* Get first column of appropriate character */
    }
  }
  else
  {
    /* Requested character is not available in this font ... send a space instead */
    for (col = 0; col < font.u8Width; col++)
    {
      column[col] = 0xFF;    /* Send solid space */
    }
  }

  /* Render each column */
  for (xoffset = 0; xoffset < font.u8Width; xoffset++)
  {
    for (yoffset = 0; yoffset < (font.u8Height + 1); yoffset++)
    {
      uint8_t bit = 0x00;
      bit = (column[xoffset] << (8 - (yoffset + 1)));   /* Shift current row bit left */
      bit = (bit >> 7);                                 /* Shift current row but right (results in 0x01 for black, and 0x00 for white) */
      if (bit)
      {
        ssd1306DrawPixel(x + xoffset, y + yoffset);
      }
    }
  }
}

/**************************************************************************/
/* Public Methods                                                         */
/**************************************************************************/

/**************************************************************************/
/*!
    @brief Initialises the SSD1306 LCD display
*/
/**************************************************************************/
err_t ssd1306Init(uint8_t vccstate)
{
  /* Make sure I2C is initialised */
  i2cInit(I2CMASTER);

  /* Ping the I2C device first to see if it exists! */
  /* It seems we can't ping it */
  // ASSERT(!(i2cCheckAddress(SSD1306_I2C_ADDRESS)), ERROR_I2C_DEVICENOTFOUND);

  /* I2C Initialisation */
  ssd1306SendCommand(SSD1306_DISPLAYOFF);
  ssd1306SendCommand(SSD1306_SETDISPLAYCLOCKDIV);
  ssd1306SendCommand(0x80);
  ssd1306SendCommand(SSD1306_SETMULTIPLEX);
  #if defined SSD1306_128_64
    ssd1306SendCommand(0x3F);
  #elif defined SSD1306_128_32
    ssd1306SendCommand(0x1F);
  #endif
  ssd1306SendCommand(SSD1306_SETDISPLAYOFFSET);
  ssd1306SendCommand(0x0);
  ssd1306SendCommand(SSD1306_SETSTARTLINE | 0x0);
  ssd1306SendCommand(SSD1306_CHARGEPUMP);
  ssd1306SendCommand(vccstate == SSD1306_EXTERNALVCC ? 0x10 : 0x14);
  ssd1306SendCommand(SSD1306_MEMORYMODE);
  ssd1306SendCommand(0x00);
  ssd1306SendCommand(SSD1306_SEGREMAP | 0x1);
  ssd1306SendCommand(SSD1306_COMSCANDEC);
  ssd1306SendCommand(SSD1306_SETCOMPINS);
  #if defined SSD1306_128_64
    ssd1306SendCommand(0x12);
  #elif defined SSD1306_128_32
    ssd1306SendCommand(0x02);
  #endif
  ssd1306SendCommand(SSD1306_SETCONTRAST);
  ssd1306SendCommand(vccstate == SSD1306_EXTERNALVCC ? 0x9F : 0xCF);
  ssd1306SendCommand(SSD1306_SETPRECHARGE);
  ssd1306SendCommand(vccstate == SSD1306_EXTERNALVCC ? 0x22 : 0xF1);
  ssd1306SendCommand(SSD1306_SETVCOMDETECT);
  ssd1306SendCommand(0x40);
  ssd1306SendCommand(SSD1306_DISPLAYALLON_RESUME);
  ssd1306SendCommand(SSD1306_NORMALDISPLAY);

  /* Enable the OLED panel */
  ASSERT_STATUS(ssd1306SendCommand(SSD1306_DISPLAYON));

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief Draws a single pixel in image buffer

    @param[in]  x
                The x position (0..127)
    @param[in]  y
                The y position (0..63)
*/
/**************************************************************************/
void ssd1306DrawPixel(uint8_t x, uint8_t y)
{
  if ((x >= SSD1306_LCDWIDTH) || (y >= SSD1306_LCDHEIGHT))
    return;

  _ssd1306buffer[x+ (y/8)*SSD1306_LCDWIDTH] |= (1 << y%8);
}

/**************************************************************************/
/*!
    @brief Clears a single pixel in image buffer

    @param[in]  x
                The x position (0..127)
    @param[in]  y
                The y position (0..63)
*/
/**************************************************************************/
void ssd1306ClearPixel(uint8_t x, uint8_t y)
{
  if ((x >= SSD1306_LCDWIDTH) || (y >= SSD1306_LCDHEIGHT))
    return;

  _ssd1306buffer[x+ (y/8)*SSD1306_LCDWIDTH] &= ~(1 << y%8);
}

/**************************************************************************/
/*!
    @brief Gets the value (1 or 0) of the specified pixel from the buffer

    @param[in]  x
                The x position (0..127)
    @param[in]  y
                The y position (0..63)

    @return     1 if the pixel is enabled, 0 if disabled
*/
/**************************************************************************/
uint8_t ssd1306GetPixel(uint8_t x, uint8_t y)
{
  if ((x >= SSD1306_LCDWIDTH) || (y >=SSD1306_LCDHEIGHT)) return 0;
  return _ssd1306buffer[x+ (y/8)*SSD1306_LCDWIDTH] & (1 << y%8) ? 1 : 0;
}

/**************************************************************************/
/*!
    @brief Clears the screen
*/
/**************************************************************************/
void ssd1306ClearScreen()
{
  memset(_ssd1306buffer, 0x00, sizeof(_ssd1306buffer));
}

/**************************************************************************/
/*!
    @brief Renders the contents of the pixel buffer on the LCD
*/
/**************************************************************************/
void ssd1306Refresh(void)
{
  uint16_t i;

  ssd1306SendCommand(SSD1306_SETLOWCOLUMN | 0x0);  // low col = 0
  ssd1306SendCommand(SSD1306_SETHIGHCOLUMN | 0x0);  // hi col = 0
  ssd1306SendCommand(SSD1306_SETSTARTLINE | 0x0); // line #0

  for (i=0; i < sizeof(_ssd1306buffer); i++)
  {
    ssd1306SendData(_ssd1306buffer[i]);
  }
}

/**************************************************************************/
/*!
    @brief  Draws a string using the supplied font data.

    @param[in]  x
                Starting x co-ordinate
    @param[in]  y
                Starting y co-ordinate
    @param[in]  text
                The string to render
    @param[in]  font
                Pointer to the FONT_DEF to use when drawing the string

    @section Example

    @code

    #include "drivers/displays/bitmap/ssd1306/ssd1306_i2c.h"
    #include "drivers/displays/smallfonts.h"

    // Configure the pins and initialise the LCD screen
    ssd1306Init(SSD1306_INTERNALVCC);

    // Render some text on the screen
    ssd1306DrawString(1, 10, "5x8 System", Font_System5x8);
    ssd1306DrawString(1, 20, "7x8 System", Font_System7x8);

    // Refresh the screen to see the results
    ssd1306Refresh();

    @endcode
*/
/**************************************************************************/
void ssd1306DrawString(uint16_t x, uint16_t y, char* text, struct FONT_DEF font)
{
  volatile uint8_t l;
  for (l = 0; l < strlen(text); l++)
  {
    ssd1306DrawChar(x + (l * (font.u8Width + 1)), y, text[l], font);
  }
}

/**************************************************************************/
/*!
    @brief  Shifts the contents of the frame buffer up the specified
            number of pixels

    @param[in]  height
                The number of pixels to shift the frame buffer up, leaving
                a blank space at the bottom of the frame buffer x pixels
                high

    @section Example

    @code

    #include "drivers/displays/bitmap/ssd1306/ssd1306_i2c.h"
    #include "drivers/displays/smallfonts.h"

    // Configure the pins and initialise the LCD screen
    ssd1306Init(SSD1306_INTERNALVCC);

    // Continually write some text, scrolling upward one line each time
    while (1)
    {
      // Shift the buffer up 8 pixels (adjust for font-height)
      ssd1306ShiftFrameBuffer(8);
      // Render some text on the screen with different fonts
      ssd1306DrawString(1, 56, "INSERT TEXT HERE", Font_System5x8);
      // Refresh the screen to see the results
      ssd1306Refresh();
      // Wait a bit before writing the next line
      delay(1000);
    }

    @endcode
*/
/**************************************************************************/
void ssd1306ShiftFrameBuffer( uint8_t height )
{
  uint8_t y, x;

  if (height == 0) return;
  if (height >= SSD1306_LCDHEIGHT)
  {
    /* Clear the entire frame buffer */
    ssd1306ClearScreen();
    return;
  }

  /* This is horribly inefficient, but at least easy to understand
     In a production environment, this should be significantly optimised */

  for (y = 0; y < SSD1306_LCDHEIGHT; y++)
  {
    for (x = 0; x < SSD1306_LCDWIDTH; x++)
    {
      if ((SSD1306_LCDHEIGHT - 1) - y > height)
      {
        /* Shift height from further ahead in the buffer */
        ssd1306GetPixel(x, y + height) ? ssd1306DrawPixel(x, y) : ssd1306ClearPixel(x, y);
      }
      else
      {
        /* Clear the entire line */
        ssd1306ClearPixel(x, y);
      }
    }
  }
}
