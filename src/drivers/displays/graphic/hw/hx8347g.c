/**************************************************************************/
/*! 
    @file     hx8347g.c
    @author   K. Townsend (microBuilder.eu)

    @section  DESCRIPTION

    Driver for HX8347G 240x320 pixel TFT LCD displays.
    
    This driver uses an 8-bit interface and a 16-bit RGB565 colour palette.

    @section  LICENSE

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
#include "hx8347g.h"
#include "core/delay/delay.h"
#include "core/gpio/gpio.h"
// #include "drivers/displays/graphic/touchscreen.h"

static volatile lcdOrientation_t lcdOrientation = LCD_ORIENTATION_PORTRAIT;

// Screen/Driver Properties
static lcdProperties_t hx8347gProperties = {  240,      // Screen width
                                              320,      // Screen height
                                              true,     // Has touchscreen?
                                              false,    // Allows orientation changes?
                                              false,    // Supports HW scrolling?
                                              true,     // Driver includes fast horizontal line function?
                                              false };  // Driver includes fast vertical line function?

// Initialisation sequence (Ugly here but saves a bit of code space handled like this)
static const uint8_t HX8347G_InitSequence[] = {
  HX8347G_CMD_CYCLECONTROL2,            0x89,
  HX8347G_CMD_FRAMERATECONTROL1,        0x8F,
  HX8347G_CMD_FRAMERATECONTROL3,        0x02,
  0xE2,                                 0x00,
  HX8347G_CMD_POWERSAVING1,             0x01,
  HX8347G_CMD_POWERSAVING2,             0x10,
  HX8347G_CMD_POWERSAVING3,             0x01,
  HX8347G_CMD_POWERSAVING4,             0x10,
  HX8347G_CMD_SOURCEOP_CONTROLNORMAL,   0x70,
  0xF2,                                 0x00,
  0xEA,                                 0x00,
  0xEB,                                 0x20,
  0xEC,                                 0x3C,
  0xED,                                 0xC8,
  HX8347G_CMD_SOURCEOP_CONTROLIDLE,     0x38,
  0xF1,                                 0x01,
  // skip gamma, do later
  HX8347G_CMD_POWERCONTROL2,            0x1B, // was 0x1A
  HX8347G_CMD_POWERCONTROL1,            0x02,
  HX8347G_CMD_VCOMCONTROL2,             0x65, // was 0x61
  HX8347G_CMD_VCOMCONTROL3,             0x5C,
  HX8347G_CMD_VCOMCONTROL1,             0x62,
  HX8347G_CMD_OSCCONTROL2,              0x36,
  HX8347G_CMD_OSCCONTROL1,              0x01,  // OSC_EN = 1
  HX8347G_CMD_POWERCONTROL6,            0x88, 
  HX8347G_INIT_DELAY,                   5,
  HX8347G_CMD_POWERCONTROL6,            0x80,
  HX8347G_INIT_DELAY,                   5,
  HX8347G_CMD_POWERCONTROL6,            0x90,
  HX8347G_INIT_DELAY,                   5,
  HX8347G_CMD_POWERCONTROL6,            0xD4,
  HX8347G_INIT_DELAY,                   5,
  HX8347G_CMD_COLMOD,                   0x05,   // 0x05 = 16-bit RGB565
  HX8347G_CMD_PANELCHARACTERISTICS,     0x09,
  HX8347G_CMD_DISPLAYCONTROL3,          0x38,
  HX8347G_INIT_DELAY,                   40,
  HX8347G_CMD_DISPLAYCONTROL3,          0x3C,   // Turn the display on (GON = 1, DTE = 1, D = 1100)
  HX8347G_CMD_COLADDRSTART2,            0x00,
  HX8347G_CMD_COLADDRSTART1,            0x00,
  HX8347G_CMD_COLADDREND2,              0x00,
  HX8347G_CMD_COLADDREND1,              0xEF,
  HX8347G_CMD_ROWADDRSTART2,            0x00,
  HX8347G_CMD_ROWADDRSTART1,            0x00,
  HX8347G_CMD_ROWADDREND2,              0x01,
  HX8347G_CMD_ROWADDREND1,              0x3F,
};

/*************************************************/
/* Private Methods                               */
/*************************************************/
void hx8347gDelay(unsigned int t)
{
  unsigned char t1;
  while(t--)
  for ( t1=10; t1 > 0; t1-- )
  {
    ASM("nop");
  }
}

/**************************************************************************/
/*! 
    @brief  Sends an 8-bit command + 8-bits data
*/
/**************************************************************************/
void hx8347gWriteRegister(uint8_t command, uint8_t data)
{
  // Write command
  CLR_CS_CD_SET_RD_WR;
  // This won't work since it will only set the 1 bits and leave 0's as is
  LPC_GPIO->SET[HX8347G_DATA_PORT] = (command & 0xFF) << HX8347G_DATA_OFFSET;
  CLR_WR;
  SET_WR;

  // Write data
  SET_CD;
  // CLR_CS_SET_CD_RD_WR;
  // This won't work since it will only set the 1 bits and leave 0's as is
  LPC_GPIO->SET[HX8347G_DATA_PORT] = (data & 0xFF) << HX8347G_DATA_OFFSET;
  CLR_WR;
  SET_WR;
}

/**************************************************************************/
/*! 
    @brief  Sends an 8-bit command
*/
/**************************************************************************/
void hx8347gWriteCommand(const uint8_t command)
{
  // Send command
  CLR_CS_CD_SET_RD_WR;
  // This won't work since it will only set the 1 bits and leave 0's as is
  LPC_GPIO->SET[HX8347G_DATA_PORT] = (command & 0xFF) << HX8347G_DATA_OFFSET;
  CLR_WR;
  SET_WR;
}

/**************************************************************************/
/*! 
    @brief  Sends 16-bits of data
*/
/**************************************************************************/
void hx8347gWriteData(const uint16_t data)
{
  // Send data
  CLR_CS_SET_CD_RD_WR;
  // This won't work since it will only set the 1 bits and leave 0's as is
  LPC_GPIO->SET[HX8347G_DATA_PORT] = (data >> 8) << HX8347G_DATA_OFFSET;
  CLR_WR;
  SET_WR;
  // This won't work since it will only set the 1 bits and leave 0's as is
  LPC_GPIO->SET[HX8347G_DATA_PORT] = (data & 0xFF) << HX8347G_DATA_OFFSET;
  CLR_WR;
  SET_WR;
}

/**************************************************************************/
/*! 
    @brief  Reads the results from an 8-bit command
*/
/**************************************************************************/
uint16_t hx8347gReadRegister(uint8_t command)
{
  uint16_t d = 0;

  // Send command
  CLR_CS_CD_SET_RD_WR;
  // This won't work since it will only set the 1 bits and leave 0's as is
  LPC_GPIO->SET[HX8347G_DATA_PORT] = (command & 0xFF) << HX8347G_DATA_OFFSET;
  CLR_WR;
  SET_WR;

  // Set pins to input
  HX8347G_GPIO2DATA_SETINPUT;

  // Read results
  SET_CD_RD_WR;  
  CLR_RD;
  hx8347gDelay(10);
  // This won't work since it will only set the 1 bits and leave 0's as is
  d = (((LPC_GPIO->SET[HX8347G_DATA_PORT]) & HX8347G_DATA_MASK) >> HX8347G_DATA_OFFSET);
  SET_RD;
  SET_CS;

  // Set pins to output
  HX8347G_GPIO2DATA_SETOUTPUT;

  return d;
}

/**************************************************************************/
/*! 
    @brief  Reads 16-bits of data from the current pixel location
*/
/**************************************************************************/
uint16_t hx8347gReadData(void)
{
  uint8_t high, low;
  high = low = 0;

  CLR_CS_SET_CD_RD_WR;

  // Set pins to input
  HX8347G_GPIO2DATA_SETINPUT;
  CLR_RD;
  hx8347gDelay(10);
  // This won't work since it will only set the 1 bits and leave 0's as is
  high = ((LPC_GPIO->SET[HX8347G_DATA_PORT]) & HX8347G_DATA_MASK);
  high >>= HX8347G_DATA_OFFSET;
  printf("high: 0x%02X\r\n", high);
  SET_RD;
  CLR_RD;
  hx8347gDelay(10);
  // This won't work since it will only set the 1 bits and leave 0's as is
  low = ((LPC_GPIO->SET[HX8347G_DATA_PORT]) & HX8347G_DATA_MASK);
  low >>= HX8347G_DATA_OFFSET;
  printf("low: 0x%02X\r\n", low);
  SET_RD;
  SET_CS;
  HX8347G_GPIO2DATA_SETOUTPUT;

  return (uint16_t)((high << 8) | (low));
}

/**************************************************************************/
/*! 
    @brief  Sets the cursor to the specified X/Y position
*/
/**************************************************************************/
void hx8347gSetCursor(const uint16_t x, const uint16_t y)
{
  hx8347gWriteRegister(HX8347G_CMD_COLADDRSTART2, x>>8);
  hx8347gWriteRegister(HX8347G_CMD_COLADDRSTART1, x);
  hx8347gWriteRegister(HX8347G_CMD_ROWADDRSTART2, y>>8);
  hx8347gWriteRegister(HX8347G_CMD_ROWADDRSTART1, y);
}

/**************************************************************************/
/*! 
    @brief  Sends the initialisation sequence to the display controller
*/
/**************************************************************************/
void hx8347gInitDisplay(void)
{
  uint8_t i, a, d;

  // Clear data line
  LPC_GPIO->SET[HX8347G_DATA_PORT] &= ~HX8347G_DATA_MASK;
    
  SET_RD;
  SET_WR;
  SET_CS;
  SET_CD;

  // Reset display
  CLR_RESET;
  delay(10);
  SET_RESET;
  delay(500);

  // Send the init sequence
  for (i = 0; i < sizeof(HX8347G_InitSequence) / 2; i++) 
  {
    a = HX8347G_InitSequence[i*2];
    d = HX8347G_InitSequence[i*2 + 1];

    if (a == HX8347G_INIT_DELAY)
    {
      hx8347gDelay(1000);
    } 
    else 
    {
      hx8347gWriteRegister(a, d);
    }
  }
}

/**************************************************************************/
/*! 
    @brief  Sets the window confines
*/
/**************************************************************************/
void hx8347gSetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  // ToDo
}

/*************************************************/
/* Public Methods                                */
/*************************************************/

/**************************************************************************/
/*! 
    @brief  Configures any pins or HW and initialises the LCD controller
*/
/**************************************************************************/
void lcdInit(void)
{
  // Set control line pins to output
  LPC_GPIO->DIR[HX8347G_CONTROL_PORT] |=  (1 << HX8347G_CS_PIN);
  LPC_GPIO->DIR[HX8347G_CONTROL_PORT] |=  (1 << HX8347G_CD_PIN);
  LPC_GPIO->DIR[HX8347G_CONTROL_PORT] |=  (1 << HX8347G_WR_PIN);
  LPC_GPIO->DIR[HX8347G_CONTROL_PORT] |=  (1 << HX8347G_RD_PIN);
  
  // Set data port pins to output
  LPC_GPIO->DIR[HX8347G_DATA_PORT] |=  (1 << HX8347G_DATA_MASK);

  // Set backlight pin to output and turn it on
  LPC_GPIO->DIR[HX8347G_BL_PORT] |=  (1 << HX8347G_BL_PIN);
  lcdBacklight(TRUE);

  // Set reset pin to output
  LPC_GPIO->DIR[HX8347G_RES_PORT] |=  (1 << HX8347G_RES_PIN);
  // Low then high to reset
  CLR_RESET;
  delay(50);
  SET_RESET;

  // Initialize the display
  hx8347gInitDisplay();

  delay(50);

  // Set lcd to default orientation
  // lcdSetOrientation(lcdOrientation);

  // Fill screen
  lcdFillRGB(COLOR_BLUE);
  
  // Initialise the touch screen (and calibrate if necessary)
  // tsInit();
}

/**************************************************************************/
/*! 
    @brief  Enables or disables the LCD backlight
*/
/**************************************************************************/
void lcdBacklight(bool state)
{
  // Set the backlight
  if (state)
  {
    SET_BL;
  }
  else
  {
    CLR_BL;
  }
}

/**************************************************************************/
/*! 
    @brief  Renders a simple test pattern on the LCD
*/
/**************************************************************************/
void lcdTest(void)
{
}

/**************************************************************************/
/*! 
    @brief  Fills the LCD with the specified 16-bit color
*/
/**************************************************************************/
void lcdFillRGB(uint16_t color)
{
  uint32_t i;
  i = lcdGetWidth() * lcdGetHeight();

  hx8347gSetCursor(0,0);
  hx8347gWriteCommand(HX8347G_CMD_SRAMWRITECONTROL);

  // Fill screen
  while (i--) 
  {
    hx8347gWriteData(color);
  }
}

/**************************************************************************/
/*! 
    @brief  Draws a single pixel at the specified X/Y location
*/
/**************************************************************************/
void lcdDrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
  if ((x >= lcdGetWidth()) || (y >= lcdGetHeight())) return;

  hx8347gSetCursor(x,y);
  hx8347gWriteCommand(HX8347G_CMD_SRAMWRITECONTROL);
  hx8347gWriteData(color);
}

/**************************************************************************/
/*! 
    @brief  Draws an array of consecutive RGB565 pixels (much
            faster than addressing each pixel individually)
*/
/**************************************************************************/
void lcdDrawPixels(uint16_t x, uint16_t y, uint16_t *data, uint32_t len)
{
}

/**************************************************************************/
/*! 
    @brief  Optimised routine to draw a horizontal line faster than
            setting individual pixels
*/
/**************************************************************************/
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

  hx8347gSetCursor(x0, y);
  hx8347gWriteCommand(HX8347G_CMD_SRAMWRITECONTROL);
  // Draw line
  for (pixels = 0; pixels < x1 - x0 + 1; pixels++)
  {
    hx8347gWriteData(color);
  }
}

/**************************************************************************/
/*! 
    @brief  Optimised routine to draw a vertical line faster than
            setting individual pixels
*/
/**************************************************************************/
void lcdDrawVLine(uint16_t x, uint16_t y0, uint16_t y1, uint16_t color)
{
}

/**************************************************************************/
/*! 
    @brief  Gets the 16-bit color of the pixel at the specified location
*/
/**************************************************************************/
uint16_t lcdGetPixel(uint16_t x, uint16_t y)
{
  /*  The first byte of data apparently has to be ignored (DS section 4.1, p.29):  
      Furthermore, there are two 18-bit bus control registers used to temporarily store the
      data written to or read from the GRAM. When the data is written into the GRAM from
      the MPU, it is first written into the write-data latch and then automatically written into
      the GRAM by internal operation. Data is read through the read-data latch when
      reading from the GRAM. Therefore, the first read data operation is invalid and the
      following read data operations are valid. */

  if ((x >= hx8347gProperties.width) || (y >= hx8347gProperties.height)) return 0;

  hx8347gSetCursor(x,y);
  hx8347gWriteCommand(HX8347G_CMD_SRAMWRITECONTROL);
  return hx8347gReadData();
}

/**************************************************************************/
/*! 
    @brief  Sets the LCD orientation to horizontal and vertical
*/
/**************************************************************************/
void lcdSetOrientation(lcdOrientation_t orientation)
{
}

/**************************************************************************/
/*! 
    @brief  Gets the current screen orientation (horizontal or vertical)
*/
/**************************************************************************/
lcdOrientation_t lcdGetOrientation(void)
{
}

/**************************************************************************/
/*! 
    @brief  Gets the width in pixels of the LCD screen (varies depending
            on the current screen orientation)
*/
/**************************************************************************/
uint16_t lcdGetWidth(void)
{
    return hx8347gProperties.width;
}

/**************************************************************************/
/*! 
    @brief  Gets the height in pixels of the LCD screen (varies depending
            on the current screen orientation)
*/
/**************************************************************************/
uint16_t lcdGetHeight(void)
{
    return hx8347gProperties.height;
}

/**************************************************************************/
/*! 
    @brief  Scrolls the contents of the LCD screen vertically the
            specified number of pixels using a HW optimised routine
*/
/**************************************************************************/
void lcdScroll(int16_t pixels, uint16_t fillColor)
{
}

/**************************************************************************/
/*! 
    @brief  Gets the controller's 16-bit (4 hexdigit) ID
*/
/**************************************************************************/
uint16_t lcdGetControllerID(void)
{
    return 0x8347;
}

/**************************************************************************/
/*! 
    @brief  Returns the LCDs 'lcdProperties_t' that describes the LCDs
            generic capabilities and dimensions
*/
/**************************************************************************/
lcdProperties_t lcdGetProperties(void)
{
    return hx8347gProperties;
}
