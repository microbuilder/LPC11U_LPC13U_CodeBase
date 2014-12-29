/**************************************************************************/
/*!
    @file     ht16k33.c
    @author   K. Townsend (microBuilder.eu)

    @section DESCRIPTION

    I2C Driver for the HT16K33 16*8 LED Driver

    This driver is based on the HT16K33 Library from Limor Fried
    (Adafruit Industries) at:
    https://github.com/adafruit/Adafruit-LED-Backpack-Library

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
#include "projectconfig.h"
#include <string.h>
#include "ht16k33.h"
#include "core/i2c/i2c.h"
#include "core/delay/delay.h"

#define DELAY(mS)     do { delay(mS); } while(0);

extern volatile uint8_t   I2CMasterBuffer[I2C_BUFSIZE];
extern volatile uint8_t   I2CSlaveBuffer[I2C_BUFSIZE];
extern volatile uint32_t  I2CReadLength, I2CWriteLength;

volatile uint16_t _ht16k33_Buffer[8];

/**************************************************************************/
/* Private Methods                                                        */
/**************************************************************************/

/**************************************************************************/
/*!
    @brief  Writes to a register over I2C
*/
/**************************************************************************/
err_t ht16k33WriteRegister (uint8_t reg)
{
  I2CWriteLength = 2;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = HT16K33_I2C_ADDRESS;       // I2C device address
  I2CMasterBuffer[1] = reg;                       // Command register
  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Writes an unsigned 8 bit values over I2C
*/
/**************************************************************************/
err_t ht16k33Write8 (uint8_t reg, uint8_t value)
{
  I2CWriteLength = 3;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = HT16K33_I2C_ADDRESS;       // I2C device address
  I2CMasterBuffer[1] = reg;                       // Command register
  I2CMasterBuffer[2] = (value);                   // Value to write
  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  return ERROR_NONE;
}

/**************************************************************************/
/* Public Methods                                                         */
/**************************************************************************/

/**************************************************************************/
/*!
    @brief Initialises the HT16K33 LED driver
*/
/**************************************************************************/
err_t ht16k33Init(void)
{
  // Make sure I2C is initialised
  i2cInit(I2CMASTER);

  /* Ping the I2C device first to see if it exists! */
  //ASSERT(!(i2cCheckAddress(HT16K33_I2C_ADDRESS)), ERROR_I2C_DEVICENOTFOUND);
  ASSERT(i2cCheckAddress(HT16K33_I2C_ADDRESS), ERROR_I2C_DEVICENOTFOUND);
  // Turn the oscillator on
  ASSERT_STATUS(ht16k33WriteRegister(HT16K33_REGISTER_SYSTEM_SETUP | 0x01));

  // Turn blink off
  ASSERT_STATUS(ht16k33SetBlinkRate(HT16K33_BLINKRATE_OFF));

  // Set max brightness
  ASSERT_STATUS(ht16k33SetBrightness(15));

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief Sets the display brightness/dimming (0..15)
*/
/**************************************************************************/
err_t ht16k33SetBrightness(uint8_t brightness)
{
  if (brightness > 15) brightness = 15;
  return ht16k33WriteRegister(HT16K33_REGISTER_DIMMING | brightness);
}

/**************************************************************************/
/*!
    @brief Sets the display blink rate
*/
/**************************************************************************/
err_t ht16k33SetBlinkRate(ht16k33BlinkRate_t blinkRate)
{
  if (blinkRate > HT16K33_BLINKRATE_HALFHZ) blinkRate = HT16K33_BLINKRATE_OFF;
  return ht16k33WriteRegister(HT16K33_REGISTER_DISPLAY_SETUP | 0x01 | (blinkRate << 1));
}

/**************************************************************************/
/*!
    @brief Updates the display memory
*/
/**************************************************************************/
err_t ht16k33WriteDisplay(void)
{
  int32_t i;

  // Send 8x16-bits of data
  I2CWriteLength = 18;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = HT16K33_I2C_ADDRESS;       // I2C device address
  I2CMasterBuffer[1] = 0x00;                      // Start at address 0
  for (i = 0; i < 8; i++)                         // Individual characters
  {
    I2CMasterBuffer[(i*2)+2] = _ht16k33_Buffer[i] & 0xFF;
    I2CMasterBuffer[(i*2)+2] = _ht16k33_Buffer[i] >> 8;
  }
  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief Clears the display memory
*/
/**************************************************************************/
void ht16k33Clear(void)
{
  uint8_t i;
  for (i=0; i<8; i++)
  {
    _ht16k33_Buffer[i] =  0x0000;
  }
}

/**************************************************************************/
/*!
    @brief  Displays a hexadecimal value string up to 5 characters in
            length. Assumes a 4-Digit, 7-Segment display w/":" between
            digits 2 and 3.  Will handle "-" and "." chars in addition to
            standard hexadecimal values (0..9, A..F).
            
    @args[in] c   
              Pointer to the string containing the text to display
    @args[in] justification
              0 = right justified, 1 = left justified
              
    @author Robert Davidson

    @section EXAMPLE
    @code
    
    static uint8_t sbuff[5];
    float x;
    int i;
    
    ht16k33Init();
    ht16k33Clear();
    ht16k33WriteDisplay();

    // Make sure you're using Redlib(nohost) for this in LPCXpresso
    // so that sprintf is handled by /src/core/libc/stdio.c

	// Cycle through the range of -10.0 to 200.0
    for(i=-100; i<=2000; i++)
    {
      x = i / 10.0;
      sprintf(sbuff, "%5f", x);
      ht16k33LoadString7Seg4Digit(sbuff, 0);
      ht16k33WriteDisplay();
      delay(100);
    }
    
    @endcode
*/
/**************************************************************************/
void ht16k33LoadString7Seg4Digit(char *s, uint8_t justification)
{
  char buf[5];
  uint8_t i=0, j=0, k=0, numDigits=0, length=0;

  length = strlen(s);

  if(length > 5) // only fill _ht16k33_buffer with 5 chars
  {
    length = 5;
  }

  strncpy(buf, s, length);

  for (i = 0; i < length; i++)
  {
    // compute 7-segment Hex value of each char using gfedcba encoding
    // contained in _ht16k33_numbertable[].

    if (buf[i] >= 0x30 && buf[i] <= 0x39) /* for digits 0 - 9 */
    {
      j = buf[i] - 0x30; // compute index into table
      _ht16k33_Buffer[k] = _ht16k33_numbertable[j] * 0x100;  // shift to upper byte
      k++; // track number of char to hex conversions
    }
    else if(buf[i] == 0x2E) /* char is a decimal point,  "." */
    {
      // In strings, the "." is just another char.  However on 7-seg
      // display, each digit contains a decimal point that can be
      // turned on.  Thus a 4 digit number with a decimal point is 5 chars
      // in string but can be displayed on a 4 digit 7 segment display.

      if(k != 3)
      {
        // this display has a ":" that looks like a 5th digit to the
        // ht16k33 so have to take that into account.  It's the 3rd
        // digit out of 5 from the point of view of the ht16k33.
        // OR'ing 0x80000 with the 7-seg hex value (shifted to upper
        // byte in ht16t33 array) turns on the decimal point.
        //
        // decimal point goes with previous digit on 7 seg

        _ht16k33_Buffer[k-1] = _ht16k33_Buffer[k-1] | 0x8000;
      }
      else
      {
        // decimal point is third char, goes with 2nd digit
        // (0-offset array)
        _ht16k33_Buffer[k-2] = _ht16k33_Buffer[k-2] | 0x8000;
      }
    }
    else if (buf[i] == 0x2D)
    {
      _ht16k33_Buffer[k] = 0x4000;  // char is minus sign
      k++;
    }
    else if (buf[i] == 0x2B)  // char is plus sign, just drop it
    {
      _ht16k33_Buffer[k] = 0x0000;
      k++;
    }

    if(k == 2)
    {
      // always turn ":" off (unless it's a clock!)

      _ht16k33_Buffer[k] = 0x0000;
      k++;
    }
  }

  if(justification == 0)
  {
    numDigits = k;

    if (numDigits == 4)
    {
      _ht16k33_Buffer[4] = _ht16k33_Buffer[3];
      _ht16k33_Buffer[3] = _ht16k33_Buffer[1];
      _ht16k33_Buffer[1] = _ht16k33_Buffer[0];
      _ht16k33_Buffer[2] = 0x0000;
      _ht16k33_Buffer[0] = 0x0000;
    }

    if (numDigits == 3)
    {
      _ht16k33_Buffer[4] = _ht16k33_Buffer[1];
      _ht16k33_Buffer[3] = _ht16k33_Buffer[0];
      _ht16k33_Buffer[0] = 0x0000;
      _ht16k33_Buffer[1] = 0x0000;
      _ht16k33_Buffer[2] = 0x0000;
    }

    if (numDigits == 2)
    {
      _ht16k33_Buffer[4] = _ht16k33_Buffer[0];
      _ht16k33_Buffer[3] = 0x0000;
      _ht16k33_Buffer[0] = 0x0000;
      _ht16k33_Buffer[1] = 0x0000;
      _ht16k33_Buffer[2] = 0x0000;
    }
  }
}
