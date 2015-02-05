/**************************************************************************/
/*!
    @file     si5351.c
    @author   K. Townsend (Adafruit Industries)

    @brief    Driver for the SI5351 160MHz Clock Gen

    @section  REFERENCES

    Si5351A/B/C Datasheet:
    http://www.silabs.com/Support%20Documents/TechnicalDocs/Si5351.pdf

    Manually Generating an Si5351 Register Map:
    http://www.silabs.com/Support%20Documents/TechnicalDocs/AN619.pdf

    @section  LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2014, Adafruit Industries
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
#include <math.h>
#include "projectconfig.h"
#include "si5351.h"
#include "core/delay/delay.h"

extern volatile uint8_t    I2CMasterBuffer[I2C_BUFSIZE];
extern volatile uint8_t    I2CSlaveBuffer[I2C_BUFSIZE];
extern volatile uint32_t   I2CReadLength, I2CWriteLength;

static si5351Config_t m_si5351Config =
{
  .initialised     = false,
  .crystalFreq     = SI5351_CRYSTAL_FREQ_25MHZ,
  .crystalLoad     = SI5351_CRYSTAL_LOAD_10PF,
  .crystalPPM      = 30,
  .plla_configured = false,
  .plla_freq       = 0,
  .pllb_configured = false,
  .pllb_freq       = 0
};

/* ---------------------------------------------------------------------- */
/* PRIVATE FUNCTIONS                                                      */
/* ---------------------------------------------------------------------- */

/**************************************************************************/
/*!
    @brief  Writes an 8 bit values over I2C
*/
/**************************************************************************/
err_t si5351Write8 (uint8_t reg, uint32_t value)
{
  I2CWriteLength = 3;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = SI5351_ADDRESS;
  I2CMasterBuffer[1] = reg;
  I2CMasterBuffer[2] = (value & 0xFF);

  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads a 8 bit values over I2C
*/
/**************************************************************************/
err_t si5351Read8(uint8_t reg, uint8_t *p_value)
{
  /* Write transaction */
  I2CWriteLength = 2;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = SI5351_ADDRESS;
  I2CMasterBuffer[1] = reg;
  i2cEngine();

  /* Read transaction */
  I2CWriteLength = 0;
  I2CReadLength = 1;
  I2CMasterBuffer[0] = SI5351_ADDRESS | SI5351_READBIT;
  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  /* Assign the output value */
  *p_value = I2CSlaveBuffer[0];

  return ERROR_NONE;
}

/* ---------------------------------------------------------------------- */
/* PUBLIC FUNCTIONS                                                       */
/* ---------------------------------------------------------------------- */

/**************************************************************************/
/*!
    @brief  Initialises the I2C block
*/
/**************************************************************************/
err_t si5351Init(void)
{
  /* Initialise I2C */
  i2cInit(I2CMASTER);

  /* Ping the I2C device first to see if it exists! */
  ASSERT(i2cCheckAddress(SI5351_ADDRESS), ERROR_I2C_DEVICENOTFOUND);

  /* Disable all outputs setting CLKx_DIS high */
  ASSERT_STATUS(si5351Write8(SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, 0xFF));

  /* Power down all output drivers */
  ASSERT_STATUS(si5351Write8(SI5351_REGISTER_16_CLK0_CONTROL, 0x80));
  ASSERT_STATUS(si5351Write8(SI5351_REGISTER_17_CLK1_CONTROL, 0x80));
  ASSERT_STATUS(si5351Write8(SI5351_REGISTER_18_CLK2_CONTROL, 0x80));
  ASSERT_STATUS(si5351Write8(SI5351_REGISTER_19_CLK3_CONTROL, 0x80));
  ASSERT_STATUS(si5351Write8(SI5351_REGISTER_20_CLK4_CONTROL, 0x80));
  ASSERT_STATUS(si5351Write8(SI5351_REGISTER_21_CLK5_CONTROL, 0x80));
  ASSERT_STATUS(si5351Write8(SI5351_REGISTER_22_CLK6_CONTROL, 0x80));
  ASSERT_STATUS(si5351Write8(SI5351_REGISTER_23_CLK7_CONTROL, 0x80));

  /* Set the load capacitance for the XTAL */
  ASSERT_STATUS(si5351Write8(SI5351_REGISTER_183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE,
  		                     m_si5351Config.crystalLoad));

  /* Set interrupt masks as required (see Register 2 description in AN619).
     By default, ClockBuilder Desktop sets this register to 0x18.
     Note that the least significant nibble must remain 0x8, but the most
     significant nibble may be modified to suit your needs. */

  /* Reset the PLL config fields just in case we call init again */
  m_si5351Config.plla_configured = false;
  m_si5351Config.plla_freq = 0;
  m_si5351Config.pllb_configured = false;
  m_si5351Config.pllb_freq = 0;

  /* All done! */
  m_si5351Config.initialised = true;

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Configures the Si5351 with config settings generated in
            ClockBuilder. You can use this function to make sure that
            your HW is properly configure and that there are no problems
            with the board itself.

            Running this function should provide the following output:
			* Channel 0: 120.00 MHz
			* Channel 1: 12.00  MHz
			* Channel 2: 13.56  MHz

    @note	This will overwrite all of the config registers!
*/
/**************************************************************************/
err_t si5351SetClockBuilderData(void)
{
  uint16_t i = 0;

  /* Make sure we've called init first */
  ASSERT(m_si5351Config.initialised, ERROR_DEVICENOTINITIALISED);

  /* Disable all outputs setting CLKx_DIS high */
  ASSERT_STATUS(si5351Write8(SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, 0xFF));

  /* Writes configuration data to device using the register map contents
	 generated by ClockBuilder Desktop (registers 15-92 + 149-170) */
  for (i=0; i<sizeof(m_si5351_regs_15to92_149to170)/2; i++)
  {
	ASSERT_STATUS(si5351Write8( m_si5351_regs_15to92_149to170[i][0],
								m_si5351_regs_15to92_149to170[i][1] ));
  }

  /* Apply soft reset */
  ASSERT_STATUS(si5351Write8(SI5351_REGISTER_177_PLL_RESET, 0xAC));

  /* Enabled desired outputs (see Register 3) */
  ASSERT_STATUS(si5351Write8(SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, 0x00));

  return ERROR_NONE;
}

/**************************************************************************/
/*!
  @brief  Sets the multiplier for the specified PLL using integer values

  @param  pll   The PLL to configure, which must be one of the following:
                - SI5351_PLL_A
                - SI5351_PLL_B
  @param  mult  The PLL integer multiplier (must be between 15 and 90)
*/
/**************************************************************************/
err_t si5351SetupPLLInt(si5351PLL_t pll, uint8_t mult)
{
  return si5351SetupPLL(pll, mult, 0, 1);
}

/**************************************************************************/
/*!
    @brief  Sets the multiplier for the specified PLL

    @param  pll   The PLL to configure, which must be one of the following:
                  - SI5351_PLL_A
                  - SI5351_PLL_B
    @param  mult  The PLL integer multiplier (must be between 15 and 90)
    @param	num   The 20-bit numerator for fractional output (0..1,048,575).
    		      Set this to '0' for integer output.
    @param  denom The 20-bit denominator for fractional output (1..1,048,575).
    			  Set this to '1' or higher to avoid divider by zero errors.

    @section PLL Configuration

	fVCO is the PLL output, and must be between 600..900MHz, where:

		fVCO = fXTAL * (a+(b/c))

	fXTAL = the crystal input frequency
	a     = an integer between 15 and 90
    b     = the fractional numerator (0..1,048,575)
    c     = the fractional denominator (1..1,048,575)

	NOTE: Try to use integers whenever possible to avoid clock jitter
	      (only use the a part, setting b to '0' and c to '1').

    See: http://www.silabs.com/Support%20Documents/TechnicalDocs/AN619.pdf

    @section Examples

	INTEGER VALUES

    @code

    si5351Init();

    // Setup the PLLB and Mutltisynth 1 & 2 using only integer values
    si5351SetupPLLInt(SI5351_PLL_B, 36); // PLLB = 900MHz
    si5351SetupMultisynthInt(1, SI5351_PLL_B, SI5351_MULTISYNTH_DIV_6); // CLK1 = PLLB/6 = 150MHz
    si5351SetupMultisynthInt(2, SI5351_PLL_B, SI5351_MULTISYNTH_DIV_8); // CLK2 = PLLB/8 = 112.5MHz

    // Enable to outputs
    si5351EnableOutputs(true);
    @endcode

    FRACTIONAL VALUES

    @code
    si5351Init();

    // Setup PLLA and Multisyth O using fractional values
    si5351SetupPLL(SI5351_PLL_A, 24, 2, 3); // PLLA = 616.66667 MHz (25MHz * 24.66667)
    si5351SetupMultisynth(0, SI5351_PLL_A, 45, 1, 2); // CLK0 = PLLA/45.5 = 13.55311MHz

    // Enable to outputs
    si5351EnableOutputs(true);
    @endcode
*/
/**************************************************************************/
err_t si5351SetupPLL(si5351PLL_t pll,
                     uint8_t     mult,
                     uint32_t    num,
                     uint32_t    denom)
{
  uint32_t P1;       /* PLL config register P1 */
  uint32_t P2;	     /* PLL config register P2 */
  uint32_t P3;	     /* PLL config register P3 */

  /* Basic validation */
  ASSERT( m_si5351Config.initialised, ERROR_DEVICENOTINITIALISED );
  ASSERT( (mult > 14) && (mult < 91), ERROR_INVALIDPARAMETER ); /* mult = 15..90 */
  ASSERT( denom > 0,                  ERROR_INVALIDPARAMETER ); /* Avoid divide by zero */
  ASSERT( num <= 0xFFFFF,             ERROR_INVALIDPARAMETER ); /* 20-bit limit */
  ASSERT( denom <= 0xFFFFF,           ERROR_INVALIDPARAMETER ); /* 20-bit limit */

  /* Feedback Multisynth Divider Equation
   *
   * where: a = mult, b = num and c = denom
   *
   * P1 register is an 18-bit value using following formula:
   *
   * 	P1[17:0] = 128 * mult + floor(128*(num/denom)) - 512
   *
   * P2 register is a 20-bit value using the following formula:
   *
   * 	P2[19:0] = 128 * num - denom * floor(128*(num/denom))
   *
   * P3 register is a 20-bit value using the following formula:
   *
   * 	P3[19:0] = denom
   */

  /* Set the main PLL config registers */
  if (num == 0)
  {
    /* Integer mode */
    P1 = 128 * mult - 512;
    P2 = num;
    P3 = denom;
  }
  else
  {
    /* Fractional mode */
    P1 = (uint32_t)(128 * mult + floor(128 * ((float)num/(float)denom)) - 512);
    P2 = (uint32_t)(128 * num - denom * floor(128 * ((float)num/(float)denom)));
    P3 = denom;
  }

  /* Get the appropriate starting point for the PLL registers */
  uint8_t baseaddr = (pll == SI5351_PLL_A ? 26 : 34);

  /* The datasheet is a nightmare of typos and inconsistencies here! */
  ASSERT_STATUS( si5351Write8( baseaddr,   (P3 & 0x0000FF00) >> 8));
  ASSERT_STATUS( si5351Write8( baseaddr+1, (P3 & 0x000000FF)));
  ASSERT_STATUS( si5351Write8( baseaddr+2, (P1 & 0x00030000) >> 16));
  ASSERT_STATUS( si5351Write8( baseaddr+3, (P1 & 0x0000FF00) >> 8));
  ASSERT_STATUS( si5351Write8( baseaddr+4, (P1 & 0x000000FF)));
  ASSERT_STATUS( si5351Write8( baseaddr+5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16) ));
  ASSERT_STATUS( si5351Write8( baseaddr+6, (P2 & 0x0000FF00) >> 8));
  ASSERT_STATUS( si5351Write8( baseaddr+7, (P2 & 0x000000FF)));

  /* Reset both PLLs */
  ASSERT_STATUS( si5351Write8(SI5351_REGISTER_177_PLL_RESET, (1<<7) | (1<<5) ));

  /* Store the frequency settings for use with the Multisynth helper */
  if (pll == SI5351_PLL_A)
  {
    float32_t fvco = m_si5351Config.crystalFreq * (mult + ( (float32_t)num / (float32_t)denom ));
    m_si5351Config.plla_configured = true;
    m_si5351Config.plla_freq = (uint32_t)floor(fvco);
  }
  else
  {
	float32_t fvco = m_si5351Config.crystalFreq * (mult + ( (float32_t)num / (float32_t)denom ));
	m_si5351Config.pllb_configured = true;
	m_si5351Config.pllb_freq = (uint32_t)floor(fvco);
  }

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Configures the Multisynth divider using integer output.

    @param  output		The output channel to use (0..2)
    @param  pllSource	The PLL input source to use, which must be one of:
						- SI5351_PLL_A
						- SI5351_PLL_B
	@param	div			The integer divider for the Multisynth output,
						which must be one of the following values:
						- SI5351_MULTISYNTH_DIV_4
						- SI5351_MULTISYNTH_DIV_6
						- SI5351_MULTISYNTH_DIV_8
*/
/**************************************************************************/
err_t si5351SetupMultisynthInt(uint8_t               output,
                               si5351PLL_t           pllSource,
                               si5351MultisynthDiv_t div)
{
  return si5351SetupMultisynth(output, pllSource, div, 0, 1);
}

/**************************************************************************/
/*!
    @brief  Configures the Multisynth divider, which determines the
            output clock frequency based on the specified PLL input.

    @param  output		The output channel to use (0..2)
    @param  pllSource	The PLL input source to use, which must be one of:
						- SI5351_PLL_A
						- SI5351_PLL_B
	@param	div			The integer divider for the Multisynth output.
	                    If pure integer values are used, this value must
	                    be one of:
						- SI5351_MULTISYNTH_DIV_4
						- SI5351_MULTISYNTH_DIV_6
						- SI5351_MULTISYNTH_DIV_8
						If fractional output is used, this value must be
						between 8 and 900.
    @param	num         The 20-bit numerator for fractional output
                        (0..1,048,575). Set this to '0' for integer output.
    @param  denom       The 20-bit denominator for fractional output
                        (1..1,048,575). Set this to '1' or higher to
                        avoid divide by zero errors.

    @section Output Clock Configuration

	The multisynth dividers are applied to the specified PLL output,
	and are used to reduce the PLL output to a valid range (500kHz
	to 160MHz). The relationship can be seen in this formula, where
	fVCO is the PLL output frequency and MSx is the multisynth
	divider:

		fOUT = fVCO / MSx

	Valid multisynth dividers are 4, 6, or 8 when using integers,
	or any fractional values between 8 + 1/1,048,575 and 900 + 0/1

	The following formula is used for the fractional mode divider:

		a + b / c

	a = The integer value, which must be 4, 6 or 8 in integer mode (MSx_INT=1)
		or 8..900 in fractional mode (MSx_INT=0).
	b = The fractional numerator (0..1,048,575)
	c = The fractional denominator (1..1,048,575)

	@note 	Try to use integers whenever possible to avoid clock jitter

	@note	For output frequencies > 150MHz, you must set the divider
			to 4 and adjust to PLL to generate the frequency (for example
			a PLL of 640 to generate a 160MHz output clock). This is not
			yet supported in the driver, which limits frequencies to
			500kHz .. 150MHz.

	@note   For frequencies below 500kHz (down to 8kHz) Rx_DIV must be
			used, but this isn't currently implemented in the driver.
*/
/**************************************************************************/
err_t si5351SetupMultisynth(uint8_t     output,
                            si5351PLL_t pllSource,
                            uint32_t    div,
                            uint32_t    num,
                            uint32_t    denom)
{
  uint32_t P1;       /* Multisynth config register P1 */
  uint32_t P2;	     /* Multisynth config register P2 */
  uint32_t P3;	     /* Multisynth config register P3 */

  /* Basic validation */
  ASSERT( m_si5351Config.initialised, ERROR_DEVICENOTINITIALISED);
  ASSERT( output < 3,                 ERROR_INVALIDPARAMETER);  /* Channel range */
  ASSERT( div > 3,                    ERROR_INVALIDPARAMETER);  /* Divider integer value */
  ASSERT( div < 901,                  ERROR_INVALIDPARAMETER);  /* Divider integer value */
  ASSERT( denom > 0,                  ERROR_INVALIDPARAMETER ); /* Avoid divide by zero */
  ASSERT( num <= 0xFFFFF,             ERROR_INVALIDPARAMETER ); /* 20-bit limit */
  ASSERT( denom <= 0xFFFFF,           ERROR_INVALIDPARAMETER ); /* 20-bit limit */

  /* Make sure the requested PLL has been initialised */
  if (pllSource == SI5351_PLL_A)
  {
    ASSERT(m_si5351Config.plla_configured = true, ERROR_INVALIDPARAMETER);
  }
  else
  {
    ASSERT(m_si5351Config.pllb_configured = true, ERROR_INVALIDPARAMETER);
  }

  /* Output Multisynth Divider Equations
   *
   * where: a = div, b = num and c = denom
   *
   * P1 register is an 18-bit value using following formula:
   *
   * 	P1[17:0] = 128 * a + floor(128*(b/c)) - 512
   *
   * P2 register is a 20-bit value using the following formula:
   *
   * 	P2[19:0] = 128 * b - c * floor(128*(b/c))
   *
   * P3 register is a 20-bit value using the following formula:
   *
   * 	P3[19:0] = c
   */

  /* Set the main PLL config registers */
  if (num == 0)
  {
    /* Integer mode */
    P1 = 128 * div - 512;
    P2 = num;
    P3 = denom;
  }
  else
  {
    /* Fractional mode */
    P1 = (uint32_t)(128 * div + floor(128 * ((float)num/(float)denom)) - 512);
    P2 = (uint32_t)(128 * num - denom * floor(128 * ((float)num/(float)denom)));
    P3 = denom;
  }

  /* Get the appropriate starting point for the PLL registers */
  uint8_t baseaddr = 0;
  switch (output)
  {
    case 0:
      baseaddr = SI5351_REGISTER_42_MULTISYNTH0_PARAMETERS_1;
      break;
    case 1:
      baseaddr = SI5351_REGISTER_50_MULTISYNTH1_PARAMETERS_1;
      break;
    case 2:
      baseaddr = SI5351_REGISTER_58_MULTISYNTH2_PARAMETERS_1;
      break;
  }

  /* Set the MSx config registers */
  ASSERT_STATUS( si5351Write8( baseaddr,   (P3 & 0x0000FF00) >> 8));
  ASSERT_STATUS( si5351Write8( baseaddr+1, (P3 & 0x000000FF)));
  ASSERT_STATUS( si5351Write8( baseaddr+2, (P1 & 0x00030000) >> 16));	/* ToDo: Add DIVBY4 (>150MHz) and R0 support (<500kHz) later */
  ASSERT_STATUS( si5351Write8( baseaddr+3, (P1 & 0x0000FF00) >> 8));
  ASSERT_STATUS( si5351Write8( baseaddr+4, (P1 & 0x000000FF)));
  ASSERT_STATUS( si5351Write8( baseaddr+5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16) ));
  ASSERT_STATUS( si5351Write8( baseaddr+6, (P2 & 0x0000FF00) >> 8));
  ASSERT_STATUS( si5351Write8( baseaddr+7, (P2 & 0x000000FF)));

  /* Configure the clk control and enable the output */
  uint8_t clkControlReg = 0x0F;                             /* 8mA drive strength, MS0 as CLK0 source, Clock not inverted, powered up */
  if (pllSource == SI5351_PLL_B) clkControlReg |= (1 << 5); /* Uses PLLB */
  if (num == 0) clkControlReg |= (1 << 6);                  /* Integer mode */
  switch (output)
  {
    case 0:
      ASSERT_STATUS(si5351Write8(SI5351_REGISTER_16_CLK0_CONTROL, clkControlReg));
      break;
    case 1:
      ASSERT_STATUS(si5351Write8(SI5351_REGISTER_17_CLK1_CONTROL, clkControlReg));
      break;
    case 2:
      ASSERT_STATUS(si5351Write8(SI5351_REGISTER_18_CLK2_CONTROL, clkControlReg));
      break;
  }

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Enables or disables all clock outputs
*/
/**************************************************************************/
err_t si5351EnableOutputs(bool enabled)
{
  /* Make sure we've called init first */
  ASSERT(m_si5351Config.initialised, ERROR_DEVICENOTINITIALISED);

  /* Enabled desired outputs (see Register 3) */
  ASSERT_STATUS(si5351Write8(SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, enabled ? 0x00: 0xFF));

  return ERROR_NONE;
}
