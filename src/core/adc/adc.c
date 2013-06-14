/**************************************************************************/
/*!
    @file     adc.c

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
#include "adc.h"

/**************************************************************************/
/*!
    @brief Powers up and configures the ADC block
*/
/**************************************************************************/
void adcInit(void)
{
  /* Disable Power down bit to the ADC block. */
  LPC_SYSCON->PDRUNCFG &= ~(0x1<<4);

  /* Enable AHB clock to the ADC. */
  LPC_SYSCON->SYSAHBCLKCTRL |= (1<<13);

  /* These pins should be setup elsewhere in your code -- ideally in
   * boardInit() or in the driver init sequence -- but the various pin
   * config options are presented below for reference sake */

  /* P0.11 = ADC0 */
  // LPC_IOCON->TDI_PIO0_11   &= ~0x9F;
  // LPC_IOCON->TDI_PIO0_11   |= 0x02;

  /* P0.12 = ADC1 */
  // LPC_IOCON->TMS_PIO0_12   &= ~0x9F;
  // LPC_IOCON->TMS_PIO0_12   |= 0x02;

  /* P0.13 = ADC2 */
  // LPC_IOCON->TDO_PIO0_13   &= ~0x9F;
  // LPC_IOCON->TDO_PIO0_13   |= 0x02;

  /* P0.14 = ADC3 */
  // LPC_IOCON->TRST_PIO0_14  &= ~0x9F;
  // LPC_IOCON->TRST_PIO0_14  |= 0x02;

  /* P0.15 = ADC4 ... this is also SWDIO so be careful with this pin! */
  // LPC_IOCON->SWDIO_PIO0_15 &= ~0x9F;
  // LPC_IOCON->SWDIO_PIO0_15 |= 0x02;

  /* P0.16 = ADC5 */
  // LPC_IOCON->PIO0_16       &= ~0x9F;
  // LPC_IOCON->PIO0_16       |= 0x01;

  /* P0.22 = ADC6 */
  // LPC_IOCON->PIO0_22       &= ~0x9F;
  // LPC_IOCON->PIO0_22       |= 0x01;

  /* P0.23 = ADC7 */
  // LPC_IOCON->PIO0_23       &= ~0x9F;
  // LPC_IOCON->PIO0_23       |= 0x01;

  #if defined CFG_MCU_FAMILY_LPC11UXX
    /* Setup the ADC clock, conversion mode, etc. */
    LPC_ADC->CR = (0x01 << 0)                            |
                  ((SystemCoreClock / ADC_CLK - 1) << 8) |  /* CLKDIV = Fpclk / 1000000 - 1 */
                  (0 << 16)                              |  /* BURST = 0, no BURST, software controlled */
                  (0 << 24)                              |  /* START = 0 A/D conversion stops */
                  (0 << 27);                                /* EDGE = 0 (CAP/MAT rising edge, trigger A/D conversion) */
  #elif defined CFG_MCU_FAMILY_LPC13UXX
    /* Setup the ADC clock, conversion mode, etc. */
    LPC_ADC->CR = (0x01 << 0)                            |
                  ((SystemCoreClock / ADC_CLK - 1) << 8) |  /* CLKDIV = Fpclk / 1000000 - 1 */
                  (0 << 16)                              |  /* BURST = 0, no BURST, software controlled */
                  (0 << 17)                              |  /* CLKS = 0, 11 clocks/10 bits */
                  #if CFG_ADC_MODE_LOWPOWER
                  (1 << 22)                              |  /* Low-power mode */
                  #endif
                  #if CFG_ADC_MODE_10BIT
                  (1 << 23)                              |  /* 10-bit mode */
                  #endif
                  (0 << 24)                              |  /* START = 0 A/D conversion stops */
                  (0 << 27);                                /* EDGE = 0 (CAP/MAT rising edge, trigger A/D conversion) */
  #endif
}

/**************************************************************************/
/*!
    @brief  Reads the specified ADC channel

    @param  channelNum
            The ADC channel number to read (0..7)
*/
/**************************************************************************/
uint32_t adcRead(uint8_t channelNum)
{
  uint32_t regVal;

  /* Channel number is 0 through 7 */
  if (channelNum >= ADC_CHANNELS)
  {
    channelNum = 0;
  }

  /* Enable the correct channel and start a new conversion */
  LPC_ADC->CR &= 0xFFFFFF00;
  LPC_ADC->CR |= (1 << 24) | (1 << channelNum);

  /* Wait until the conversion is complete */
  while (1)
  {
    regVal = LPC_ADC->DR[channelNum];
    if ( regVal & ADC_DONE )
    {
      break;
    }
  }

  /* Stop the ADC */
  LPC_ADC->CR &= 0xF8FFFFFF;

  #if defined CFG_MCU_FAMILY_LPC11UXX
      return ( regVal >> 6 ) & 0x3FF;
  #elif defined CFG_MCU_FAMILY_LPC13UXX
    #if CFG_ADC_MODE_10BIT
      return ( regVal >> 6 ) & 0x3FF;
    #else
      return ( regVal >> 4 ) & 0xFFF;
    #endif
  #endif
}
