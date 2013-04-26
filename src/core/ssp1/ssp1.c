/**************************************************************************/
/*!
    @file     ssp1.c
    @author   K. Townsend (microBuilder.eu)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2012, K. Townsend (microBuilder.eu)
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

#include "core/gpio/gpio.h"
#include "core/ssp1/ssp1.h"

/**************************************************************************/
/*!
    Set SSP clock to slow (400 KHz)
*/
/**************************************************************************/
void ssp1ClockSlow()
{
  /* Divide by 15 for SSPCLKDIV */
  LPC_SYSCON->SSP1CLKDIV = SCB_CLKDIV_DIV15;

  /* (PCLK / (CPSDVSR * [SCR+1])) = (4,800,000 / (2 x [5 + 1])) = 400 KHz */
  LPC_SSP1->CR0 = ( (7u << 0)     // Data size = 8-bit  (bits 3:0)
           | (0 << 4)             // Frame format = SPI (bits 5:4)
           #if CFG_SSP_CPOL1 == 1
           | (1  << 6)            // CPOL = 1           (bit 6)
           #else
           | (0  << 6)            // CPOL = 0           (bit 6)
           #endif
           #if CFG_SSP_CPHA1 == 1
           | (1 << 7)             // CPHA = 1           (bit 7)
           #else
           | (0 << 7)             // CPHA = 0           (bit 7)
           #endif
           | SSP1_SCR_5);         // Clock rate = 5     (bits 15:8)

  /* Clock prescale register must be even and at least 2 in master mode */
  LPC_SSP1->CPSR = 2;
}

/**************************************************************************/
/*!
    Set SSP clock to fast (6.0 MHz)
*/
/**************************************************************************/
void ssp1ClockFast()
{
  /* Divide by 1 for SSPCLKDIV */
  LPC_SYSCON->SSP1CLKDIV = SCB_CLKDIV_DIV1;

  /* (PCLK / (CPSDVSR * [SCR+1])) = (72,000,000 / (2 * [5 + 1])) = 6.0 MHz */
  LPC_SSP1->CR0 = ( (7u << 0)     // Data size = 8-bit  (bits 3:0)
           | (0 << 4)             // Frame format = SPI (bits 5:4)
           #if CFG_SSP_CPOL1 == 1
           | (1  << 6)            // CPOL = 1           (bit 6)
           #else
           | (0  << 6)            // CPOL = 0           (bit 6)
           #endif
           #if CFG_SSP_CPHA1 == 1
           | (1 << 7)             // CPHA = 1           (bit 7)
           #else
           | (0 << 7)             // CPHA = 0           (bit 7)
           #endif
           | SSP1_SCR_5);         // Clock rate = 5     (bits 15:8)

  /* Clock prescale register must be even and at least 2 in master mode */
  LPC_SSP1->CPSR = 2;
}

/**************************************************************************/
/*!
    @brief Initialise SSP1
*/
/**************************************************************************/
void ssp1Init(void)
{
  uint8_t i, Dummy=Dummy;

  /* Reset SSP */
  LPC_SYSCON->PRESETCTRL |= (0x1<<2);

  /* Enable AHB clock to the SSP domain. */
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 18);

  #if CFG_SSP_MOSI1_LOCATION == CFG_SSP_MOSI1_0_21
    /* Set P0.21 to SSP MOSI1 */
    LPC_IOCON->PIO0_21 &= ~0x07;
    LPC_IOCON->PIO0_21 |= 0x02;
  #elif CFG_SSP_MOSI1_LOCATION == CFG_SSP_MOSI1_1_22
    /* Set P1.22 to SSP MOSI1 */
    LPC_IOCON->PIO1_22 &= ~0x07;
    LPC_IOCON->PIO1_22 |= 0x02;
  #endif

  #if CFG_SSP_MISO1_LOCATION == CFG_SSP_MISO1_0_22
    /* Set P0.22 to SSP MISO1 */
    LPC_IOCON->PIO0_22 &= ~0x07;
    LPC_IOCON->PIO0_22 |= (0x03); // | (0<<3) | (1<<7);   // MISO1, No pull-up/down, ADMODE = digital
  #elif CFG_SSP_MISO1_LOCATION == CFG_SSP_MISO1_1_21
    /* Set P1.21 to SSP MISO1 */
    LPC_IOCON->PIO1_21 &= ~0x07;
    LPC_IOCON->PIO1_21 |= 0x02;
  #endif

  #if CFG_SSP_SCK1_LOCATION == CFG_SSP_SCK1_1_20
    /* Set 1.20 to SSP SCK1 */
    LPC_IOCON->PIO1_20 &= ~0x07;
    LPC_IOCON->PIO1_20 |= 0x02;
  #elif CFG_SSP_SCK1_LOCATION == CFG_SSP_SCK1_1_15
    /* Set 1.15 to SSP SCK1 */
    LPC_IOCON->PIO1_15 &= ~0x07;
    LPC_IOCON->PIO1_15 |= 0x03;
  #else
    #error "Invalid CFG_SSP_SCK0_LOCATION"
  #endif

  /* Set SPI clock to high-speed by default */
  ssp1ClockFast();

  /* Clear the Rx FIFO */
  for ( i = 0; i < SSP1_FIFOSIZE; i++ )
  {
    Dummy = LPC_SSP1->DR;
  }

  /* Enable device and set it to master mode, no loopback */
  LPC_SSP1->CR1 = SSP1_CR1_SSE_ENABLED | SSP1_CR1_MS_MASTER | SSP1_CR1_LBM_NORMAL;
}

/**************************************************************************/
/*!
    @brief Sends a block of data using SSP1

    @param[in]  buf
                Pointer to the data buffer
    @param[in]  length
                Block length of the data buffer
*/
/**************************************************************************/
void ssp1Send (uint8_t *buf, uint32_t length)
{
  uint32_t i;
  uint8_t Dummy = Dummy;

  for (i = 0; i < length; i++)
  {
    /* Move on only if NOT busy and TX FIFO not full. */
    while ((LPC_SSP1->SR & (SSP1_SR_TNF_NOTFULL | SSP1_SR_BSY_BUSY)) != SSP1_SR_TNF_NOTFULL);
    LPC_SSP1->DR = *buf;
    buf++;

    while ( (LPC_SSP1->SR & (SSP1_SR_BSY_BUSY|SSP1_SR_RNE_NOTEMPTY)) != SSP1_SR_RNE_NOTEMPTY );
    /* Whenever a byte is written, MISO FIFO counter increments, Clear FIFO
    on MISO. Otherwise, when sspReceive is called, previous data byte
    is left in the FIFO. */
    Dummy = LPC_SSP1->DR;
  }

  return;
}

/**************************************************************************/
/*!
    @brief Receives a block of data using SSP1

    @param[in]  buf
                Pointer to the data buffer
    @param[in]  length
                Block length of the data buffer
*/
/**************************************************************************/
void ssp1Receive(uint8_t *buf, uint32_t length)
{
  uint32_t i;

  for ( i = 0; i < length; i++ )
  {
    /* As long as the receive FIFO is not empty, data can be received. */
    LPC_SSP1->DR = 0xFF;

    /* Wait until the Busy bit is cleared */
    while ( (LPC_SSP1->SR & (SSP1_SR_BSY_BUSY|SSP1_SR_RNE_NOTEMPTY)) != SSP1_SR_RNE_NOTEMPTY );

    *buf = LPC_SSP1->DR;
    buf++;
  }

  return;
}
