/*******************************************************************
    Copyright (C) 2009 FreakLabs
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
    3. Neither the name of the the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS'' AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
    OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
    OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
    SUCH DAMAGE.

    Originally written by Christopher Wang aka Akiba.
    Please post support questions to the FreakLabs forum.

*******************************************************************/
/*!
    \file
    \ingroup


*/
/**************************************************************************/
#include "projectconfig.h"

#ifdef CFG_CHIBI

#include "chb.h"
#include "chb_spi.h"
#if CFG_CHIBI_SPIPORT == 0
  #include "core/ssp0/ssp0.h"
#elif CFG_CHIBI_SPIPORT == 1
  #include "core/ssp1/ssp1.h"
#endif

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_spi_init()
{
    // Set slave select to output and high
    LPC_GPIO->DIR[CFG_CHIBI_SSPORT] |= (1 << CFG_CHIBI_SSPIN);

    // set the slave select to idle
    CHB_SPI_DISABLE();

    // Initialise SPI (use Mode 0 - SSEL active low and clock starts low)
    #if CFG_CHIBI_SPIPORT == 0
      ssp0Init();
    #elif CFG_CHIBI_SPIPORT == 1
      ssp1Init();
    #endif
}

/**************************************************************************/
/*!
    This function both reads and writes data. For write operations, include data
    to be written as argument. For read ops, use dummy data as arg. Returned
    data is read byte val.
*/
/**************************************************************************/
U8 chb_xfer_byte(U8 data)
{
    #if CFG_CHIBI_SPIPORT == 0
      /* Move on only if NOT busy and TX FIFO not full */
      while ((LPC_SSP0->SR & (SSP0_SR_TNF_NOTFULL | SSP0_SR_BSY_BUSY)) != SSP0_SR_TNF_NOTFULL);
      LPC_SSP0->DR = data;

      /* Wait until the busy bit is cleared and receive buffer is not empty */
      while ( (LPC_SSP0->SR & (SSP0_SR_BSY_BUSY|SSP0_SR_RNE_NOTEMPTY)) != SSP0_SR_RNE_NOTEMPTY );

      // Read the queue
      return LPC_SSP0->DR;
    #elif CFG_CHIBI_SPIPORT == 1
      /* Move on only if NOT busy and TX FIFO not full */
      while ((LPC_SSP1->SR & (SSP1_SR_TNF_NOTFULL | SSP1_SR_BSY_BUSY)) != SSP1_SR_TNF_NOTFULL);
      LPC_SSP1->DR = data;

      /* Wait until the busy bit is cleared and receive buffer is not empty */
      while ( (LPC_SSP1->SR & (SSP1_SR_BSY_BUSY|SSP1_SR_RNE_NOTEMPTY)) != SSP1_SR_RNE_NOTEMPTY );

      // Read the queue
      return LPC_SSP1->DR;
    #endif
}

#endif /* CFG_CHIBI */
