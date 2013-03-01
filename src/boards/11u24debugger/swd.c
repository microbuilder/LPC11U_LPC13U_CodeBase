/****************************************************************************
 *   $Id:: swd.c 6085 2011-01-05 23:55:06Z usb00423                   $
 *   Project: LPC11U00 SWD to USB bridge
 *
 *   Description:
 *     Source code file for basic SWD bitbang support
 *
 ****************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
****************************************************************************/
#include "projectconfig.h"
#include "swd.h"

/* Flag to indicate if the SWD bus is controlled by the host or target */
char SWD_HostBus = 0;

#if SWD_BIT_DELAY > 0
/**************************************************************************/
/*!
    @brief  Bit delay (not really needed on the LPC11U)
*/
/**************************************************************************/
// __attribute__ ((section(".ramFunc")))
void swdBitDelay(void)
{
  volatile char i;

  i=SWD_BIT_DELAY;
  while(i)
    i--;
}
#endif

/**************************************************************************/
/*!
    @brief  Switches to host-controlled bus
*/
/**************************************************************************/
// __attribute__ ((section(".ramFunc")))
void swdTurnaroundHost()
{
  if(!SWD_HostBus)
  {
    /* Clock is idle (low) */
    #if SWD_BIT_DELAY > 0
      swdBitDelay();
    #endif

    /* Set up a port bitmask so that we can write data + clock
     * at the same time. 1s mean ignore this bit. */
    LPC_GPIO->MASK[SWD_PORT] = ~(SWD_SWDIO_BIT_MSK | SWD_SWCLK_BIT_MSK);
    /* Drive clock high and set SWDIO */
    LPC_GPIO->MPIN[SWD_PORT] = SWD_SWDIO_BIT_MSK | SWD_SWCLK_BIT_MSK;
    /* Take over the SWDIO pin */
    LPC_GPIO->DIR[SWD_PORT] |= SWD_SWDIO_BIT_MSK | SWD_SWCLK_BIT_MSK;

    SWD_HostBus = 1;
    /* Clock is idle (high) */
  }
}

/**************************************************************************/
/*!
    @brief  Switch to target-controlled bus
*/
/**************************************************************************/
// __attribute__ ((section(".ramFunc")))
void swdTurnaroundTarget()
{
  if(SWD_HostBus)
  {
    /* Clock is idle (high) */
    #if SWD_BIT_DELAY > 0
      swdBitDelay();
    #endif

    /* Set up a port bitmask so that we can write data + clock
     * at the same time. 1s mean ignore this bit. */
    LPC_GPIO->MASK[SWD_PORT] = ~(SWD_SWDIO_BIT_MSK | SWD_SWCLK_BIT_MSK);
    /* Drive clock low and set SWDIO */
    LPC_GPIO->MPIN[SWD_PORT] = SWD_SWDIO_BIT_MSK;

    #if SWD_BIT_DELAY > 0
      swdBitDelay();
    #endif

    /* Drive clock high */
    LPC_GPIO->SET[SWD_PORT] = SWD_SWCLK_BIT_MSK;

    #if SWD_BIT_DELAY > 0
      swdBitDelay();
    #endif

    /* Drive clock low */
    LPC_GPIO->CLR[SWD_PORT] = SWD_SWCLK_BIT_MSK;

    /* Release SWDIO pin */
    LPC_GPIO->DIR[SWD_PORT] &= ~SWD_SWDIO_BIT_MSK; /* tristate data pin */

    SWD_HostBus = 0;
  }
}

/**************************************************************************/
/*!
    @brief  Sends bits out the wire.  This is a low-level function that
            does not implement any SWD protocol.  The bits are clocked out
            on the clock falling edge for the SWD state machine to latch
            on the rising edge.  After it is done this function leaves the
            clock low.
*/
/**************************************************************************/
// __attribute__ ((section(".ramFunc")))
void swdWriteBits(int nbits, const unsigned long *bits)
{
  int i;
  unsigned long wbuf = *bits;

  swdTurnaroundHost();
  /* Clock is idle (high) */

  i = 0;
  while(nbits)
  {
    #if SWD_BIT_DELAY > 0
      swdBitDelay();
    #endif

    /* Drive SWCLK low and output next bit to SWDIO line */
    LPC_GPIO->MPIN[SWD_PORT] = (((wbuf)&1) << SWD_SWDIO_BIT);

    #if SWD_BIT_DELAY > 0
      swdBitDelay();
    #endif

    /* Prepare for next bit */
    nbits--;
    i++;
    if(i>=32)
      bits++,wbuf=*bits,i=0;
    else
      (wbuf) >>= 1;

    /* Drive SWCLK high */
    LPC_GPIO->SET[SWD_PORT] = SWD_SWCLK_BIT_MSK;
  }
}

/**************************************************************************/
/*!
    @brief  Reads buts from the wire.  This is a low-level function that
            does not implement any SWD protocol.  The bits are expected to
            be clock out on the clock rising edge and then they are latched
            on the clock falling edge.  After it is done this function
            leaves the clock low.
*/
/**************************************************************************/
// __attribute__ ((section(".ramFunc")))
void swdReadBits(int nbits, volatile unsigned long *bits)
{
  int i;

  swdTurnaroundTarget();
  /* Clock is idle (low) */

  *bits = 0;
  i = 0;
  while(nbits)
  {
    #if SWD_BIT_DELAY > 0
      swdBitDelay();
    #endif

    /* Read bit from SWDIO line */
    (*bits) |= ((LPC_GPIO->PIN[SWD_PORT] >> SWD_SWDIO_BIT)&1) << i;

    /* Drive SWCLK high */
    LPC_GPIO->SET[SWD_PORT] = SWD_SWCLK_BIT_MSK;

    #if SWD_BIT_DELAY > 0
      swdBitDelay();
    #endif

    /* Drive SWCLK low */
    LPC_GPIO->CLR[SWD_PORT] = SWD_SWCLK_BIT_MSK;

    /* Prepare for next bit */
    i++;
    if(i>=32)
    {
      bits++;
      if(nbits>1)
      *bits=0; /* prevent hard faults */
      i=0;
    }
    nbits--;
  }
}

/**************************************************************************/
/*!
    @brief  Send 8 zero bits to flush SWD state
*/
/**************************************************************************/
void swdFlush(void)
{
  unsigned long data = 0;

  swdWriteBits(8, &data);
}

/**************************************************************************/
/*!
    @brief  Send magic number to switch to SWD mode.  This function sends
            many zeros, 1s, then the magic number, then more 1s and zeros
            to try to get the SWD state machine's attention if it is
            connected in some unusual state.
*/
/**************************************************************************/
void swdEnable(void)
{
  unsigned long data;

  /* Write 0s */
  data = 0;
  swdWriteBits(32, &data);
  swdWriteBits(32, &data);
  swdWriteBits(32, &data);
  swdWriteBits(32, &data);

  /* Write FFs */
  data = 0xFFFFFFFF;
  swdWriteBits(32, &data);
  swdWriteBits(32, &data);
  swdWriteBits(32, &data);
  swdWriteBits(32, &data);

  data = 0xE79E;
  swdWriteBits(16, &data);

  /* Write FFs */
  data = 0xFFFFFFFF;
  swdWriteBits(32, &data);
  swdWriteBits(32, &data);
  swdWriteBits(32, &data);
  swdWriteBits(32, &data);

  /* Write 0s */
  data = 0;
  swdWriteBits(32, &data);
  swdWriteBits(32, &data);
  swdWriteBits(32, &data);
  swdWriteBits(32, &data);
}

/**************************************************************************/
/*!
    @brief  The SWD protocol consists of read and write requests sent
            from the host in the form of 8-bit packets.  These requests are
            followed by a 3-bit status response from the target and then a
            data phase which is 32 bits of data + 1 bit of parity.  This
            function reads the three bit status responses.
*/
/**************************************************************************/
// __attribute__ ((section(".ramFunc")))
int swdGetTargetResponse(void)
{
  unsigned long data;

  swdReadBits(3, &data);

  return data;
}

/**************************************************************************/
/*!
    @brief  This function counts the number of 1s in an integer.  It is
            used to calculate parity.  This is the MIT HAKMEM (Hacks Memo)
            implementation.
*/
/**************************************************************************/
// __attribute__ ((section(".ramFunc")))
int swdCountOnes(unsigned long n)
{
  register unsigned int tmp;

  tmp = n - ((n >> 1) & 033333333333)
         - ((n >> 2) & 011111111111);
  return ((tmp + (tmp >> 3)) & 030707070707) % 63;
}

/**************************************************************************/
/*!
    @brief  This is one of the two core SWD functions.  It can write to
            a debug port register or to an access port register.  It
            implements the host->target write request, reading the 3-bit
            status, and then writing the 33 bit data+parity.
*/
/**************************************************************************/
// __attribute__ ((section(".ramFunc")))
int swdWrite(char APnDP, int A, unsigned long data)
{
  unsigned long wpr;
  int response;

  wpr = 0b10000001 | (APnDP<<1) | (A<<1); /* A is a 32-bit address bits 0,1 are 0. */
  if(swdCountOnes(wpr&0x1E)%2)            /* odd number of 1s */
    wpr |= 1<<5;                          /* set parity bit */

  #ifdef SWD_DEBUG
    printf("swdWrite(Port=%s, Addr=%d, data=%08x, wpr=%02x) = ",
            APnDP ? "Access" : "Debug",
            A, data, wpr);
  #endif

  swdWriteBits( 8, &wpr);

  /* Now read acknowledgement */
  response = swdGetTargetResponse();
  if(response != SWD_ACK)
  {
    #ifdef SWD_DEBUG
      printf("%s starting write\n", response == SWD_FAULT ? "FAULT" : "WAIT");
    #endif
    swdTurnaroundHost();
    return response;
  }

  /* Send write data */
  swdWriteBits( 32, &data);
  wpr = 0;
  if(swdCountOnes(data)%2)  /* Odd number of 1s */
    wpr = 1;                /* Set parity bit */
  swdWriteBits( 1, &wpr);   /* Send parity */

  #ifdef SWD_DEBUG
    printf("OK\n");
  #endif

  return SWD_ACK;
}

/**************************************************************************/
/*!
    @brief  This is one of the two core SWD functions.  It can read from
            a debug port register or an access port register.  It
            implements the host->target read request, reading the 3-bit
            status, and then reading the 33-bit data+parity.
*/
/**************************************************************************/
// __attribute__ ((section(".ramFunc")))
int swdRead(char APnDP, int A, volatile unsigned long *data)
{
  unsigned long wpr;
  int response;

  wpr = 0b10000101 | (APnDP<<1) | (A<<1); /* A is a 32-bit address bits 0,1 are 0. */
  if(swdCountOnes(wpr&0x1E)%2)            /* Odd number of 1s */
    wpr |= 1<<5;                          /* Set parity bit */

  #ifdef SWD_DEBUG
    printf("swdRead(Port=%s, Addr=%d, wpr=%02x) = ",
                  APnDP ? "Access" : "Debug",
                  A, wpr);
  #endif

  swdWriteBits( 8, &wpr);

  /* Now read acknowledgement */
  response = swdGetTargetResponse();
  if(response != SWD_ACK)
  {
    #ifdef SWD_DEBUG
      printf("%s starting read\n", response == SWD_FAULT ? "FAULT" : "WAIT");
    #endif
    swdTurnaroundHost();
    return response;
  }

  swdReadBits( 32, data);   /* Read data */
  swdReadBits( 1, &wpr);    /* Read parity */
  if(swdCountOnes(wpr)%2)   /* Odd number of 1s */
  {
    if(wpr != 1)
    {
      swdTurnaroundHost();
      return SWD_PARITY;    /* Bad parity */
    }
  }
  else
  {
    if(wpr != 0)
    {
      swdTurnaroundHost();
      return SWD_PARITY;    /* Bad parity */
    }
  }

  #ifdef SWD_DEBUG
    printf("%08x\n", *data);
  #endif
  swdTurnaroundHost();

  return SWD_ACK;
}
