/*****************************************************************************
*
*  spi.c - CC3000 Host Driver Implementation.
*  Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*****************************************************************************/
#include "projectconfig.h"

#ifdef CFG_CC3000

#include "spi.h"
#include "hostdriver/hci.h"
#include "hostdriver/evnt_handler.h"
#include "core/gpio/gpio.h"
#include "core/delay/delay.h"

#if CFG_CC3000_SPI_PORT == 1
  #include "core/ssp1/ssp1.h"
#else
  #include "core/ssp0/ssp0.h"
#endif

#define READ                            (3)
#define WRITE                           (1)
#define HI(value)                       (((value) & 0xFF00) >> 8)
#define LO(value)                       ((value) & 0x00FF)
#define CC3000_ASSERT_CS                do { LPC_GPIO->CLR[CFG_CC3000_CS_PORT] = (1 << CFG_CC3000_CS_PIN); } while(0)
#define CC3000_DEASSERT_CS              do { LPC_GPIO->SET[CFG_CC3000_CS_PORT] = (1 << CFG_CC3000_CS_PIN); } while(0)
#define HEADERS_SIZE_EVNT               (SPI_HEADER_SIZE + 5)
#define SPI_HEADER_SIZE                 (5)

#define eSPI_STATE_POWERUP              (0)
#define eSPI_STATE_INITIALIZED          (1)
#define eSPI_STATE_IDLE                 (2)
#define eSPI_STATE_WRITE_IRQ            (3)
#define eSPI_STATE_WRITE_FIRST_PORTION  (4)
#define eSPI_STATE_WRITE_EOT            (5)
#define eSPI_STATE_READ_IRQ             (6)
#define eSPI_STATE_READ_FIRST_PORTION   (7)
#define eSPI_STATE_READ_EOT             (8)

typedef struct
{
  gcSpiHandleRx  SPIRxHandler;

  unsigned short usTxPacketLength;
  unsigned short usRxPacketLength;
  unsigned long  ulSpiState;
  unsigned char *pTxPacket;
  unsigned char *pRxPacket;

} tSpiInformation;

tSpiInformation sSpiInformation;

/* Static buffer for 5 bytes of SPI HEADER */
unsigned char tSpiReadHeader[] = {READ, 0, 0, 0, 0};

/* Function prototypes for private functions */
void SpiWriteDataSynchronous(unsigned char *data, unsigned short size);
void SpiWriteAsync(const unsigned char *data, unsigned short size);
void SpiPauseSpi(void);
void SpiResumeSpi(void);
void SSIContReadOperation(void);

/* The magic number that resides at the end of the TX/RX buffer (1 byte after
 * the allocated size) for the purpose of detection of the overrun. The
 * location of the memory where the magic number resides shall never be
 * written. In case it is written - an overrun occured and either receive
 * function or send function will be stuck forever. */
#define CC3000_BUFFER_MAGIC_NUMBER (0xDE)

char          wlan_rx_buffer[CC3000_RX_BUFFER_SIZE];
unsigned char wlan_tx_buffer[CC3000_TX_BUFFER_SIZE];

/**************************************************************************/
/*!

 */
/**************************************************************************/
void SpiClose(void)
{
  if (sSpiInformation.pRxPacket)
  {
    sSpiInformation.pRxPacket = 0;
  }

  /*  Disable Interrupt in GPIOA module... */
  tSLInformation.WlanInterruptDisable();
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
void SpiOpen(gcSpiHandleRx pfRxHandler)
{
  sSpiInformation.ulSpiState = eSPI_STATE_POWERUP;

  memset(wlan_rx_buffer, 0, sizeof(wlan_rx_buffer));
  memset(wlan_tx_buffer, 0, sizeof(wlan_rx_buffer));

  sSpiInformation.SPIRxHandler              = pfRxHandler;
  sSpiInformation.usTxPacketLength          = 0;
  sSpiInformation.pTxPacket                 = NULL;
  sSpiInformation.pRxPacket                 = (unsigned char *)wlan_rx_buffer;
  sSpiInformation.usRxPacketLength          = 0;
  wlan_rx_buffer[CC3000_RX_BUFFER_SIZE - 1] = CC3000_BUFFER_MAGIC_NUMBER;
  wlan_tx_buffer[CC3000_TX_BUFFER_SIZE - 1] = CC3000_BUFFER_MAGIC_NUMBER;

  /* Enable interrupt on the GPIO pin of WLAN IRQ */
  tSLInformation.WlanInterruptEnable();
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
int init_spi(void)
{
  #if CFG_CC3000_SPI_PORT == 1
    ssp1Init();
  #else
    ssp0Init();
  #endif

  /* Set POWER_EN pin to output */
  LPC_GPIO->DIR[CFG_CC3000_EN_PORT] |= (1 << CFG_CC3000_EN_PIN);
  LPC_GPIO->CLR[CFG_CC3000_EN_PORT]  = (1 << CFG_CC3000_EN_PIN);
  delay(100);

  /* Set CS pin to output */
  LPC_GPIO->DIR[CFG_CC3000_CS_PORT] |= (1 << CFG_CC3000_CS_PIN);
  CC3000_DEASSERT_CS;

  /* Setup the interrupt pin */
  LPC_GPIO->DIR[CFG_CC3000_IRQ_PORT] &= ~(1 << CFG_CC3000_IRQ_PIN);
  /* Channel 2, sense (0=edge, 1=level), polarity (0=low/falling, 1=high/rising) */
  GPIOSetPinInterrupt( 2, CFG_CC3000_IRQ_PORT, CFG_CC3000_IRQ_PIN, 0, 0 );
  /* Disable interrupt 2 on falling edge */
  GPIOPinIntDisable(2, 0);

  return(ESUCCESS);
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
long SpiFirstWrite(unsigned char *ucBuf, unsigned short usLength)
{
  /* Workaround for the first transaction */
  CC3000_ASSERT_CS;

  /* SPI writes first 4 bytes of data */
  delay(1);
  SpiWriteDataSynchronous(ucBuf, 4);
  delay(1);

  SpiWriteDataSynchronous(ucBuf + 4, usLength - 4);

  /* From this point on - operate in a regular manner */
  sSpiInformation.ulSpiState = eSPI_STATE_IDLE;

  CC3000_DEASSERT_CS;

  return(0);
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
long SpiWrite(unsigned char *pUserBuffer, unsigned short usLength)
{
  unsigned char ucPad = 0;

  /* Figure out the total length of the packet in order to figure out if
   * there is padding or not */
  if(!(usLength & 0x0001))
  {
    ucPad++;
  }

  pUserBuffer[0] = WRITE;
  pUserBuffer[1] = HI(usLength + ucPad);
  pUserBuffer[2] = LO(usLength + ucPad);
  pUserBuffer[3] = 0;
  pUserBuffer[4] = 0;

  usLength += (SPI_HEADER_SIZE + ucPad);

/* The magic number that resides at the end of the TX/RX buffer (1 byte after
 * the allocated size) for the purpose of detection of the overrun. The
 * location of the memory where the magic number resides shall never be
 * written. In case it is written - an overrun occured and either receive
 * function or send function will be stuck forever. */
  if (wlan_tx_buffer[CC3000_TX_BUFFER_SIZE - 1] != CC3000_BUFFER_MAGIC_NUMBER)
  {
    while (1);
  }

  if (sSpiInformation.ulSpiState == eSPI_STATE_POWERUP)
  {
    while (sSpiInformation.ulSpiState != eSPI_STATE_INITIALIZED);
  }

  if (sSpiInformation.ulSpiState == eSPI_STATE_INITIALIZED)
  {
    /* This is time for first TX/RX transactions over SPI: the IRQ is down
     * - so we need to send read buffer size command */
    SpiFirstWrite(pUserBuffer, usLength);
  }
  else
  {
    /* We need to prevent a race condition here that can occur in case two
     * back to back packets are senr to the device, so the state will move
     * to IDLE and once again to not IDLE due to IRQ */
    tSLInformation.WlanInterruptDisable();

    while (sSpiInformation.ulSpiState != eSPI_STATE_IDLE);

    sSpiInformation.ulSpiState = eSPI_STATE_WRITE_IRQ;
    sSpiInformation.pTxPacket = pUserBuffer;
    sSpiInformation.usTxPacketLength = usLength;

    /* Assert the CS line and wait till SSI IRQ line is active and then
     * initialize write operation */
    CC3000_ASSERT_CS;

    /* Re-enable IRQ - if it was not disabled - this is not a problem... */
    tSLInformation.WlanInterruptEnable();

    /* Check for a missing interrupt between the CS assertion and re-enabling
     * the interrupts */
    if (tSLInformation.ReadWlanInterruptPin() == 0)
    {
      SpiWriteDataSynchronous(sSpiInformation.pTxPacket, sSpiInformation.usTxPacketLength);

      sSpiInformation.ulSpiState = eSPI_STATE_IDLE;

      CC3000_DEASSERT_CS;
    }
  }

  /* Due to the fact that we are currently implementing a blocking situation
   * here we will wait till end of transaction */
  while (eSPI_STATE_IDLE != sSpiInformation.ulSpiState);

  return(0);
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
void SpiWriteDataSynchronous(unsigned char *data, unsigned short size)
{
  unsigned char dummy = dummy;
  while (size)
  {
    /* TX */
    #if CFG_CC3000_SPI_PORT == 1
      while ((LPC_SSP1->SR & (SSP1_SR_TNF_NOTFULL | SSP1_SR_BSY_BUSY)) != SSP1_SR_TNF_NOTFULL);
      LPC_SSP1->DR = *data;
    #else
      while ((LPC_SSP0->SR & (SSP0_SR_TNF_NOTFULL | SSP0_SR_BSY_BUSY)) != SSP0_SR_TNF_NOTFULL);
      LPC_SSP0->DR = *data;
    #endif

    /* RX */
    #if CFG_CC3000_SPI_PORT == 1
      while ((LPC_SSP1->SR & (SSP1_SR_BSY_BUSY|SSP1_SR_RNE_NOTEMPTY)) != SSP1_SR_RNE_NOTEMPTY);
      dummy = LPC_SSP1->DR;
    #else
      while ((LPC_SSP0->SR & (SSP0_SR_BSY_BUSY|SSP0_SR_RNE_NOTEMPTY)) != SSP0_SR_RNE_NOTEMPTY);
      dummy = LPC_SSP0->DR;
    #endif

    size--;
    data++;
  }
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
void SpiReadDataSynchronous(unsigned char *data, unsigned short size)
{
  long i = 0;
  unsigned char *data_to_send = tSpiReadHeader;

  for (i = 0; i < size; i ++)
  {
    /* Dummy write */
    #if CFG_CC3000_SPI_PORT == 1
      while ((LPC_SSP1->SR & (SSP1_SR_TNF_NOTFULL | SSP1_SR_BSY_BUSY)) != SSP1_SR_TNF_NOTFULL);
      LPC_SSP1->DR = data_to_send[0];
    #else
      while ((LPC_SSP0->SR & (SSP0_SR_TNF_NOTFULL | SSP0_SR_BSY_BUSY)) != SSP0_SR_TNF_NOTFULL);
      LPC_SSP0->DR = data_to_send[0];
    #endif

    /* RX */
    #if CFG_CC3000_SPI_PORT == 1
      while ( (LPC_SSP1->SR & (SSP1_SR_BSY_BUSY|SSP1_SR_RNE_NOTEMPTY)) != SSP1_SR_RNE_NOTEMPTY );
      data[i] = LPC_SSP1->DR;
    #else
      while ( (LPC_SSP0->SR & (SSP0_SR_BSY_BUSY|SSP0_SR_RNE_NOTEMPTY)) != SSP0_SR_RNE_NOTEMPTY );
      data[i] = LPC_SSP0->DR;
    #endif
  }
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
void SpiReadHeader(void)
{
  SpiReadDataSynchronous(sSpiInformation.pRxPacket, 10);
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
long SpiReadDataCont(void)
{
  long data_to_recv;
  unsigned char *evnt_buff, type;

  /* Determine what type of packet we have */
  evnt_buff =  sSpiInformation.pRxPacket;
  data_to_recv = 0;
  STREAM_TO_UINT8((char *)(evnt_buff + SPI_HEADER_SIZE), HCI_PACKET_TYPE_OFFSET, type);

  switch(type)
  {
    case HCI_TYPE_DATA:
      {
        /* We need to read the rest of the data.. */
        STREAM_TO_UINT16((char *)(evnt_buff + SPI_HEADER_SIZE), HCI_DATA_LENGTH_OFFSET, data_to_recv);
        if (!((HEADERS_SIZE_EVNT + data_to_recv) & 1))
        {
          data_to_recv++;
        }

        if (data_to_recv)
        {
          SpiReadDataSynchronous(evnt_buff + 10, data_to_recv);
        }
        break;
      }
    case HCI_TYPE_EVNT:
      {
        /* Calculate the length of the data */
        STREAM_TO_UINT8((char *)(evnt_buff + SPI_HEADER_SIZE), HCI_EVENT_LENGTH_OFFSET, data_to_recv);
        data_to_recv -= 1;

        /* Add padding byte if needed */
        if ((HEADERS_SIZE_EVNT + data_to_recv) & 1)
        {
          data_to_recv++;
        }

        if (data_to_recv)
        {
          SpiReadDataSynchronous(evnt_buff + 10, data_to_recv);
        }

        sSpiInformation.ulSpiState = eSPI_STATE_READ_EOT;
        break;
      }
  }

  return (0);
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
void SpiPauseSpi(void)
{
  GPIOPinIntDisable( 2, 0 );
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
void SpiResumeSpi(void)
{
  GPIOPinIntEnable( 2, 0 );
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
void SpiTriggerRxProcessing(void)
{
  /* Trigger Rx processing */
  SpiPauseSpi();
  CC3000_DEASSERT_CS;

  /* The magic number that resides at the end of the TX/RX buffer (1 byte after
   * the allocated size) for the purpose of detection of the overrun. The
   * location of the memory where the magic number resides shall never be
   * written. In case it is written - an overrun occured and either receive
   * function or send function will be stuck forever. */
  if (sSpiInformation.pRxPacket[CC3000_RX_BUFFER_SIZE - 1] != CC3000_BUFFER_MAGIC_NUMBER)
  {
    /* You've got problems if you're here! */
    while (1);
  }

  sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
  sSpiInformation.SPIRxHandler(sSpiInformation.pRxPacket + SPI_HEADER_SIZE);
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
void SSIContReadOperation(void)
{
  /* The header was read - continue with the payload read */
  if (!SpiReadDataCont())
  {
    /* All the data was read - finalize handling by switching to the task
     *  and calling from task event handler */
    SpiTriggerRxProcessing();
  }
}

/**************************************************************************/
/*!
    IRQ Handler (triggers on CC3000MOD IRQ line)

    \brief  GPIO interrupt handler. When the external SPI WLAN device is
            ready to interact with Host CPU it generates an interrupt signal.
            After that Host CPU has registrated this interrupt request
            it sets the corresponding /CS pin in active state.
*/
/**************************************************************************/
#if defined CFG_MCU_FAMILY_LPC11UXX
void FLEX_INT2_IRQHandler(void)
#elif defined CFG_MCU_FAMILY_LPC13UXX
void PIN_INT2_IRQHandler(void)
#else
  #error "CC3000 spi.c: No MCU defined"
#endif
{
  /* Make sure the right flag is set in the int status register */
  if ( LPC_GPIO_PIN_INT->IST & (0x1 << 2) )
  {
    /* Falling Edge */
    if ( ( LPC_GPIO_PIN_INT->FALL & (0x1 << 2) ) && ( LPC_GPIO_PIN_INT->IENF & (0x1 << 2) ) )
    {
      if (sSpiInformation.ulSpiState == eSPI_STATE_POWERUP)
      {
        /* This means IRQ line was low call a callback of HCI Layer to inform on event */
         sSpiInformation.ulSpiState = eSPI_STATE_INITIALIZED;
      }
      else if (sSpiInformation.ulSpiState == eSPI_STATE_IDLE)
      {
        sSpiInformation.ulSpiState = eSPI_STATE_READ_IRQ;

        /* IRQ line goes down - start reception */
        CC3000_ASSERT_CS;

        /* Wait for TX/RX Compete which will come as DMA interrupt */
        SpiReadHeader();

        sSpiInformation.ulSpiState = eSPI_STATE_READ_EOT;

        SSIContReadOperation();
      }
      else if (sSpiInformation.ulSpiState == eSPI_STATE_WRITE_IRQ)
      {
        SpiWriteDataSynchronous(sSpiInformation.pTxPacket, sSpiInformation.usTxPacketLength);

        sSpiInformation.ulSpiState = eSPI_STATE_IDLE;

        CC3000_DEASSERT_CS;
      }
      LPC_GPIO_PIN_INT->FALL = 0x1 << 2;
    }
    /* Clear the FLEX/PIN interrupt */
    LPC_GPIO_PIN_INT->IST = 0x1 << 2;
  }
  return;
}



#endif
