/**************************************************************************/
/*!
    @file     uart.c
    @author   K. Townsend (microBuilder.eu)

    @section DESCRIPTION

    Generic code for UART-based communication.  Incoming text is stored
    in a FIFO Queue for safer processing.

    @section LICENSE

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
#include "projectconfig.h"

#ifdef CFG_ENABLE_UART

#include <string.h>

#include "uart.h"

/**************************************************************************/
/*!
    UART protocol control block, which is used to safely access the
    RX FIFO buffer from elsewhere in the code.  This should be accessed
    through 'uartGetPCB()'.
*/
/**************************************************************************/
static uart_pcb_t uart_pcb;

/**************************************************************************/
/*!
    IRQ to handle incoming data, etc.
*/
/**************************************************************************/
#if defined CFG_MCU_FAMILY_LPC11UXX
void UART_IRQHandler(void)
#elif defined CFG_MCU_FAMILY_LPC13UXX
void USART_IRQHandler(void)
#else
  #error "uart.c: No MCU defined"
#endif
{
  uint8_t IIRValue, LSRValue;
  uint8_t Dummy = Dummy;

  IIRValue = LPC_USART->IIR;
  IIRValue &= ~(USART_IIR_IntStatus_MASK); /* skip pending bit in IIR */
  IIRValue &= USART_IIR_IntId_MASK;        /* check bit 1~3, interrupt identification */

  // 1.) Check receiver line status
  if (IIRValue == USART_IIR_IntId_RLS)
  {
    LSRValue = LPC_USART->LSR;
    // Check for errors
    if (LSRValue & (USART_LSR_OE | USART_LSR_PE | USART_LSR_FE | USART_LSR_RXFE | USART_LSR_BI))
    {
      /* There are errors or break interrupt */
      /* Read LSR will clear the interrupt */
      uart_pcb.status = LSRValue;
      Dummy = LPC_USART->RBR;  /* Dummy read on RX to clear interrupt, then bail out */
      return;
    }
    // No error and receive data is ready
    if (LSRValue & USART_LSR_RDR_DATA)
    {
      /* If no error on RLS, normal ready, save into the data buffer. */
      /* Note: read RBR will clear the interrupt */
      uartRxBufferWrite(LPC_USART->RBR);
    }
  }

  // 2.) Check receive data available
  else if (IIRValue == USART_IIR_IntId_RDA)
  {
    // Add incoming text to UART buffer
    uartRxBufferWrite(LPC_USART->RBR);
  }

  // 3.) Check character timeout indicator
  else if (IIRValue == USART_IIR_IntId_CTI)
  {
    /* Bit 9 as the CTI error */
    uart_pcb.status |= 0x100;
  }

  // 4.) Check THRE (transmit holding register empty)
  else if (IIRValue == USART_IIR_IntId_THRE)
  {
    /* Check status in the LSR to see if valid data in U0THR or not */
    LSRValue = LPC_USART->LSR;
    if (LSRValue & USART_LSR_THRE)
    {
      uart_pcb.pending_tx_data = 0;
    }
    else
    {
      uart_pcb.pending_tx_data= 1;
    }
  }
  return;
}

/**************************************************************************/
/*!
    @brief  Get a pointer to the UART's protocol control block, which can
            be used to control the RX FIFO buffer and check whether UART
            has already been initialised or not.

    @section Example

    @code
    // Make sure that UART is initialised
    uart_pcb_t *pcb = uartGetPCB();
    if (!pcb->initialised)
    {
      uartInit(CFG_UART_BAUDRATE);
    }
    @endcode

*/
/**************************************************************************/
uart_pcb_t *uartGetPCB()
{
    return &uart_pcb;
}

/**************************************************************************/
/*!
    @brief Initialises UART at the specified baud rate.

    @param[in]  baudRate
                The baud rate to use when configuring the UART.
*/
/**************************************************************************/
void uartInit(uint32_t baudrate)
{
  uint32_t fDiv;
  uint32_t regVal=regVal;

  #if defined CFG_MCU_FAMILY_LPC11UXX
    NVIC_DisableIRQ(UART_IRQn);
  #elif defined CFG_MCU_FAMILY_LPC13UXX
    NVIC_DisableIRQ(USART_IRQn);
  #endif

  /* Clear protocol control blocks */
  memset(&uart_pcb, 0, sizeof(uart_pcb_t));
  uart_pcb.pending_tx_data = 0;
  uartRxBufferInit();

  /* Set 0.18 UART RXD */
  LPC_IOCON->PIO0_18 &= ~0x07;
  LPC_IOCON->PIO0_18 |= 0x01;

  /* Set 0.19 UART TXD */
  LPC_IOCON->PIO0_19 &= ~0x07;
  LPC_IOCON->PIO0_19 |= 0x01;

#if defined UART_RTS_CTS_FLOWCONTROL
  /* start RTS/CTS flow control setup - davidson 130531 */
  LPC_IOCON->PIO0_7 &= ~0x07;   /* Flow control CTS UART I/O Config  Type Input */
  LPC_IOCON->PIO0_7 |= 0x01; 

  LPC_IOCON->PIO1_17 &= ~0x07;  /* Flow control RTS UART I/O Config  Type Output */
  LPC_IOCON->PIO1_17 |= 0x01;  
  /* end of RTS/CTS flow control setup */
#endif  
  
  /* Enable UART clock */
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 12);
  LPC_SYSCON->UARTCLKDIV = 1;

  /* 8 bits, no Parity, 1 Stop bit */
  LPC_USART->LCR = (USART_LCR_Word_Length_Select_8Chars |
                USART_LCR_Stop_Bit_Select_1Bits |
                USART_LCR_Parity_Disabled |
                USART_LCR_Parity_Select_OddParity |
                USART_LCR_Break_Control_Disabled |
                USART_LCR_Divisor_Latch_Access_Enabled);

  /* Baud rate */
  regVal = LPC_SYSCON->UARTCLKDIV;
  fDiv = ((SystemCoreClock/LPC_SYSCON->UARTCLKDIV)/16)/baudrate;

  LPC_USART->DLM = fDiv / 256;
  LPC_USART->DLL = fDiv % 256;

  /* Set DLAB back to 0 */
  LPC_USART->LCR = (USART_LCR_Word_Length_Select_8Chars |
                USART_LCR_Stop_Bit_Select_1Bits |
                USART_LCR_Parity_Disabled |
                USART_LCR_Parity_Select_OddParity |
                USART_LCR_Break_Control_Disabled |
                USART_LCR_Divisor_Latch_Access_Disabled);

  /* Enable and reset TX and RX FIFO. */
  LPC_USART->FCR = (USART_FCR_FIFO_Enabled |
                USART_FCR_Rx_FIFO_Reset |
                USART_FCR_Tx_FIFO_Reset);

#if defined UART_RTS_CTS_FLOWCONTROL
  /* Enable Auto RTS and Auto CTS  - davidson 130531 */
  LPC_UART->MCR = 0xC0;
#endif
                
  /* Read to clear the line status. */
  regVal = LPC_USART->LSR;

  /* Ensure a clean start, no data in either TX or RX FIFO. */
  while (( LPC_USART->LSR & (USART_LSR_THRE|USART_LSR_TEMT)) != (USART_LSR_THRE|USART_LSR_TEMT) );
  while ( LPC_USART->LSR & USART_LSR_RDR_DATA )
  {
    /* Dump data from RX FIFO */
    regVal = LPC_USART->RBR;
  }

  /* Set the initialised flag in the protocol control block */
  uart_pcb.initialised = 1;
  uart_pcb.baudrate = baudrate;

  /* Enable the UART Interrupt */
  #if defined CFG_MCU_FAMILY_LPC11UXX
    NVIC_EnableIRQ(UART_IRQn);
  #elif defined CFG_MCU_FAMILY_LPC13UXX
    NVIC_EnableIRQ(USART_IRQn);
  #endif
  LPC_USART->IER = USART_IER_RBR_Interrupt_Enabled | USART_IER_RLS_Interrupt_Enabled;

  return;
}

/**************************************************************************/
/*!
    @brief Sends the contents of supplied text buffer over UART.

    @param[in]  bufferPtr
                Pointer to the text buffer
    @param[in]  bufferPtr
                The size of the text buffer

    @section Example

    @code
    // Set 5-character text buffer
    uint8_t uartBuffer[5] = { 'T', 'e', 's', 't', '\n' };
    // Send contents of uartBuffer
    uartSend((uint8_t *)uartBuffer, 5);
    @endcode

*/
/**************************************************************************/
void uartSend (uint8_t *bufferPtr, uint32_t length)
{
  while (length != 0)
  {
    /* THRE status, contain valid data */
    while ( !(LPC_USART->LSR & USART_LSR_THRE) );
    LPC_USART->THR = *bufferPtr;

    bufferPtr++;
    length--;
  }

  return;
}

/**************************************************************************/
/*!
    @brief Sends a single byte over UART.

    @param[in]  byte
                Byte value to send

    @section Example

    @code
    // Send 0xFF over UART
    uartSendByte(0xFF);
    // Send 'B' over UART (note single quotes)
    uartSendByte('B');
    @endcode

*/
/**************************************************************************/
void uartSendByte (uint8_t byte)
{
  /* THRE status, contain valid data */
  while ( !(LPC_USART->LSR & USART_LSR_THRE) );
  LPC_USART->THR = byte;

  return;
}

#endif
