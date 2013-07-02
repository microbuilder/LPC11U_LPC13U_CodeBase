/**************************************************************************/
/*! 
    @file     uart.h
    @author   K. Townsend (microBuilder.eu)
    @date     22 March 2010
    @version  0.10

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
#ifndef __USART_H__ 
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

/* Uncomment this line to enable HW flow control */
// #define UART_RTS_CTS_FLOWCONTROL

#define USART_RBR_MASK                           ((unsigned int) 0x000000FF)

#define USART_IER_RBR_Interrupt_MASK             ((unsigned int) 0x00000001) // Enables the received data available interrupt
#define USART_IER_RBR_Interrupt_Enabled          ((unsigned int) 0x00000001)
#define USART_IER_RBR_Interrupt_Disabled         ((unsigned int) 0x00000000)
#define USART_IER_THRE_Interrupt_MASK            ((unsigned int) 0x00000002) // Enables the THRE interrupt
#define USART_IER_THRE_Interrupt_Enabled         ((unsigned int) 0x00000002)
#define USART_IER_THRE_Interrupt_Disabled        ((unsigned int) 0x00000000)
#define USART_IER_RLS_Interrupt_MASK             ((unsigned int) 0x00000004) // Enables the Rx line status interrupt
#define USART_IER_RLS_Interrupt_Enabled          ((unsigned int) 0x00000004)
#define USART_IER_RLS_Interrupt_Disabled         ((unsigned int) 0x00000000)
#define USART_IER_ABEOIntEn_MASK                 ((unsigned int) 0x00000100) // End of auto-baud interrupt
#define USART_IER_ABEOIntEn_Enabled              ((unsigned int) 0x00000100)
#define USART_IER_ABEOIntEn_Disabled             ((unsigned int) 0x00000000)
#define USART_IER_ABTOIntEn_MASK                 ((unsigned int) 0x00000200) // Auto-baud timeout interrupt
#define USART_IER_ABTOIntEn_Enabled              ((unsigned int) 0x00000200)
#define USART_IER_ABTOIntEn_Disabled             ((unsigned int) 0x00000000)

#define USART_IIR_IntStatus_MASK                 ((unsigned int) 0x00000001) // Interrupt status
#define USART_IIR_IntStatus_InterruptPending     ((unsigned int) 0x00000001)
#define USART_IIR_IntStatus_NoInterruptPending   ((unsigned int) 0x00000000)
#define USART_IIR_IntId_MASK                     ((unsigned int) 0x0000000E) // Interrupt identification
#define USART_IIR_IntId_RLS                      ((unsigned int) 0x00000006) // Receive line status
#define USART_IIR_IntId_RDA                      ((unsigned int) 0x00000004) // Receive data available
#define USART_IIR_IntId_CTI                      ((unsigned int) 0x0000000C) // Character time-out indicator
#define USART_IIR_IntId_THRE                     ((unsigned int) 0x00000002) // THRE interrupt
#define USART_IIR_IntId_MODEM                    ((unsigned int) 0x00000000) // Modem interrupt
#define USART_IIR_FIFO_Enable_MASK               ((unsigned int) 0x000000C0)
#define USART_IIR_ABEOInt_MASK                   ((unsigned int) 0x00000100) // End of auto-baud interrupt
#define USART_IIR_ABEOInt                        ((unsigned int) 0x00000100)
#define USART_IIR_ABTOInt_MASK                   ((unsigned int) 0x00000200) // Auto-baud time-out interrupt
#define USART_IIR_ABTOInt                        ((unsigned int) 0x00000200)

#define USART_FCR_FIFO_Enable_MASK               ((unsigned int) 0x00000001) // UART FIFOs enabled/disabled
#define USART_FCR_FIFO_Enabled                   ((unsigned int) 0x00000001)
#define USART_FCR_FIFO_Disabled                  ((unsigned int) 0x00000000)
#define USART_FCR_Rx_FIFO_Reset_MASK             ((unsigned int) 0x00000002)
#define USART_FCR_Rx_FIFO_Reset                  ((unsigned int) 0x00000002) // Clear Rx FIFO
#define USART_FCR_Tx_FIFO_Reset_MASK             ((unsigned int) 0x00000004)
#define USART_FCR_Tx_FIFO_Reset                  ((unsigned int) 0x00000004) // Clear Tx FIFO
#define USART_FCR_Rx_Trigger_Level_Select_MASK   ((unsigned int) 0x000000C0) // Chars written before before interrupt
#define USART_FCR_Rx_Trigger_Level_Select_1Char  ((unsigned int) 0x00000000) 
#define USART_FCR_Rx_Trigger_Level_Select_4Char  ((unsigned int) 0x00000040) 
#define USART_FCR_Rx_Trigger_Level_Select_8Char  ((unsigned int) 0x00000080) 
#define USART_FCR_Rx_Trigger_Level_Select_12Char ((unsigned int) 0x000000C0) 

#define USART_MCR_DTR_Control_MASK               ((unsigned int) 0x00000001) // Source for modem output pin DTR
#define USART_MCR_DTR_Control                    ((unsigned int) 0x00000001)
#define USART_MCR_RTS_Control_MASK               ((unsigned int) 0x00000002) // Source for modem output pin RTS
#define USART_MCR_RTS_Control                    ((unsigned int) 0x00000002)
#define USART_MCR_Loopback_Mode_Select_MASK      ((unsigned int) 0x00000010) // Diagnostic loopback mode
#define USART_MCR_Loopback_Mode_Select_Enabled   ((unsigned int) 0x00000010)
#define USART_MCR_Loopback_Mode_Select_Disabled  ((unsigned int) 0x00000000)
#define USART_MCR_RTSen_MASK                     ((unsigned int) 0x00000040) // Disable auto-rts flow control
#define USART_MCR_RTSen_Enabled                  ((unsigned int) 0x00000040)
#define USART_MCR_RTSen_Disabled                 ((unsigned int) 0x00000000)
#define USART_MCR_CTSen_MASK                     ((unsigned int) 0x00000080) // Disable auto-cts flow control
#define USART_MCR_CTSen_Enabled                  ((unsigned int) 0x00000080)
#define USART_MCR_CTSen_Disabled                 ((unsigned int) 0x00000000)

#define USART_LCR_Word_Length_Select_MASK        ((unsigned int) 0x00000003) // Word Length Selector
#define USART_LCR_Word_Length_Select_5Chars      ((unsigned int) 0x00000000)
#define USART_LCR_Word_Length_Select_6Chars      ((unsigned int) 0x00000001)
#define USART_LCR_Word_Length_Select_7Chars      ((unsigned int) 0x00000002)
#define USART_LCR_Word_Length_Select_8Chars      ((unsigned int) 0x00000003)
#define USART_LCR_Stop_Bit_Select_MASK           ((unsigned int) 0x00000004) // Stop bit select
#define USART_LCR_Stop_Bit_Select_1Bits          ((unsigned int) 0x00000000)
#define USART_LCR_Stop_Bit_Select_2Bits          ((unsigned int) 0x00000004)
#define USART_LCR_Parity_Enable_MASK             ((unsigned int) 0x00000008) // Parity enable
#define USART_LCR_Parity_Enabled                 ((unsigned int) 0x00000008)
#define USART_LCR_Parity_Disabled                ((unsigned int) 0x00000000)
#define USART_LCR_Parity_Select_MASK             ((unsigned int) 0x00000030) // Parity select
#define USART_LCR_Parity_Select_OddParity        ((unsigned int) 0x00000000)
#define USART_LCR_Parity_Select_EvenParity       ((unsigned int) 0x00000010)
#define USART_LCR_Parity_Select_Forced1          ((unsigned int) 0x00000020)
#define USART_LCR_Parity_Select_Forced0          ((unsigned int) 0x00000030)
#define USART_LCR_Break_Control_MASK             ((unsigned int) 0x00000040) // Break transmission control
#define USART_LCR_Break_Control_Enabled          ((unsigned int) 0x00000040)
#define USART_LCR_Break_Control_Disabled         ((unsigned int) 0x00000000)
#define USART_LCR_Divisor_Latch_Access_MASK      ((unsigned int) 0x00000080) // Divisor latch access
#define USART_LCR_Divisor_Latch_Access_Enabled   ((unsigned int) 0x00000080)
#define USART_LCR_Divisor_Latch_Access_Disabled  ((unsigned int) 0x00000000)

#define USART_LSR_RDR_MASK                       ((unsigned int) 0x00000001) // Receiver data ready
#define USART_LSR_RDR_EMPTY                      ((unsigned int) 0x00000000) // U0RBR is empty
#define USART_LSR_RDR_DATA                       ((unsigned int) 0x00000001) // U0RBR contains valid data
#define USART_LSR_OE_MASK                        ((unsigned int) 0x00000002) // Overrun error
#define USART_LSR_OE                             ((unsigned int) 0x00000002)
#define USART_LSR_PE_MASK                        ((unsigned int) 0x00000004) // Parity error
#define USART_LSR_PE                             ((unsigned int) 0x00000004)
#define USART_LSR_FE_MASK                        ((unsigned int) 0x00000008) // Framing error
#define USART_LSR_FE                             ((unsigned int) 0x00000008)
#define USART_LSR_BI_MASK                        ((unsigned int) 0x00000010) // Break interrupt
#define USART_LSR_BI                             ((unsigned int) 0x00000010)
#define USART_LSR_THRE_MASK                      ((unsigned int) 0x00000020) // Transmitter holding register empty
#define USART_LSR_THRE                           ((unsigned int) 0x00000020)
#define USART_LSR_TEMT_MASK                      ((unsigned int) 0x00000040) // Transmitter empty
#define USART_LSR_TEMT                           ((unsigned int) 0x00000040)
#define USART_LSR_RXFE_MASK                      ((unsigned int) 0x00000080) // Error in Rx FIFO
#define USART_LSR_RXFE                           ((unsigned int) 0x00000080)

#define USART_MSR_Delta_CTS_MASK                 ((unsigned int) 0x00000001) // State change of input CTS
#define USART_MSR_Delta_CTS                      ((unsigned int) 0x00000001)
#define USART_MSR_Delta_DSR_MASK                 ((unsigned int) 0x00000002) // State change of input DSR
#define USART_MSR_Delta_DSR                      ((unsigned int) 0x00000002)
#define USART_MSR_Trailing_Edge_RI_MASK          ((unsigned int) 0x00000004) // Low to high transition of input RI
#define USART_MSR_Trailing_Edge_RI               ((unsigned int) 0x00000004)
#define USART_MSR_Delta_DCD_MASK                 ((unsigned int) 0x00000008) // State change of input DCD
#define USART_MSR_Delta_DCD                      ((unsigned int) 0x00000008)
#define USART_MSR_CTS_MASK                       ((unsigned int) 0x00000010) // Clear to send state
#define USART_MSR_CTS                            ((unsigned int) 0x00000010)
#define USART_MSR_DSR_MASK                       ((unsigned int) 0x00000020) // Data set ready state
#define USART_MSR_DSR                            ((unsigned int) 0x00000020)
#define USART_MSR_RI_MASK                        ((unsigned int) 0x00000040) // Ring indicator state
#define USART_MSR_RI                             ((unsigned int) 0x00000040)
#define USART_MSR_DCD_MASK                       ((unsigned int) 0x00000080) // Data carrier detect state
#define USART_MSR_DCD                            ((unsigned int) 0x00000080)

#define USART_ACR_Start_MASK                     ((unsigned int) 0x00000001) // Auto-baud start/stop
#define USART_ACR_Start                          ((unsigned int) 0x00000001)
#define USART_ACR_Stop                           ((unsigned int) 0x00000000)
#define USART_ACR_Mode_MASK                      ((unsigned int) 0x00000002) // Auto-baud mode select
#define USART_ACR_Mode_Mode1                     ((unsigned int) 0x00000000)
#define USART_ACR_Mode_Mode2                     ((unsigned int) 0x00000002)
#define USART_ACR_AutoRestart_MASK               ((unsigned int) 0x00000004)
#define USART_ACR_AutoRestart_NoRestart          ((unsigned int) 0x00000000)
#define USART_ACR_AutoRestart_Restart            ((unsigned int) 0x00000004) // Restart in case of time-out
#define USART_ACR_ABEOIntClr_MASK                ((unsigned int) 0x00000100) // End of auto-baud interrupt clear bit
#define USART_ACR_ABEOIntClr                     ((unsigned int) 0x00000100) 
#define USART_ACR_ABTOIntClr_MASK                ((unsigned int) 0x00000200) // Auto-baud timeout interrupt clear bit
#define USART_ACR_ABTOIntClr                     ((unsigned int) 0x00000200)

#define USART_FDR_DIVADDVAL_MASK                 ((unsigned int) 0x0000000F) // Fractional divider: prescaler register
#define USART_FDR_MULVAL_MASK                    ((unsigned int) 0x000000F0) // Fractional divider: prescaler multiplier

#define USART_TER_TXEN_MASK                      ((unsigned int) 0x00000080) // UART transmit enable
#define USART_TER_TXEN_Enabled                   ((unsigned int) 0x00000080)
#define USART_TER_TXEN_Disabled                  ((unsigned int) 0x00000000)

#define USART_RS485CTRL_NMMEN_MASK               ((unsigned int) 0x00000001) // Normal multi-drop mode
#define USART_RS485CTRL_NMMEN                    ((unsigned int) 0x00000001)
#define USART_RS485CTRL_RXDIS_MASK               ((unsigned int) 0x00000002) // Receiver
#define USART_RS485CTRL_RXDIS                    ((unsigned int) 0x00000002)
#define USART_RS485CTRL_AADEN_MASK               ((unsigned int) 0x00000004) // Auto-address detect
#define USART_RS485CTRL_AADEN                    ((unsigned int) 0x00000004)
#define USART_RS485CTRL_SEL_MASK                 ((unsigned int) 0x00000008) 
#define USART_RS485CTRL_SEL_RTS                  ((unsigned int) 0x00000000) // Use RTS for direction control
#define USART_RS485CTRL_SEL_DTS                  ((unsigned int) 0x00000008) // Use DTS for direction control
#define USART_RS485CTRL_DCTRL_MASK               ((unsigned int) 0x00000010) // Enable/Disable auto-direction control
#define USART_RS485CTRL_DCTRL_Disabled           ((unsigned int) 0x00000000)
#define USART_RS485CTRL_DCTRL_Enabled            ((unsigned int) 0x00000010)
#define USART_RS485CTRL_OINV_MASK                ((unsigned int) 0x00000020) // Reverse polarity of direction control signal on RTS/DTR pin
#define USART_RS485CTRL_OINV_Normal              ((unsigned int) 0x00000000)
#define USART_RS485CTRL_OINV_Inverted            ((unsigned int) 0x00000020)

#define USART_FIFOLVL_RXFIFOLVL_MASK             ((unsigned int) 0x0000000F)
#define USART_FIFOLVL_RXFIFOLVL_Empty            ((unsigned int) 0x00000000)
#define USART_FIFOLVL_RXFIFOLVL_Full             ((unsigned int) 0x0000000F)
#define USART_FIFOLVL_TXFIFOLVL_MASK             ((unsigned int) 0x00000F00)
#define USART_FIFOLVL_TXFIFOLVL_Empty            ((unsigned int) 0x00000000)
#define USART_FIFOLVL_TXFIFOLVL_Full             ((unsigned int) 0x00000F00)

// Buffer used for circular fifo
typedef struct _uart_buffer_t
{
  uint8_t ep_dir;
  volatile uint8_t len;
  volatile uint8_t wr_ptr;
  volatile uint8_t rd_ptr;
  uint8_t buf[CFG_UART_BUFSIZE];
} uart_buffer_t;

// UART Protocol control block
typedef struct _uart_pcb_t
{
  bool initialised;
  uint32_t baudrate;
  uint32_t status;
  uint32_t pending_tx_data;
  uart_buffer_t rxfifo;
} uart_pcb_t;

void USART_IRQHandler(void);
uart_pcb_t *uartGetPCB(void);
void uartInit(uint32_t Baudrate);
void uartSend(uint8_t *BufferPtr, uint32_t Length);
void uartSendByte (uint8_t byte);

// Rx Buffer access control
void uartRxBufferInit(void);
uint8_t uartRxBufferRead(void);
void uartRxBufferWrite(uint8_t data);
void uartRxBufferClearFIFO(void);
uint8_t uartRxBufferDataPending(void);
bool uartRxBufferReadArray(byte_t* rx, size_t* len);

#ifdef __cplusplus
}
#endif 

#endif
