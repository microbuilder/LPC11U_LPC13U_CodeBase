/**************************************************************************/
/*!
    @file     usb_cdc.c
    @author   Thach Ha (tinyusb.net)

    @section DESCRIPTION

    CDC support functions for USB

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
#include <string.h>
#include "usbd.h"
#include "core/fifo/fifo.h"
#include "core/delay/delay.h"

#ifdef CFG_USB_CDC

#define ALIAS(f) __attribute__ ((weak, alias (#f)))

static USBD_HANDLE_T g_hCdc;
static CDC_LINE_CODING line_coding;
static bool isConnected = false;             /* ToDo: Consider work-around */

#if defined CFG_MCU_FAMILY_LPC11UXX
  FIFO_DEF(ff_cdc_tx, CDC_BUFFER_SIZE, uint8_t, false, USB_IRQn);
  FIFO_DEF(ff_cdc_rx, CDC_BUFFER_SIZE, uint8_t, true , USB_IRQn);
#elif defined CFG_MCU_FAMILY_LPC13UXX
  FIFO_DEF(ff_cdc_tx, CDC_BUFFER_SIZE, uint8_t, false, USB_IRQ_IRQn);
  FIFO_DEF(ff_cdc_rx, CDC_BUFFER_SIZE, uint8_t, true , USB_IRQ_IRQn);
#else
    #error __FILE__ No MCU defined
#endif

void usb_cdc_recv_isr(void) ALIAS(usb_cdc_recv_isr_default);
/**************************************************************************/
/*!
    @brief  Stub for the optional CDC receive ISR that can be used
            to perform some action when data arrives via USB CDC
*/
/**************************************************************************/
void usb_cdc_recv_isr_default (void)
{
  return;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
bool usb_cdc_isConnected(void)
{
  return isConnected;
}

/**************************************************************************/
/*!
    @brief Adds a single byte to the transmit buffer for USB CDC

    @param[in]  c
                The byte to send

    @returns  TRUE if the byte was successfully added to the TX buffer

    @note See example for usb_cdc_getc
*/
/**************************************************************************/
bool usb_cdc_putc(uint8_t c)
{
  uint32_t start_time = delayGetSecondsActive();

  while ( !fifo_write(&ff_cdc_tx, &c) ) /* TODO: blocking until fifo is available */
  {
    if(delayGetSecondsActive() - start_time > 2)
    {
      isConnected = false;
      fifo_clear(&ff_cdc_tx);
      return false;
    }
  }

  return true;
}

/**************************************************************************/
/*!
    @brief Reads a single byte from the USB CDC buffer

    @param[in]  c
                Pointer to the location where the byte should be written

    @returns  TRUE if a byte was read from the buffer

    @code
    // Convert incoming characters to upper case and send back via CDC
    if (usb_isConfigured())
    {
      uint8_t cdc_char;
      if( usb_cdc_getc(&cdc_char) )
      {
        switch (cdc_char)
        {
          default :
            cdc_char = toupper(cdc_char);
            usb_cdc_putc(cdc_char);
            break;
        }
      }
    }
    @endcode
*/
/**************************************************************************/
bool usb_cdc_getc(uint8_t *c)
{
  ASSERT(c, false); /* Make sure pointer isn't NULL */

  return fifo_read(&ff_cdc_rx, c);
}

/**************************************************************************/
/*!
    @brief Writes the supplied buffer to the USB CDC device

    @param[in]  buffer
                Pointer to the buffer that should be written via USB CDC
    @param[in]  count
                The number of bytes to write

    @returns  The actual number of bytes sent out via USB CDC

    @code
    // Capture printf output (in Red Suite) and send it to USB CDC
    // (Note: With newlib this function should be renamed to _write)
    int __sys_write(int file, char *ptr, int len)
    {
      #ifdef CFG_PRINTF_USBCDC
        // Handle USB CDC output
        if (usb_isConfigured())
        {
          int length = len;
          while(length > 0)
          {
            uint16_t transferredCount;

            transferredCount = usb_cdc_send( (uint8_t*) ptr, length);
            ptr += transferredCount;
            length -= transferredCount;
          }
        }
      #endif

      return len;
    }

    @endcode
*/
/**************************************************************************/
uint16_t usb_cdc_send(uint8_t* buffer, uint16_t count)
{
  uint16_t i=0;

  ASSERT(buffer && count, 0);

  while (i < count && usb_cdc_putc(buffer[i]) )
  {
    i++;
  }

  return i;
}

/**************************************************************************/
/*!
    @brief Reads the incoming CDC buffer up to a maximum number of bytes

    @param[in]  buffer
                Pointer to the buffer where data should be written
    @param[in]  max
                The maximum number of bytes to read

    @returns  The actual number of bytes received
*/
/**************************************************************************/
uint16_t usb_cdc_recv(uint8_t* buffer, uint16_t max)
{
  ASSERT(buffer && max, 0);

  return fifo_readArray(&ff_cdc_rx, buffer, max);
}

// ROM driver bug: cannot hook this to CIC_GetRequest
// Need this to implement GetLineCode & detect
//ErrorCode_t CDC_Control_GetRequest(USBD_HANDLE_T hUsb, USB_SETUP_PACKET *pSetup, uint8_t **pBuffer, uint16_t *length)
//{
//  return LPC_OK;
//}

/**************************************************************************/
/*!
    @brief TODO Add description
*/
/**************************************************************************/
ErrorCode_t CDC_SetLineCoding(USBD_HANDLE_T hUsb, CDC_LINE_CODING *lineCoding)
{
  ASSERT(lineCoding, ERR_FAILED);
  memcpy(&line_coding, lineCoding, sizeof(CDC_LINE_CODING));
  return LPC_OK;
}

/**************************************************************************/
/*!
    @brief TODO Add description
*/
/**************************************************************************/
ErrorCode_t CDC_SendBreak(USBD_HANDLE_T hCDC, uint16_t mstime)
{
  return LPC_OK;
}

/**************************************************************************/
/*!
    @brief Bulk Out handler for the USB ROM drivers (UART TX)
*/
/**************************************************************************/
ErrorCode_t CDC_BulkIn_Hdlr(USBD_HANDLE_T hUsb, void* data, uint32_t event)
{
  if (USB_EVT_IN == event)
  {
    uint8_t buffer[CDC_DATA_EP_MAXPACKET_SIZE];
    uint16_t count;

    count = fifo_readArray(&ff_cdc_tx, buffer, CDC_DATA_EP_MAXPACKET_SIZE);
    USBD_API->hw->WriteEP(hUsb, CDC_DATA_EP_IN, buffer, count); // write data to EP

    isConnected = true;
  }

  return LPC_OK;
}

/**************************************************************************/
/*!
    @brief Bulk Out handler for the USB ROM drivers (UART RX)
*/
/**************************************************************************/
ErrorCode_t CDC_BulkOut_Hdlr(USBD_HANDLE_T hUsb, void* data, uint32_t event)
{
  if (USB_EVT_OUT == event)
  {
    uint16_t count, i;
    uint8_t buffer[CDC_DATA_EP_MAXPACKET_SIZE];

    count = USBD_API->hw->ReadEP(hUsb, CDC_DATA_EP_OUT, buffer);
    for (i=0; i<count; i++)
    {
      fifo_write(&ff_cdc_rx, buffer+i);
    }

    isConnected = true;

    usb_cdc_recv_isr();
  }

  return LPC_OK;
}

/**************************************************************************/
/*!
    @brief Initialises USB CDC using the ROM driver
*/
/**************************************************************************/
ErrorCode_t usb_cdc_init(USBD_HANDLE_T hUsb, USB_INTERFACE_DESCRIPTOR const *const pControlIntfDesc, USB_INTERFACE_DESCRIPTOR const *const pDataIntfDesc, uint32_t* mem_base, uint32_t* mem_size)
{
  USBD_CDC_INIT_PARAM_T cdc_param =
  {
    .mem_base      = *mem_base,
    .mem_size      = *mem_size,

    .cif_intf_desc = (uint8_t*) pControlIntfDesc,
    .dif_intf_desc = (uint8_t*) pDataIntfDesc,

    .SetLineCode   = CDC_SetLineCoding,
    .SendBreak     = CDC_SendBreak,

    // .CIC_GetRequest   = CDC_Control_GetRequest, // bug from romdrive cannot hook to this handler
    // Bug from ROM driver: can not hook bulk in & out handler here, must use USBD API register instead
    // .CDC_BulkIN_Hdlr  = CDC_BulkIn_Hdlr,
    // .CDC_BulkOUT_Hdlr = CDC_BulkOut_Hdlr,
  };

  ASSERT (pControlIntfDesc && pDataIntfDesc, ERR_FAILED);

  /* register Bulk IN & OUT endpoint interrupt handler */
  ASSERT_USB_STATUS ( USBD_API->core->RegisterEpHandler (hUsb , ((CDC_DATA_EP_IN & 0x0F) << 1) +1 , CDC_BulkIn_Hdlr  , NULL) );
  ASSERT_USB_STATUS ( USBD_API->core->RegisterEpHandler (hUsb , (CDC_DATA_EP_OUT & 0x0F) << 1     , CDC_BulkOut_Hdlr , NULL) );

  ASSERT_USB_STATUS( USBD_API->cdc->init(hUsb, &cdc_param, &g_hCdc) );

  /* Update memory variables */
  ASSERT_MESSAGE(*mem_size > cdc_param.mem_size, ERR_FAILED, "not enough memory");

  *mem_base += (*mem_size - cdc_param.mem_size);
  *mem_size = cdc_param.mem_size;

  isConnected = false;

  return LPC_OK;
}

/**************************************************************************/
/*!
    @brief  Callback when the USB Set Configured request is received
*/
/**************************************************************************/
ErrorCode_t usb_cdc_configured(USBD_HANDLE_T hUsb)
{
  uint8_t dummy=0;
  USBD_API->hw->WriteEP(hUsb, CDC_DATA_EP_IN, &dummy, 1); // initial packet for IN endpoint, will not work if omitted

  isConnected = true;

  fifo_clear(&ff_cdc_tx);
  fifo_clear(&ff_cdc_rx);

  return LPC_OK;
}

#endif
