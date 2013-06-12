/**************************************************************************/
/*!
    @file     usb_msc.c
    @author   Thach Ha (tinyusb.net)

    @section DESCRIPTION

    MSC support functions for USB

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
#include "drivers/storage/fatfs/diskio.h"

#ifdef CFG_USB_MSC
#define CACHE_SIZE 512

const uint8_t usb_msc_inquiry[] = "microBuilder.eu";

/* void print_cache(uint8_t* buffer)
{
  uint32_t i;
  for (i=0; i<512; i++)
  {
    if (i%16 == 0)
    {
      printf("\n");
    }
    printf("%X ", buffer[i]);
  }
  printf("\n");
}*/

/**************************************************************************/
/*!

*/
/**************************************************************************/
void MSC_Write(uint32_t offset, uint8_t **src, uint32_t length)
{
  static uint8_t cache_data[CACHE_SIZE];
  static uint32_t cache_sector = UINT32_MAX;
  static uint32_t cache_count = 0;

  ASSERT(length + cache_count <= CACHE_SIZE, (void) 0); // current implementation of Rom Driver does not has length > 512, here is just safe guard

  if ( cache_sector != (offset / CACHE_SIZE) ) // new block
  {
    cache_sector = (offset / CACHE_SIZE);
    cache_count = 0;
  }

  memcpy(cache_data + cache_count, *src, length);
  cache_count += length;

  if ( cache_count == CACHE_SIZE) // not enough to write, continue caching
  {
    ASSERT( disk_write(0, cache_data, cache_sector, 1) == RES_OK, (void) 0 );
    cache_count = 0;
  }

}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void MSC_Read(uint32_t offset, uint8_t **dst, uint32_t length)
{
  static uint8_t cache_data[CACHE_SIZE];
  static uint32_t cache_sector = UINT32_MAX;

  ASSERT(length <= CACHE_SIZE, (void) 0); // current implementation of Rom Driver does not has length > 512, here is just safe guard

  if ( cache_sector != (offset / CACHE_SIZE) ) // new block
  {
    cache_sector = (offset / CACHE_SIZE);
    ASSERT( disk_read(0, cache_data, cache_sector, 1) == RES_OK, (void) 0 );
  }

  memcpy(*dst, cache_data + (offset%CACHE_SIZE), length);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
ErrorCode_t MSC_Verify(uint32_t offset, uint8_t buf[], uint32_t length)
{
//  return memcmp(DiskImage + offset, buf, length) ? ERR_FAILED : LPC_OK;
  return LPC_OK;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void MSC_GetWriteBuf(uint32_t offset, uint8_t **buff_adr, uint32_t length)
{

}

/**************************************************************************/
/*!

*/
/**************************************************************************/
ErrorCode_t usb_msc_init(USBD_HANDLE_T hUsb, USB_INTERFACE_DESCRIPTOR const * const pInterface, uint32_t *mem_base, uint32_t *mem_size)
{
  uint16_t sector_size = 0;
  uint32_t sector_count = 0;

  ASSERT( pInterface, ERR_FAILED);

  if (disk_status(0) & STA_NOINIT)
  {
    ASSERT( !(disk_initialize(0) & (STA_NOINIT | STA_NODISK) ), ERR_FAILED);
  }

  ASSERT ( disk_ioctl(0, GET_SECTOR_SIZE , &sector_size)  == RES_OK, ERR_FAILED);
  ASSERT ( disk_ioctl(0, GET_SECTOR_COUNT, &sector_count) == RES_OK, ERR_FAILED);

  USBD_MSC_INIT_PARAM_T msc_param =
  {
      .mem_base        = *mem_base,
      .mem_size        = *mem_size,

      .BlockSize       = sector_size,
      .BlockCount      = sector_count,
      .MemorySize      = sector_size*sector_count,

      .InquiryStr      = (uint8_t*) usb_msc_inquiry,
      .intf_desc       = (uint8_t*) pInterface,

      .MSC_Write       = MSC_Write,
      .MSC_Read        = MSC_Read,
      .MSC_Verify      = MSC_Verify,
      .MSC_GetWriteBuf = MSC_GetWriteBuf
  };


  ASSERT_USB_STATUS( USBD_API->msc->init(hUsb, &msc_param) );

  ASSERT_MESSAGE( *mem_size > msc_param.mem_size, ERR_FAILED, "not enough memory");

  *mem_base += (*mem_size - msc_param.mem_size);
  *mem_size = msc_param.mem_size;

  return LPC_OK;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
ErrorCode_t usb_msc_configured(USBD_HANDLE_T hUsb)
{
  uint8_t dummy = 0;

  USBD_API->hw->WriteEP(hUsb , MSC_EP_IN , &dummy , 1); // initial packet for IN endpoint , will not work if omitted
  return LPC_OK;
}

#endif
