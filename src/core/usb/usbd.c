/**************************************************************************/
/*!
    @file     usbd.c
    @author   Thach Ha (tinyusb.net)

    @section DESCRIPTION

    Core USB functions

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
#ifdef __CODE_RED
  #include <cr_section_macros.h>
#endif

#include "projectconfig.h"

#include <string.h>

#include "usbd.h"
#include "core/iap/iap.h"

#ifdef CFG_USB

#define USB_ROM_SIZE (1024*2)

volatile static bool isConfigured = false;

#if defined(__CODE_RED)
  /* Comment or uncomment the __DATA(RAM2) appendix to place the
     buffer in the 2KB USB SRAM or the regular 8KB SRAM block.
     This may need to go in the 8KB block if the buffer requires
     more than 2KB, and should be verified when modifying the
     USB drivers */
  uint8_t usb_RomDriver_buffer[USB_ROM_SIZE] ALIGNED(2048) __DATA(RAM2);
#elif defined(__CROSSWORKS_ARM)
  /* Crossworks doesn't define a 2KB USB SRAM region in the */
  /* default memory map so just point to the address for now */
  uint8_t *usb_RomDriver_buffer = (uint8_t*)0x20004800;
#else
  uint8_t *usb_RomDriver_buffer = (uint8_t*)0x20004800;
#endif

USBD_HANDLE_T g_hUsb;

/**************************************************************************/
/*!
    @brief Indicates whether USB is configured or not
*/
/**************************************************************************/
bool usb_isConfigured(void)
{
  return isConfigured;
}


/**************************************************************************/
/*!
    @brief Handler for the USB Configure Event
*/
/**************************************************************************/
ErrorCode_t USB_Configure_Event (USBD_HANDLE_T hUsb)
{
  USB_CORE_CTRL_T* pCtrl = (USB_CORE_CTRL_T*)hUsb;
  if (pCtrl->config_value)
  {
    #if defined(CFG_USB_HID)
    ASSERT_USB_STATUS( usb_hid_configured(hUsb) );
    #endif

    #ifdef CFG_USB_CDC
    ASSERT_USB_STATUS( usb_cdc_configured(hUsb) );
    #endif

    #ifdef CFG_USB_MSC
    ASSERT_USB_STATUS( usb_msc_configured(hUsb));
    #endif

    #ifdef CFG_USB_CUSTOM_CLASS
    ASSERT_USB_STATUS( usb_custom_configured(hUsb) );
    #endif
  }

  isConfigured = true;

  return LPC_OK;
}

/**************************************************************************/
/*!
    @brief Handler for the USB Reset Event
*/
/**************************************************************************/
ErrorCode_t USB_Reset_Event (USBD_HANDLE_T hUsb)
{
  isConfigured = false;
  return LPC_OK;
}

/**************************************************************************/
/*!
    @brief Initialises device for USB and starts the enumeration process
*/
/**************************************************************************/
ErrorCode_t usb_init(void)
{
  uint32_t i;
  uint32_t uid[4];

  /* HARDWARE INIT */

  /* Enable AHB clock to the USB block and USB RAM. */
  LPC_SYSCON->SYSAHBCLKCTRL |= ((0x1<<14) | (0x1<<27));

  /* Pull-down is needed, or internally, VBUS will be floating. This is to
  address the wrong status in VBUSDebouncing bit in CmdStatus register.  */
  LPC_IOCON->PIO0_3   &= ~0x1F;
  LPC_IOCON->PIO0_3   |= (0x01<<0);            /* Secondary function VBUS */
  LPC_IOCON->PIO0_6   &= ~0x07;
  LPC_IOCON->PIO0_6   |= (0x01<<0);            /* Secondary function SoftConn */

  for (i=0; i < strlen(CFG_USB_STRING_MANUFACTURER); i++)
    USB_StringDescriptor.strManufacturer[i] = CFG_USB_STRING_MANUFACTURER[i];

  for (i=0; i < strlen(CFG_USB_STRING_PRODUCT); i++)
    USB_StringDescriptor.strProduct[i] = CFG_USB_STRING_PRODUCT[i];

  /* Use the 128-bit chip ID for USB serial to make sure it's unique */
  iapReadUID(uid);  /* 1st byte is LSB, 4th byte is MSB */
  sprintf((char*)USB_StringDescriptor.strSerial , "%08X%08X%08X%08X", (unsigned int)uid[3], (unsigned int)uid[2], (unsigned int)uid[1], (unsigned int)uid[0]);
  for (i = USB_STRING_SERIAL_LEN-1; i > 0; i--)
  {
    USB_StringDescriptor.strSerial[i] = ((uint8_t*)USB_StringDescriptor.strSerial)[i];
    ((uint8_t*)USB_StringDescriptor.strSerial)[i] = 0;
  }

  /* ROM DRIVER INIT */
  uint32_t membase = (uint32_t) usb_RomDriver_buffer;
  uint32_t memsize = USB_ROM_SIZE;

  USBD_API_INIT_PARAM_T usb_param =
    {
    .usb_reg_base        = LPC_USB_BASE,
    .max_num_ep          = USB_MAX_EP_NUM,
    .mem_base            = membase,
    .mem_size            = memsize,

    .USB_Configure_Event = USB_Configure_Event,
    .USB_Reset_Event     = USB_Reset_Event
  };

  USB_CORE_DESCS_T DeviceDes =
  {
    .device_desc      = (uint8_t*) &USB_DeviceDescriptor,
    .string_desc      = (uint8_t*) &USB_StringDescriptor,
    .full_speed_desc  = (uint8_t*) &USB_FsConfigDescriptor,
    .high_speed_desc  = (uint8_t*) &USB_FsConfigDescriptor,
    .device_qualifier = NULL
  };

  /* Start USB hardware initialisation */
  ASSERT_USB_STATUS(USBD_API->hw->Init(&g_hUsb, &DeviceDes, &usb_param));

  membase += (memsize - usb_param.mem_size);
  memsize = usb_param.mem_size;

  /* Initialise the class driver(s) */
  #ifdef CFG_USB_CDC
    ASSERT_USB_STATUS( usb_cdc_init(g_hUsb, &USB_FsConfigDescriptor.CDC_CCI_Interface,
            &USB_FsConfigDescriptor.CDC_DCI_Interface, &membase, &memsize) );
  #endif

  #ifdef CFG_USB_HID_KEYBOARD
    ASSERT_USB_STATUS( usb_hid_init(g_hUsb , &USB_FsConfigDescriptor.HID_KeyboardInterface ,
            HID_KeyboardReportDescriptor, USB_FsConfigDescriptor.HID_KeyboardHID.DescriptorList[0].wDescriptorLength,
            &membase , &memsize) );
  #endif

  #ifdef CFG_USB_HID_MOUSE
    ASSERT_USB_STATUS( usb_hid_init(g_hUsb , &USB_FsConfigDescriptor.HID_MouseInterface    ,
            HID_MouseReportDescriptor, USB_FsConfigDescriptor.HID_MouseHID.DescriptorList[0].wDescriptorLength,
            &membase , &memsize) );
  #endif

  #ifdef CFG_USB_HID_GENERIC
    ASSERT_USB_STATUS( usb_hid_init(g_hUsb , &USB_FsConfigDescriptor.HID_GenericInterface    ,
            HID_GenericReportDescriptor, USB_FsConfigDescriptor.HID_GenericHID.DescriptorList[0].wDescriptorLength,
            &membase , &memsize) );
  #endif

  #ifdef CFG_USB_MSC
    // there is chance where SD card is not inserted, thus the msc init fails, should continue instead of return
    if ( usb_msc_init(g_hUsb, &USB_FsConfigDescriptor.MSC_Interface, &membase, &memsize) != LPC_OK)
    {
      _PRINTF("MSC class fails to init\n");
    }
  #endif

  #ifdef CFG_USB_CUSTOM_CLASS
    ASSERT_USB_STATUS( usb_custom_init(g_hUsb, &USB_FsConfigDescriptor.Custom_Interface) );
  #endif

  /* Enable the USB interrupt */
  #if defined CFG_MCU_FAMILY_LPC11UXX
    NVIC_EnableIRQ(USB_IRQn);
  #elif defined CFG_MCU_FAMILY_LPC13UXX
    NVIC_EnableIRQ(USB_IRQ_IRQn);
  #else
    #error "No MCU defined"
  #endif

  /* Perform USB soft connect */
  USBD_API->hw->Connect(g_hUsb, 1);

  return LPC_OK;
}

/**************************************************************************/
/*!
    @brief Redirect the USB IRQ handler to the ROM handler
*/
/**************************************************************************/
void USB_IRQHandler(void)
{
  USBD_API->hw->ISR(g_hUsb);
}

#endif
