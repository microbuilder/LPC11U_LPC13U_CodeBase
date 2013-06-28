#include <stdio.h>
#include "libusb.h"
#include <windows.h>

#define VID             (0x1FC9)
#define PID             (0x2020)  
#define ENDPOINT_OUT    (0x04)
#define ENDPOINT_IN     (0x81)
#define INTERFACE_NUM   (0)

int main(void)
{
  libusb_device_handle *lpcdevice;

  /* Initialise libusbx */
  if(libusb_init(NULL))
  {
		printf("Failed to initialise libusbx\n");
    return -1;
  }
  
  /* Try to connect to the LPC board */
  lpcdevice = libusb_open_device_with_vid_pid(NULL, VID, PID);
  if (NULL == lpcdevice)
  {
    printf("Unable to open VID:0x%04X PID:0x%04X\n", VID, PID);
    libusb_exit(NULL);
    return -1;
  }

  /* We need to claim the interface before we can do any IO */
  if(libusb_claim_interface(lpcdevice, INTERFACE_NUM))
  {
    printf("Unable to claim interface %d for VID:0x%04X PID:0x%04X\n", INTERFACE_NUM, VID, PID);
    libusb_exit(NULL);
    return -1;
  }

	while(1)
  {
    int transferred = 0;

    /* Send LED command to the simple binary protocol */
    static uint8_t led_toggle = 0;
    led_toggle = 1 - led_toggle;

    unsigned char buffer_out[64] = { 0x10, 0x01, 0x00, 0x01, led_toggle };
    libusb_bulk_transfer(lpcdevice, ENDPOINT_OUT, buffer_out, 64, &transferred, 0);

    /* check response */
    unsigned char buffer_in[64] = { 0 };
    if(libusb_bulk_transfer(lpcdevice, ENDPOINT_IN, buffer_in, 64, &transferred, 0))
    {
      printf("\nError in read! received = %d\n", transferred);    
      return -1;    
    }
    else
    {
      printf("\nReceived %d bytes:\n", transferred);    
      for (uint32_t i=0; i<64; i++)
      {
        printf("%02x ", buffer_in[i]);
        if (i%8 == 7)
        {
          printf("\n");
        }
      }
    }
		Sleep(1000);
  }

  /* Free up the device list and close libusbx */
  libusb_release_interface(lpcdevice, INTERFACE_NUM);
  libusb_exit(NULL);

  return 0;
}
