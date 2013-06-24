#include <stdio.h>
#include "libusb.h"

/*
static void list_devices(libusb_device **devs)
{
	libusb_device *dev;
	int i = 0;

	while ((dev = devs[i++]) != NULL) {
		struct libusb_device_descriptor desc;
		int r = libusb_get_device_descriptor(dev, &desc);
		if (r < 0) {
			fprintf(stderr, "failed to get device descriptor");
			return;
		}

		printf("%04x:%04x (bus %d, device %d)\n",
			desc.idVendor, desc.idProduct,
			libusb_get_bus_number(dev), libusb_get_device_address(dev));
	}
}
*/

int main(void)
{
  // libusb_device **devices;
  // int response;
  // ssize_t i;
  
  libusb_device_handle *lpcdevice;
  unsigned char buffer[64];
  int transferred = 0;
  int bulk_out_magic_number = 0;

  /* Initialise libusbx */
  if(libusb_init(NULL))
  {
		printf("Failed to initialise libusbx\n");
    return -1;
  }
  
  //  /* Get a list of USB devices */
  //  i = libusb_get_device_list(NULL, &devices);
  //  if (i < 0)
  //  {
  //		printf("Failed getting the USB device list (Error: %d)\n", response);
  //    return (int)i;
  //  }
  //
  //  /* Show a list of connected devices */
  //	list_devices(devices);
  
  /* Try to connect to 0x1FC9, 0x2020 (LPC board) */
  lpcdevice = libusb_open_device_with_vid_pid(NULL, 0x1FC9, 0x2020);
  if (NULL == lpcdevice)
  {
    printf("Unable to open VID:0x1FC9 PID:0x2020\n");
    libusb_exit(NULL);
    return -1;
  }

  /* We need to claim the interface before we can do any IO */
  if(libusb_claim_interface(lpcdevice, 0))
  {
    printf("Unable to claim the device interface for VID:0x1FC9 PID:0x2020\n");
    libusb_exit(NULL);
    return -1;
  }  

  /* Transfer some data! */
  /*
  printf("Transferring data to 0x1FC9/0x2020\n");
  uint8_t counter = 0;
  uint32_t c;
  for( c=0; c < 1000; c++ )
  {
    buffer[0] = counter++;
    buffer[1] = 0xAA; 
    buffer[2] = 0xBB; 
    buffer[3] = 0xCC; 
    buffer[4] = 0xDD; 
    buffer[5] = 0xEE; 
    libusb_bulk_transfer(lpcdevice, 0x04, buffer, 6, &transferred, 0); 
  }
  printf("Done!\n");
  */
  
  
  /* Read incoming data! */
  while(1)
  {
    memset(buffer, 0, sizeof(buffer));
    if(libusb_bulk_transfer(lpcdevice, 0x81, buffer, 64, &transferred, 0))
    {
      printf("\nError in read! received = %d\n", transferred);    
      return -1;    
    }
    else
    {
      printf("\nReceived %d bytes\n", transferred);    
      printf("%d", buffer[0]);
    }
    
    memset(buffer, 0, sizeof(buffer));
    buffer[0] = bulk_out_magic_number++;
    if(libusb_bulk_transfer(lpcdevice, 0x04, buffer, 64, &transferred, 0))
    {
      printf("\nError in sending! sent = %d\n", transferred);
    }else
    {
      printf("\nSent %d bytes, 1st byte = %d\n", transferred, buffer[0]);    
    }
  }

  /* Free up the device list and close libusbx */
	// libusb_free_device_list(devices, 1);
  libusb_release_interface(lpcdevice, 0);
	libusb_exit(NULL);
  
	return 0;
}
