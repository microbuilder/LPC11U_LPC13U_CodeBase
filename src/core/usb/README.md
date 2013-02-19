# USB Support #

This code base include a relatively easy to use USB layer that allows you to enumerate one or more of the following USB device classes:

- **USB CDC** - allows you to redirect printf and CLI input and output to a virtual serial port on any host device
- **USB Mass Storage** - points to an on-board SD card and enumerates the card as an external storage device, allowing you to read and write files from any operating system
- **USB HID** - Out of the box support for the following HID options:
 - **HID Mouse** - emulates a 5-button, 2-scroll wheel USB mouse with an easy wrapper function to update the button/wheel state
 - **HID Keyboard** - emulates a USB keyboard with a wrapper function to send key data to the host device
 - **HID Generic** - Can be used to send and receive HID Reports of a pre-determined size, and multiple reports can optionally be implemented this way with a bit of extra code.

## Auto USB Composite Device Enumeration ##

One of the key advantages of the way USB support is implemented in the code base is that -- within the limits of the avaiable endpoints -- the system will automatically enumerate as a USB Composite device if more than one USB class is selected in the board config file.

For example, if both CFG_USB_CDC and CFG_USB_HID_GENERIC are defined in your board config file, the system will automatically enumerate both devices during initialisation, without any need to update the descriptors yourself.

The USB Product ID will be updated depending on the combination of classes selected in your board config file, and the supplied Windows .inf file to enable CDC support on Windows should take into account any of these combinations.

## Serial Number Based on the Unique Chip ID ##

During USB enumeration, the device descriptor provides a USB Serial Number based on the unique serial number stored on the LPC11U24/LPC11U37/LPC1347, which can be accessed via IAP calls.  This means that every device that you program will have a unique USB serial number, and you can have several identical devices connected on the same host.

## No Major License Restrictions ##

Because all of the USB code is based on the ROM-based drivers, you no longer have any licensing issues with the Keil USB stacks that were used in many previous LPC1K examples from NXP.  All of the custom USB code provided in this API (everything in this directory) is licensed under a BSD-style license, and the public ROM-driver files provided by NXP (everything in the 'romdriver' folder) is provided with license terms that are far more flexible than the previous Keil stacks.