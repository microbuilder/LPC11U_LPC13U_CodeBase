# LPC11U/LPC13U Code Base - Core Revision History #

Major changes in the LPC11U/LPC13U code base by code base version number.

## 0.8.2 [Ongoing] ##

- Renamed Crossworks project files to CW_*
- Renamed Keil project files to Keil_*
- Added stepper support to board config files
- Changed the clock setup in core/adc
- Added basic TCS34725 driver
- First attempt at a simple binary protocol (CFG_PROTOCOL)
- Added CFG\_BRD\_SIMULATOR as a board option (mostly for unit tests)
- Fixed negative value bug in timespanCreate
- Renamed /src/drivers/statistics to /src/drivers/filters
- Changed usb hid generic callbacks to be more general  
+ usb_hid_generic_recv_isr(USB_HID_GenericReportOut_t* report) to usb_hid_generic_recv_isr(uint8_t report[], uint32_t length)
+ usb_hid_generic_report_request_isr(USB_HID_GenericReportIn_t* report) to usb_hid_generic_report_request_isr(uint8_t report[])
+ usb_hid_generic_send(USB_HID_GenericReportIn_t* report) to usb_hid_generic_send(uint8_t report[], uint32_t length)

## 0.8.1 [23 April 2013] ##

- 'main' entry point moved to board-specific files ('src/boards/*')
- Removed main.c from src root
- LPCXpresso/Red Suite project files now default to the LPC1347
- Moved messages.c to drivers/rf/chibi
- Removed some unnecessary files
- Added binary.h to simplify binary access across toolchains (removed '0b' references)
- Added 'get_fattime' to board files (get timestamp for FAT32 and SD cards)
- Moved board selection from projectconfig.h to the make file and IDE project properties

## 0.8.0 [2 April 2013] ##

- First public release
