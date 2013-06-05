# LPC11U/LPC13U Code Base - Revision History #

Major changes in the LPC11U/LPC13U code base by code base version number.

## 0.8.6 [Ongoing] ##

- Added core/timer16
- Added core/delay
- Removed core/systick and changed all systick* calls to delay* (for RTOS compatability)
- Added flow control to uart.c
- Added faster simple moving average filter (drivers/filters/ma/sma\_i.c)
- Renamed CMSIS startup_* file to be clearer

## 0.8.5 [21 May 2013] ##

- Updated CMSIS to v3.20
- Renamed Crossworks project files to CW_*
- Renamed Keil project files to Keil_*
- Added stepper support to board config files
- Changed the clock setup in core/adc
- Added basic TCS34725 driver
- First attempt at a simple binary protocol (CFG_PROTOCOL, /src/protocol)
- Added CFG\_BRD\_SIMULATOR as a board option (mostly for unit tests)
- Fixed negative value bug in timespanCreate
- Renamed /src/drivers/statistics to /src/drivers/filters
- Added some basic Python scripts to test the IIR filter
- Updated LPCXpresso project files to use /cmsis (no more external dependencies)
- Added int32_t iir filter and matching python scripts
- Changed usb HID generic callbacks to be more general
  - Replaced 'USB\_HID\_GenericReportOut\_t' and 'USB\_HID\_GenericReportIn\_t' signatures with '(uint8\_t report[] and uint32\_t length)'
  - Affected functions are 'usb\_hid\_generic\_recv\_isr', 'usb\_hid\_generic\_report\_request\_isr', and 'usb\_hid\_generic\_send'
- Added CMSIS DSP library to the makefile, LPCXpresso and Crossworks project files
- Added RTX library for CMSIS-RTOS (currently untested)
- Removed all use of GPIOSetBitValue and GPIOSetDir (wasteful fluff)
- Added simple moving average filter and python tester
- Improved fifo_t to support any object size (previously uint8_t only)
- Added ceedling support (experimental)

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
