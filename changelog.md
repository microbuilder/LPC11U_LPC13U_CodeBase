# LPC11U/LPC13U Code Base - Revision History #

Major changes in the LPC11U/LPC13U code base by code base version number.

## 0.9.1 [12 July 2013] ##

- Fixed .bss placement in USB SRAM in linker scripts! (oops!)

## 0.9.0 [12 July 2013] ##

- delay.c interrupt priority changed to be one higher than the lowest level so that other interrupts can potentially be configured to use delay by setting them to the lowest level
- sensorpoll.c added to poll sensor data at fixed intervals using 16-bit timer 1 (though care needs to be taken using this!)
- Disabled both generic interrupt handlers in timer16.c since they are both potentially used elsewhere
- magnetometer.c, accelerometer.c and pressure.c generic helper functions added in the sensor abstraction layer
- Added debug.c to help with debugging in the field
- Merged CMSIS-RTOS (RTX) updates from RTX branch ... basic test works on M3 and M0, but further testing needed
- Added basic USB custom class support (fast bulk transfers)
- USB HID now shares the same API as USB custom class calls to make it easier to switch
- Various improvements to the simple binary protocol
- Removed unity tests (/tests) to make room for ceedling native tests (/tests_host)
- Removed Keil project files since it's too much of a headache to maintain
- Reorganised errors.h (certains numeric values were changed)
- Updated board config files for USB/Protocol additions
- Added CFG\_ENABLE\_TIMER32 to disable TIMER32 (interrupt handlers use a lot of flash/SRAM)
- CC3000 support added (experimental, see note below!)
- Added sysinfo command to the simple binary protocol
- Fixed a bug in iap.c (truncated serial numbers)
- **BREAKING CHANGE**: Moved all /src/drivers/rf code to technology-specific folders ('nfc', 'wifi', etc.)
- **KNOWN ISSUE**: There's a truckload of issues with the CC3000 API from TI!  It can't currently be built using the makefile, and is CW only at the moment. Be sure to disable CFG_CC3000 in our board config file until these issues can all be resolved!
- **KNOWN ISSUE**: CFG\_USB\_CUSTOM\_CLASS can not be combined with any HID classes, and you must use one or the other.  This seems to be an issue with the ROM drivers and memory managed, and placing the USB memory buffer in the main 8KB block (instead of the 2KB USB SRAM block) avoids this issue, but since the buffer needs to be aligned on a 2048 byte boundary this leads to a huge waste of memory.  Investigation ongoing, but for now avoid combining CFG\_USB\_CUSTOM\_CLASS and any HID class(es).

## 0.8.6 [14 June 2013] ##

- Added core/timer16
- Added core/delay (abstraction layer to use systick or timer16[0] for 1ms delays)
- Removed core/systick and changed all systick* calls to delay* (for RTOS compatability)
- Added flow control to uart.c
- Added faster simple moving average filter (drivers/filters/ma/sma\_*), removed old versions
- Added weighted moving average filter (/drivers/filters/ma/wma\_*)
- Renamed CMSIS startup_* files to be clearer
- Added drivers/storage/logger.c to log data to an SD card or a local file (local file is Crossworks only)
- Updated adc.c for differences between LPC11U and LPC1347
- Moved low power and 10-bit ADC mode settings to board config file
- Added custom M3 RTX library (canned M3 RTX lib from ARM generated hardfault)
- Added %f (float), %e (scientific notation) and %E (engineering notation) printf support in stdio.c (Pito)
- **KNOWN ISSUE**: CDC still sometimes fails with heavy traffic ... active debugging in progress
- **KNOWN ISSUE**: RTX tested under LPCXpresso, but not working under CW since startup code needs to be modified so license issues need to be resolved with Rowley

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
