# LPC11U/LPC13U Code Base #

This code base is an attempt at providing a reasonably well-organized, open-source starting point for projects based on the LPC11Uxx and LPC13Uxx family of MCUs.

## Key Features ##

It includes the following key features, which can be easily enabled or disabled via a single board-specific config file:

- [USB CDC, HID and MSC support](https://github.com/microbuilder/LPC11U_LPC13U_Codebase/tree/master/src/core/usb), including HID Keyboard and HID Mouse emulation, with any combination of devices possible up to the number of end points available on the MCU
- Easy to extend [command-line interface](https://github.com/microbuilder/LPC11U_LPC13U_Codebase/tree/master/src/cli) (CLI) with USB CDC and UART support
- [Sensor abstraction layer](https://github.com/microbuilder/LPC11U_LPC13U_Codebase/tree/master/src/drivers/sensors) where all sensors return a common descriptor and data type using standardized SI units
- Basic [localisation support](https://github.com/microbuilder/LPC11U_LPC13U_Codebase/tree/master/src/localisation), allowing multiple languages to be used in the same application
- Graphics sub-system including support for multiple font types (bitmap or anti-aliased), basic drawing functions, and a simple HW abstraction mechanism
- FAT16/32 file system support for SD cards including the option to use long names (via FatFS)
- Numerous wireless stacks, including NFC (based on the PN532) and 802.15.4 (based on the AT86RF212).
- A basic [unit testing framework](https://github.com/microbuilder/LPC11U_LPC13U_Codebase/tree/master/tests_host) suitable for embedded systems (Unity)

## Supported MCUs ##
  
This code base is designed to work transparently with the following MCUs, allowing you to select the MCU with the right price/performance/size ratio for your project without having to rewrite any underlying code:

- **LPC1347** - ARM Cortex M3, 72MHz, 64KB Flash, 8+2+2KB SRAM, 4KB EEPROM
- **LPC11U37** - ARM Cortex M0, 50MHz, 128KB Flash, 8+2KB SRAM, 4KB EEPROM
- **LPC11U24** - ARM Cortex M0, 50MHa, 32KB Flash, 8+2KB SRAM, 4KB EEPROM

## Multiple Board Support ##

In an attempt to make the code base relevant in a variety of situations, there is a basic [board abstraction layer](https://github.com/microbuilder/LPC11U_LPC13U_Codebase/tree/master/src/boards), and all config settings are board-specific.

The target board in indicated in the shared **projectconfig.h** file, which in turn  references the board-specific config and initialization code in the **'boards/'** subfolder.

## Supported IDEs/Toolchains ##

The code base contains a few dependencies on GCC extensions (notably in the localisation system), and has not been tested with any non-GCC toolchain.

At the moment the following IDEs are supported by the code base, and this list may be extended in the future:

**GCC/Makefile ('Makefile')**

The codebase includes startup code, linker scripts and a makefile to build this codebase with the cross-platform, open-source GNU/GCC toolset.  This gives you the most control over how your project is built, and allows you to build your project on any platform with support for GCC and make (*NIX, Mac OSX, Windows, etc.). [(more)](doc/toolchain_make.md)

**LPCXpresso / Code Red IDE (.cproject/.project)**

LPCXpresso is a free of charge Eclipse-based IDE based around GCC.  It's based on Code Red's commercial Red Suite IDE, but is provided free of charge by NXP Semiconductors with a debug limit up to 128Kb (you can, however, compile projects larger than this), which is within the limits of all of the chips supported by this code base.  

Inexpensive LPCXpresso development boards are available with integrated SWD debuggers that can be seperated from the MCU part of the board and used to debug any supported MCU or device. [(more)](doc/toolchain_lpcxpresso.md)

**Crossworks for ARM (CW\_*.hzp)**

Project files are also provided for Rowley Associate's popular Crossworks for ARM IDE, which is GCC based, includes an optimised standard C library, and supports a large variety of HW debuggers (including the popular J-Link from Segger). [(more)](doc/toolchain_crossworks.md)

## Current Development Status ##

This code base is still in active development, and there are almost certainly a number of improvements that can be made to it, bugs that will need to be worked out, and pieces of code that could be better organized or rewritten entirely.

The current localisation system is quite unsatisfactory, for example, but the decision was made to keep in in the code base in the hopes that other people will propose improvements to it, as well as to other parts of this code base.

Until an initial public release is made (version 1.0), the code base should be considered unstable and some reorganisation will almost certainly continue to take place in different parts of the code.  

The current code has a good overall structure, but there are still many parts that can be streamlined or reorganized (for example, reworking the UART buffer to use src/core/fifo.c instead of the older buffer from a previous code base).

## How Can I Help? ##

Quite a bit of time, effort and money has gone into producing this open source code base in the sole hope that it will make things easier for other people to get started with this well-rounded MCU family.  If you find the code base useful as is, the best thanks you can give is to contribute something useful back to it, and improve the current code base so that other people can learn from your efforts as well.

## License ##

Where possible, all code is provided under a BSD style license, but each file is individually licensed and you should ensure that you fully understand the license terms and limitations of any files you use in your project.
