# LPC11U/LPC13U Code Base Documentation #

Documentation for this codebase is spread out between this folder (/doc) and some of the key folders in the codebase where README.md files are appropriately placed.

## Basic Structure ##

While every attempt has been made to organize the LPC11U/LPC13U code base in a clear, coherent way, a basic explanation of the project structure will hopefully help you understand where to get started.

The code base uses the following folder structure:

- **cmsis/**: Contains the CMSIS header files, startup code and linkers for the GCC toolchain when using 'make'.  These files are generally only used with 'make' since LPCXpresso and Crossworks for ARM both product their own makefiles, linker scripts and startup code internally.
- **doc/**: This folder, containing some basic documentation is 'markdown' format
- **src/**: All of the main source code for the code base is placed in this folder, including main.c, projectconfig.h, etc.
- **tests/**: This folder contains the [Unity](http://throwtheswitch.org/white-papers/unity-intro.html) unit testing framework, and some basic unit tests that are used to verify certain sections of the code base.  For further information, see the [Unit Tests](/tests/README.md) page.
- **tools/**: This folder contains some binary tools and additional resources that are useful when working with the code base, such as the source code for the lpcrc tool that can be used to fix the checksum of compiled binaries to work with the USB bootloader.

## Supported Toolchains ##

This codebase is targetted at GCC, and currently supports the following toolchains or IDEs.  All of these tools support development on the three main platforms (Windows, Linux, Mac), though the exact capabilities and requirements of the different tools varies.  For more information on a specific toolchain or IDE, click on the links below:

- [GCC/Makefile](toolchain_make.md)
- [LPCXpresso](toolchain_lpcxpresso.md)
- [Crossworks for ARM](toolchain_crossworks.md)
- CodeLite

## Starting a New Project with the LPC11U/LPC13U Code Base ##

The code base is organised in a way that 'attempts' to keep the code and drivers relevant across a number of projects or boards.  The intention is to provide a common code base that only needs to be updated once, and every board supported by the code base will automatically benefit from the latest code updates.

In order to accomplish this, every source file in the code base references a common [projectconfig.h](../src/projectconfig.h) file that is placed in the root 'src/' folder.

The projectconfig.h file in turn references a board-specific config file in the 'src/boards/' folder that contains all of the HW and project-specific implementation details for a single project.  You should be able to simply change your board selection in projectconfig.h, and the rest of the project will continue to work as is, redirecting things like the LED, CLI input and output, USB end points, etc., to the appropriate destination.

If you wish to start a new project, the best thing to do is to copy the matching boardxxx.c and boardxxx.h files that best match your own HW or project, add a new reference to the projectconfig.h file for them, and then modify those files.

For example, to create a new board config for something based on the LPC1347 LPCXpresso board, create a copy of [src/boards/lpcxpresso1347/board_lpcxpresso1347.c](../src/boards/lpcxpresso1347/board_lpcxpresso1347.c) and [src/boards/lpcxpresso1347/board_lpcxpresso1347.h](../src/boards/lpcxpresso1347/board_lpcxpresso1347.h) and rename them to something appropriate like board_myproject.c/h.

Open the new files up and change the names in the ifdef check at the top of the header to something unique:

```
 #ifndef __BOARD_LPCXPRESSO1347_H__
 #define __BOARD_LPCXPRESSO1347_H__
```
To:
```
 #ifndef __BOARD_MYPROJECT_H__ 
 #define __BOARD_MYPROJECT_H__
```
And change the board guard macro in the .c file to something unique:
```
 #if defined CFG_BRD_LPCXPRESSO_LPC1347
```
To:
```
 #if defined CFG_BRD_MYPROJECT
```
Next ... open up [src/projectconfig.h](../src/projectconfig.h) and add the new definition above to the board selection list, and point it to the new board config header file:
```
 /*=========================================================================
    BOARD SELECTION

    Because several boards use this code library with sometimes slightly
    different pin configuration, you will need to specify which board you
    are using by enabling one of the following definitions. The code base
    will then try to configure itself accordingly for that board.

    -----------------------------------------------------------------------*/
    // #define CFG_BRD_LPC11U24_DEBUGGER
    // #define CFG_BRD_LPCXPRESSO_LPC1347
    #define CFG_BRD_WIRELESS_STANDALONE_AT86RF212
    // #define CFG_BRD_WIRELESS_USBSTICK_AT86RF212

    #ifdef CFG_BRD_LPC11U24_DEBUGGER
      #include "boards/board_11u24debugger.h"
    #endif
    #ifdef CFG_BRD_LPCXPRESSO_LPC1347
      #include "boards/board_lpcxpresso1347.h"
    #endif
    #ifdef CFG_BRD_WIRELESS_STANDALONE_AT86RF212
      #include "boards/board_standalone_at86rf2xx.h"
    #endif
    #ifdef CFG_BRD_WIRELESS_USBSTICK_AT86RF212
      #include "boards/board_usbstick_at86rf2xx.h"
    #endif
 /*=========================================================================*/
```
To:
```
 /*=========================================================================
    BOARD SELECTION

    Because several boards use this code library with sometimes slightly
    different pin configuration, you will need to specify which board you
    are using by enabling one of the following definitions. The code base
    will then try to configure itself accordingly for that board.

    -----------------------------------------------------------------------*/
    // #define CFG_BRD_LPC11U24_DEBUGGER
    // #define CFG_BRD_LPCXPRESSO_LPC1347
    // #define CFG_BRD_WIRELESS_STANDALONE_AT86RF212
    // #define CFG_BRD_WIRELESS_USBSTICK_AT86RF212
    #define CFG_BRD_MYPROJECT

    #ifdef CFG_BRD_LPC11U24_DEBUGGER
      #include "boards/board_11u24debugger.h"
    #endif
    #ifdef CFG_BRD_LPCXPRESSO_LPC1347
      #include "boards/board_lpcxpresso1347.h"
    #endif
    #ifdef CFG_BRD_WIRELESS_STANDALONE_AT86RF212
      #include "boards/board_standalone_at86rf2xx.h"
    #endif
    #ifdef CFG_BRD_WIRELESS_USBSTICK_AT86RF212
      #include "boards/board_usbstick_at86rf2xx.h"
    #endif
    #ifdef CFG_BRD_MYPROJECT
      #include "boards/board_myproject.h"
    #endif
 /*=========================================================================*/
```
All that's left to do now if start customizing the boardInit() and related functions in the header file, and all of the various config settings in the board header!

## Other Documentation (outside /doc)##


- [Board/HW Abstraction Layer](../src/boards/README.md)
- [Command Line Interface (CLI)](../src/cli/README.md)
- [USB Support](../src/core/usb/README.md)
- [Sensor Abstraction Layer](../src/drivers/sensors/README.md)
- [Localisation Support](../src/localisation/README.md)
- [Unit Tests](../src/tests/README.md)



