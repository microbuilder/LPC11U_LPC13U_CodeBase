# Where's main.c ? #

To make it easier to use the code base across a number of projects, the 'int main(void)' function is placed inside a board-specific file in '/src/boards/board_name/', along with a header file containing all of the configuration flags for that project.

# OK ... So How Do I Select a Board? #

To select a target board you simply need to add a single #define somewhere in your project, matching an entry in the global **projectconfig.h** config file.

If you are using the **Makefile**, the target board is set using the following flag at the top of the make file:
```
    # See projectconfig.h for a list of valid BOARD options!
    BOARD=CFG_BRD_LPCXPRESSO_LPC1347
```
If you are using the **LPCXpresso or RedSuite** (or any Eclipse-based) IDE, you can select the appropriate target board by right-clicking on your project and selecting the correct 'Build Configuration'.

If you are using **Crossworks for ARM**, the board is defined in the Solution properties, in the 'Preprocessor Definitions' field (eg.: CFG\_BRD\_LPCXPRESSO\_LPC1347).

**The target board macro must match one of the entries in projectconfig.h!**.  This file is included in every source file in the code base, ensuring that the correct board config settings are used when compiling the code base. Current board definitions (as of v0.9.2 of the case base) can been seen below:
```
    #if defined(CFG_BRD_LPCXPRESSO_LPC1347)
      #include "boards/lpcxpresso1347/board_lpcxpresso1347.h"
    #elif defined(CFG_BRD_RF1GHZNODE)
       #include "boards/rf1ghznode/board_rf1ghznode.h"
    #elif defined(CFG_BRD_RF1GHZUSB)
      #include "boards/rf1ghzusb/board_rf1ghzusb.h"
    #elif defined (CFG_BRD_LPCNFC)
      #include "boards/lpcnfc/board_lpcnfc.h"
    #elif defined (CFG_BRD_LPCSTEPPER)
      #include "boards/lpcstepper/board_lpcstepper.h"
    #elif defined (CFG_BRD_SIMULATOR)
      #include "boards/simulator/board_simulator.h"
    #else
      #error "No CFG_BRD_* has been defined"
    #endif
```

# Source Code Organisation #

To maintain a basic structure in the code base, the code is divided into several folders:

###src/boards###
Board or project-specific code, including the board-specific 'main' function that serves as the entry point for your projects, and any board/project-specific config settings.  See **projectconfig.h** for more information on selecting a target board.

###src/cli###

Core code for the text-based command-line interface. This is only used if **CFG\_INTERFACE** is enabled in your board config file, in combination with either **CFG\_PRINTF\_USBCDC** or **CFG\_PRINTF\_UART**.

###src/core###
Drivers for the core peripherals on the MCU, such as GPIO, USB, ADC, the delay timer, etc.

###src/drivers###
Drivers for any HW other than the core MCU peripherals, such as various sensors or displays, and some helper functions like code to bit-bang SPI, simple DSP filters, etc.

###src/localisation###
A rudimentary attempt at providing localised text to allow multiple languages in one firmware build (experimental code, and likely to change in future releases)

###src/protocol###
A simple binary protocol to handle commands via binary serial buses like USB, SPI or I2C. This is still in development and should be considered experimental code, but the goal is to eventually provide an easy way to interact with a PC via USB using a class like USB HID or via another MCU with SPI.
