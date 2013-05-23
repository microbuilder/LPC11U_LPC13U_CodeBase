# Source Code Organisation #

To try to keep some semblance or organiation in the code base, The source code is divided into several folders:

- **src/boards**: Board or project-specific code, including the board-specific 'main' function that serves as the entry point for your projects, and any board/project-specific config settings.  See **projectconfig.h** for more information on selecting a target board.
- **src/cli**: Core code for the text-based command-line interface, as well as any commands included in the cli (see 'src/cli/commands').  This is only used if **CFG\_INTERFACE** is enabled in your board config file, in combination with either **CFG\_PRINTF\_USBCDC** or **CFG\_PRINTF\_UART**.
- **src/core**: Drivers for the core peripherals on the MCU, such as GPIO, USB, the delay timer, etc.
- **src/drivers**: Drivers for any HW other than the core MCU peripherals, such as various sensors or displays, and some helper functions like code to bit-bang SPI, etc.
- **src/localisation**: A rudimentary attempt at providing localised text to allow multiple languages in one firmware build (experimental code, and likely to change in future releases)
- **src/protocol**: A simple binary protocol to handle commands via binary serial buses like USB, SPI or I2C. This is still in development and should be considered experimental code. 

## Where's main.c?! ##

The normal entry point in c programs is '**int main(void)**', which is typically place in a **main.c**.

To make is easier to support many different boards and projects in the LPC11U/LPC13U code base, while sharing all 'common' code,**the 'main' function has been moved to the board-specific code in 'src/boards/*'**.

If you are using the LPC1347 LPCXpresso board, for example, you will find **main** in 'src/boards/lpcxpresso1347/board_lpcxpresso1347.c'
