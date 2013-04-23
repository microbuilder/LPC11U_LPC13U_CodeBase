# Board/HW Abstraction Layer #

In an effort to make it easy to keep the underlying drivers and core code up to date across multiple projects (so that code only needs to be maintained in one place), a simple board abstraction layer is implemented.

All config flags and settings are defined in a board-specific config file, and three mandatory functions (defined in **board.h**) must be implemented in every board_*.c file:

- void boardLED(uint8_t state)
- void boardSleep(void)
- void boardWakeup(void)

These three functions are called at different places in the code, and allow you to switch target HW without having to change anything else in your underlying project code (aside from HW-specific functions which you should place in the board_*.c file when possible).

The program entry point ('main') is also implementd in the board_*.c file so that any board-specific requirements can be met at the board level.

## Selecting a Board ##

Board selection takes place via a mandatory 'define' that is stored in either the makefile (if you are using a GNU toolchain from the command-line) or in the IDE project settings.

The **projectconfig.h** file checks for a valid board flag, and throws an error if no valid board flag was found. This file should be 'included' (#include 'projectconfig.h') in every file in the code base, since it points to the appropriate **board_*.h** file, which defines all of your project settings.
