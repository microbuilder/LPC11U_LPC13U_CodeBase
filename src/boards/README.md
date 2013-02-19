# Board/HW Abstraction Layer #

In an effort to make it easy to keep the underlying drivers and core code up to date across multiple projects (so that code only needs to be maintained in one place), a simple board abstraction layer is implemented.

All config flags and settings are defined in a board-specific config file, and four mandatory functions (defined in **board.h**) must be implemented in every board_*.c file:

- void boardInit(void)
- void boardLED(uint8_t state)
- void boardSleep(void)
- void boardWakeup(void)

These four functions are called at different places in the code, and allow you to switch target HW without having to change anything else in your underlying project code (aside from HW-specific functions which you should place in the board_*.c file when possible).

## Selecting a Board ##

Board selection takes place in the **projectconfig.h** file, which should be 'included' (#include 'projectconfig.h') in every file in the code base.  The projectconfig.h file in turn points to the appropriate **board_*.h** file, which defines all of your project settings.
