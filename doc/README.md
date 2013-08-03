# LPC11U/LPC13U Code Base Documentation #

Documentation for this codebase is spread out between this folder (/doc) and some of the key folders in the codebase where README.md files are appropriately placed.

## Basic Structure ##

While every attempt has been made to organize the LPC11U/LPC13U code base in a clear, coherent way, a basic explanation of the project structure will hopefully help you understand where to get started.

The code base uses the following folder structure:

- **cmsis/**: Contains the CMSIS header files, startup code and linkers for the GCC toolchain when using 'make'.  These files are generally only used with 'make' since LPCXpresso and Crossworks for ARM both product their own makefiles, linker scripts and startup code internally.
- **doc/**: This folder, containing some basic documentation is 'markdown' format
- **src/**: All of the main source code for the code base is placed in this folder, including main.c, projectconfig.h, etc.
- **tests_host/**: This folder contains the [Unity](http://throwtheswitch.org/white-papers/unity-intro.html) unit testing framework, and some basic unit tests that are used to verify certain sections of the code base. Running these tests requires Ruby, Ceedling, CMock, a local GCC toolchain, and a variety of specific config settings. This code is really only intended for local tests before code releases by the main development team, and we're not able to provide support for this given the complexity of the test system.
- **tools/**: This folder contains some binary tools and additional resources that are useful when working with the code base, such as the source code for the lpcrc tool that can be used to fix the checksum of compiled binaries to work with the USB bootloader.

## Supported Toolchains ##

This codebase is targetted at GCC, and currently supports the following toolchains or IDEs.  All of these tools support development on the three main platforms (Windows, Linux, Mac), though the exact capabilities and requirements of the different tools varies.  For more information on a specific toolchain or IDE, click on the links below:

- [GCC/Makefile](toolchain_make.md)
- [LPCXpresso](toolchain_lpcxpresso.md)
- [Crossworks for ARM](toolchain_crossworks.md)
- CodeLite

## Other Documentation (outside /doc)##


- [Board/HW Abstraction Layer](../src/boards/README.md)
- [Command Line Interface (CLI)](../src/cli/README.md)
- [USB Support](../src/core/usb/README.md)
- [Sensor Abstraction Layer](../src/drivers/sensors/README.md)
- [Localisation Support](../src/localisation/README.md)
- [Unit Tests](../src/tests/README.md)



