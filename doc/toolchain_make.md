# Using Make with the LPC11U/LPC13U Code Base #

The 'make' command-line build system gives you a great deal of control over how build your projects.  You can choose the toolchain you want to build with (including one you rolled yourself), the exact library versions you wish to include, perform useful manipulations on your source code and the assembled binaries, and a lot of other tasks all from one central location or makefile.

Unfortunately, getting the most from a make-based system also requires a little bit of know-how before you can get the most from it, and if this is your first time working with GCC and embedded systems you may find it easier to start with one of the graphic IDEs to build and deploy your code.  There are project files for Eclipse and Crossworks for ARM included with the code base for exactly this reason.

## Why Make? ##

A make-based build system gives you complete control over every build parameters for your firmware, and allows you to step outside the box that your favorite IDE keeps you in.  

Graphical IDEs definately have their place -- 95% of the development for this code base was done in an IDE -- but when it comes to building production code, a make file is a good decision since it's repeatable, and you have the finest degree of control over the output.

## Which Toolchain? ##

This is a more complicate question to answer.  There are a lot of options out there for ARM cross-compilers, including [rolling your own toolchain](http://www.microbuilder.eu/Tutorials/SoftwareDevelopment/BuildingGCCToolchain.aspx).  Generally, the easiest option is going to be to go with a pre-compiled toolchain though.

The most common pre-compiled cross-compiling toolchains for ARM are:

- [GCC ARM Embedded](https://launchpad.net/gcc-arm-embedded) - Maintained by ARM, this new but popular toolchain includes a standard C library thats been optimised for small embedded ARM processors.  This is a good choice moving forward since development is active and it usually takes advantage of the latest changes to the underlying GCC code.  Installers are available for Windows, Mac and Linux.
- [Yagarto](http://www.yagarto.de/) - A popular pre-compiled toolchain for ARM available in an easy to use installer for both Windows and the Mac.  It also includes a seperate installer (YAGARTO Tools) that will add support for the tools you'll need when building like 'make' and common Linux commands like 'mkdir'.  This is an easy to use option and works well out of the box since it includes everything you need in two installers.
- [Sourcery CodeBench Lite](http://www.mentor.com/embedded-software/sourcery-tools/sourcery-codebench/editions/lite-edition/) - Formerly CodeSourcery G++ Lite before being purchased by Mentor, the Lite edition of this toolchain is still available free of charge and includes installers for Windows and Linux.

**Note: Even if you're using the GCC ARM Embedded toolchain, on Windows you'll still want to download and install [YAGARTO Tools](http://www.yagarto.de/) to have the pre-compiled binaries you'll need to run the makefile (make, mkdir, rm, etc.).**

# Getting Started #

Assuming you have a toolchain installed and available in your PATH variable, you simply need to run the following command to build your firmware:
```
make
```
Once the build process is complete and if there are no errors, you will find the compiled binaries in the /bin folder.

To clean up after a build process and remove all files generated, run:
```
make clean
```

# Configuring the Makefile #

The code base has a relatively basic makefile that should work out of the box with only one or two minor change depending on your exact requirements.

## Selecting Your Core ##

To produce the best possible code, GCC needs to know what core to create code for when running the compiler and the assembler.

Depending on if you are using a Cortex M0 (LPC11U24, LPC11U37) or a Cortex M3 (LPC1347) MCU, you need to modify the 'TARGET' definition at the top of the makefile as follows:

For LPC11U parts:
```
TARGET = lpc11u
```

For LPC1347 and related parts:
```
TARGET = lpc13u
```

## Setting the Optimisation Level ##

While it's normally a good idea to do your initial development with compiler optimisation turned off, if you wish to change the optimisation level for production builds, you simply need to assign an appropriate value to the OPTIMIZATION field below.
```
OPTIMIZATION = 0
```
See the GCC documentation for more info on optimisation levels, but valid values are:
```
 0     - No optimisation
 1/2/3 - Increasing levels of optimisation
 s     - Optimise for size
```

## Selecting Your Toolchain ##

The other step that needs to be customized is to indicate the toolchain that you will be using to cross-compile the code base.

You will need to update the 'GNU GCC compiler prefix' section in the Makefile to point to an appropriate path, or just use the generic toolchain 'prefix' if you have added to toolchain location to your system PATH variable.  

If you don't know if your toolchain is available in your system PATH just type 'arm-none-eabi-gcc --version' and you'll either see the version of your toolchain or you'll get an error.

By default, the following entries are included in the makefile:

```
 # Use the default toolchain (based on the PATH variable, etc.)
 CROSS_COMPILE = arm-none-eabi-

 # Use a toolchain at a specific location
 # CROSS_COMPILE = C:/code_red/RedSuiteNXP_5.0.12_1048/redsuite/tools/bin/arm-none-eabi-
 # CROSS_COMPILE = C:/arm/gnu4.7.2012.q4/bin/arm-none-eabi-
```
The first (default) entry assumes that arm-none-eabi-* is available in the system PATH and no hard-coded folder location is required.  If you have installed a pre-rolled toolchain like Yagarto or ARM's Embedded GCC toolchain and selected to option to add it to the path variable, this is probably what you want to select.

To use a specific toolchain and a specific location, you can optionally add the path directly to that toolchain, following the example of the two additional entries above.  This can be useful if you want to compare several toolchains to see which produces the smallest code, etc.

## ARM GCC Embedded Toolchain Requirements ##

ARM has recently started providing their own optimized toolchain and standard c library for embedded projects.  In order to take advantage of their new nano-C library (optimised for the smallest code size), you need to uncomment the following line under Compiler Options:

```
 # For use with the GCC ARM Embedded toolchain
 # GCFLAGS += --specs=nano.specs
```
This isn't mandatory, but is highly recommended since functions like printf are extremely large in the default c library.

## LPCXpresso Toolchain Requirements ##

If you are building with the LPCXpresso toolchain from the command-line, you will also need to uncomment the following sections in the makefile to include two additional libraries and add some required 'defines':

Uncomment the following line under Compiler Options
```
 # For use with the LPCXpresso toolchain
 # GCFLAGS += -D__REDLIB__ -D__CODE_RED
```

And uncomment the two addition external libraries below:
```
 # External Libraries
 LDLIBS   = -lm
 # The following libraries are required with the LPCXpresso toolchain
 # LDLIBS  += -lcr_c -lcr_eabihelpers
```

## Adding Files to the Build Process ##

It's relatively easy to add new files to the build process ... simply follow the example in the 'Source Files' section, and add the folder to the VPATH field and the file(s) to the OBJS field, paying attention to use '+=' and not '=', since using the latter will remove any previous entries.
```
VPATH += /my/path/to/src/files
OBJS  += $(OBJ_PATH)/file1.o
OBJS  += $(OBJ_PATH)/file2.o
```
