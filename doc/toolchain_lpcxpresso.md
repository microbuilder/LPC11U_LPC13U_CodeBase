# LPCXpresso Support in the LPC11U/LPC13U Code Base #

This codebase includes project files for NXP's free Eclipse-based LPCXpresso IDE, which is based on the Code Red's commercial Red Suite.  The key advantage of LPXpresso -- aside from being free for binaries up to 128KB -- is that along with the IDE, you can purchase very low-cost LPCXpresso development board that include a full on-board HW debugger.  The HW debugger can be 'cut' from the MCU half of the board, and you can then use that debugger inside the LPCXpresso IDE to program and debug any support LPC MCU, including any HW that you develop based on this code base.


## Getting Started with LPCXpresso and the LPC11U/LPC13U Code Base ##

While teaching you how to use LPCXpresso/RedSuite or Eclipse is beyond the scope of this tutorial, this will hopefully explain some of the main issues you'll encounter when using this code base with this IDE.

The main issue is the fact that the code base supports two different chip families with two different cores: ARM Cortex M0 for the LPC11U24/LPC11U37 and ARM Cortex M3 for the LPC1347.

If you are switching between cores, of if your project files are configured for a different core than you want to use, you'll need to make a few changes to the project files, as detailed below ...

## Switching MCUs with LPCXpresso/RedSuite ##

One of the unfortunate restrictions of using a common code base for the LPC11Uxx and LPC13Uxx with LPCXpresso/RedSuite is that the MCU settings are not included in the 'target' scope, so there's not easy way to switch your project config between LPC11U and LPC13U configs.

You need to make some manual changes to your project if you want to switch from a Cortex M0 LPC11U part to a Cortex M3 LPC13xx part or vice versa. This guide will show you the steps necessary to make these changes inside the IDE.

**Step One: Selecting your MCU**

Right-click on your project name in the Project Explorer and select 'Properties'.  From there, navigate to **C/C++ Build > MCU Settings** and select your MCU:

- LPC11Uxx > LPC11U24/401
- LPC11Uxx > LPC11U37/401
- LPC13xx (12bit ADC) > LPC1347

![MCU Settings](images/CodeRed_SwitchMCU_MCUSelection.PNG?raw=true)

**Step Two: Select your Build Configuration**

Next you need to let LPCXpresso/Red Suite know which build configuration to use.

You can do this by right-clicking on the project in the Project Explorer and selecting the **Build Configurations** option, then selecting the appropriate target HW:

![MCU Settings](images/CodeRed_BuildConfiguration.png?raw=true)

## Enabling Semihost Support ##

If you wish to use semihost support in LPCXpresso/RedSuite to redirect printf output into the LPCXpresso console window, you will need to exclude the /core/libc/* files from your build, which include a custom printf implementation and cause all printf output to be redirected to src/printf-retarget.c.

You can exclude the files in the IDE by right clicking on the /core/libc folder, and selecting 'Resource Configurations... > Exclude From Build...' and then selecting the build configurations you want to exclude these files from.  

Be sure to enable semihost support as well, which can be done in the QuickStart Panel by selecting 'Quick Settings > Set Library Type > Redlib (semihost)'