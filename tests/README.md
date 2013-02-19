# Unit Tests #

In an effort to improve code reliability, supported was added to the code base to run basic unit tests based around the open source Unity framework.

Since this decision was made quite late in the development process, very few unit tests exist at present, but this will be expanded as development continues, and any new code blocks will have unit tests written around them.

Both the LPCXpresso and Crossworks projects have build configurations called 'Test' that will configure the projects to build a special test firmware, using the code in '/test' instead of '/src' as the program entry point.

## Building Test Firmware in LPCXpresso/Red Suite ##

In LPCXpresso, right-click on your project, and select the following menu items to select the 'Test' build configuration:

`Build Configurations > Set Active > Test (Unit Testing)` 

## Building Test Firmware in Crossworks for ARM ##

In Crossworks, click the active config drop-down box in the Project Explorer window and select the 'Test' configuration.

## Running the Unit Tests ##

At present, all tests are run on the native HW, so the first step is to build and deploy your firmware in the usual manner.

Once the firmware has been deployed and execution has started, the firmware will wait to receive the 't' character via USB CDC or UART (depending on whether CFG\_PRINTF\_USBCDC or CFG\_PRINTF\_UART are defined in your board config file).

Once the 't' character is received, Unity will begin executing the unit tests defined in '/test', with the pass/fail results displayed in the terminal window.