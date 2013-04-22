/**************************************************************************/
/*!
    @file     hardfault.c
*/
/**************************************************************************/

/* Use the 'naked' attribute so that C stacking is not used. */
__attribute__((naked))
void HardFault_Handler(void)
{
  /*
   * Get the appropriate stack pointer, depending on our mode,
   * and use it as the parameter to the C handler. This function
   * will never return.
   */
/* TODO temporarily disable for keil build
  __asm(  ".syntax unified\n"
                  "MOVS   R0, #4  \n"
                  "MOV    R1, LR  \n"
                  "TST    R0, R1  \n"
                  "BEQ    _MSP    \n"
                  "MRS    R0, PSP \n"
                  "B      HardFault_HandlerC      \n"
          "_MSP:  \n"
                  "MRS    R0, MSP \n"
                  "B      HardFault_HandlerC      \n"
          ".syntax divided\n") ; */
}

/**
 * HardFaultHandler_C:
 * This is called from the HardFault_HandlerAsm with a pointer the Fault stack
 * as the parameter. We can then read the values from the stack and place them
 * into local variables for ease of reading.
 * We then read the various Fault Status and Address Registers to help decode
 * cause of the fault.
 * The function ends with a BKPT instruction to force control back into the debugger
 */
void HardFault_HandlerC(unsigned long *hardfault_args)
{
  volatile unsigned long stacked_r0;
  volatile unsigned long stacked_r1;
  volatile unsigned long stacked_r2;
  volatile unsigned long stacked_r3;
  volatile unsigned long stacked_r12;
  volatile unsigned long stacked_lr;
  volatile unsigned long stacked_pc;
  volatile unsigned long stacked_psr;

  stacked_r0  = ((unsigned long)hardfault_args[0]);
  stacked_r1  = ((unsigned long)hardfault_args[1]);
  stacked_r2  = ((unsigned long)hardfault_args[2]);
  stacked_r3  = ((unsigned long)hardfault_args[3]);
  stacked_r12 = ((unsigned long)hardfault_args[4]);
  stacked_lr  = ((unsigned long)hardfault_args[5]);
  stacked_pc  = ((unsigned long)hardfault_args[6]);
  stacked_psr = ((unsigned long)hardfault_args[7]);

  __asm("BKPT #0\n") ; // Break into the debugger
}
