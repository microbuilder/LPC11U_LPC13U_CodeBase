/*================================*/
/*=====LPC11XX GNU STARTUP========*/
/*==A CODERED COMPATIBLE STARTUP==*/
/*================================*/
#if defined (__cplusplus)
#ifdef __REDLIB__
#error Redlib does not support C++
#else
//*****************************************************************************
//
// The entry point for the C++ library startup
//
//*****************************************************************************
extern "C"
{
  extern void __libc_init_array(void);
}
#endif
#endif

#define WEAK __attribute__ ((weak))
#define ALIAS(f) __attribute__ ((weak, alias (#f)))

#include "LPC13Uxx.h"

//*****************************************************************************
#if defined (__cplusplus)
extern "C" {
#endif

extern unsigned int __data_load_addr;
extern unsigned int __data_start;
extern unsigned int __data_end;
extern unsigned int __bss_start;
extern unsigned int __bss_end;
extern unsigned int __StackTop;
#ifdef FIXED_STACKHEAP_SIZE
#define STACK_SIZE  (800)
#define HEAP_SIZE  (200)
unsigned char StackMem[STACK_SIZE] __attribute__ ((section(".stack")));
unsigned char HeapMem[HEAP_SIZE] __attribute__ ((section(".heap"), align(8)));
#endif
//*****************************************************************************
//
// Forward declaration of the default handlers. These are aliased.
// When the application defines a handler (with the same name), this will
// automatically take precedence over these weak definitions
//
//*****************************************************************************
     void Reset_Handler(void);
WEAK void NMI_Handler(void);
WEAK void HardFault_Handler(void);
WEAK void MemManage_Handler(void);
WEAK void BusFault_Handler(void);
WEAK void UsageFault_Handler(void);
WEAK void SVC_Handler(void);
WEAK void DebugMon_Handler(void);
WEAK void PendSV_Handler(void);
WEAK void SysTick_Handler(void);
WEAK void IntDefaultHandler(void);
//*****************************************************************************
//
// Forward declaration of the specific IRQ handlers. These are aliased
// to the IntDefaultHandler, which is a 'forever' loop. When the application
// defines a handler (with the same name), this will automatically take
// precedence over these weak definitions
//
//*****************************************************************************
void PIN_INT0_IRQHandler   (void) ALIAS(IntDefaultHandler);
void PIN_INT1_IRQHandler   (void) ALIAS(IntDefaultHandler);
void PIN_INT2_IRQHandler   (void) ALIAS(IntDefaultHandler);
void PIN_INT3_IRQHandler   (void) ALIAS(IntDefaultHandler);
void PIN_INT4_IRQHandler   (void) ALIAS(IntDefaultHandler);
void PIN_INT5_IRQHandler   (void) ALIAS(IntDefaultHandler);
void PIN_INT6_IRQHandler   (void) ALIAS(IntDefaultHandler);
void PIN_INT7_IRQHandler   (void) ALIAS(IntDefaultHandler);
void GINT0_IRQHandler      (void) ALIAS(IntDefaultHandler);
void GINT1_IRQHandler      (void) ALIAS(IntDefaultHandler);
void OSTIMER_IRQHandler    (void) ALIAS(IntDefaultHandler);
void SSP1_IRQHandler       (void) ALIAS(IntDefaultHandler);
void I2C_IRQHandler        (void) ALIAS(IntDefaultHandler);
void CT16B0_IRQHandler     (void) ALIAS(IntDefaultHandler);
void CT16B1_IRQHandler     (void) ALIAS(IntDefaultHandler);
void CT32B0_IRQHandler     (void) ALIAS(IntDefaultHandler);
void CT32B1_IRQHandler     (void) ALIAS(IntDefaultHandler);
void SSP0_IRQHandler       (void) ALIAS(IntDefaultHandler);
void USART_IRQHandler      (void) ALIAS(IntDefaultHandler);
void USB_IRQHandler        (void) ALIAS(IntDefaultHandler);
void USB_FIQHandler        (void) ALIAS(IntDefaultHandler);
void ADC_IRQHandler        (void) ALIAS(IntDefaultHandler);
void WDT_IRQHandler        (void) ALIAS(IntDefaultHandler);
void BOD_IRQHandler        (void) ALIAS(IntDefaultHandler);
void FMC_IRQHandler        (void) ALIAS(IntDefaultHandler);
void OSCFAIL_IRQHandler    (void) ALIAS(IntDefaultHandler);
void PVTCIRCUIT_IRQHandler (void) ALIAS(IntDefaultHandler);
void USBWakeup_IRQHandler  (void) ALIAS(IntDefaultHandler);

//*****************************************************************************
//
// The entry point for the application.
// __main() is the entry point for Redlib based applications
// main() is the entry point for Newlib based applications
//
//*****************************************************************************
#if defined (__REDLIB__)
extern void __main(void);
#else
extern int main(void);
#endif

//*****************************************************************************
#if defined (__cplusplus)
} // extern "C"
#endif
//*****************************************************************************
//
// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0000.0000.
//
//*****************************************************************************
__attribute__ ((section(".isr_vector_table")))
void (* const Vectors[])(void) = {
#ifdef FIXED_STACKHEAP_SIZE
  (void (*)(void))(StackMem + STACK_SIZE),          // The initial stack pointer
#else
  (void (*)(void))&__StackTop,
#endif
  Reset_Handler,            // The reset handler
  NMI_Handler,              // The NMI handler
  HardFault_Handler,        // The hard fault handler
  MemManage_Handler,        // The MPU fault handler
  BusFault_Handler,          // The bus fault handler
  UsageFault_Handler,        // The usage fault handler
  0,                        // Reserved
  0,                        // Reserved
  0,                        // Reserved
  0,                        // Reserved
  SVC_Handler,              // SVCall handler
  DebugMon_Handler,          // Debug monitor handler
  0,                        // Reserved
  PendSV_Handler,            // The PendSV handler
  SysTick_Handler,          // The SysTick handler

  // LPC13U External Interrupts
  PIN_INT0_IRQHandler,      // All GPIO pin can be routed to PIN_INTx
  PIN_INT1_IRQHandler,
  PIN_INT2_IRQHandler,
  PIN_INT3_IRQHandler,
  PIN_INT4_IRQHandler,
  PIN_INT5_IRQHandler,
  PIN_INT6_IRQHandler,
  PIN_INT7_IRQHandler,
  GINT0_IRQHandler,
  GINT1_IRQHandler,          // PIO0 (0:7)
  0,
  0,
  OSTIMER_IRQHandler,
  0,
  SSP1_IRQHandler,          // SSP1
  I2C_IRQHandler,            //  I2C
  CT16B0_IRQHandler,        // 16-bit Timer0
  CT16B1_IRQHandler,        // 16-bit Timer1
  CT32B0_IRQHandler,        // 32-bit Timer0
  CT32B1_IRQHandler,        // 32-bit Timer1
  SSP0_IRQHandler,          // SSP0
  USART_IRQHandler,         // USART
  USB_IRQHandler,           // USB IRQ
  USB_FIQHandler,           // USB FIQ
  ADC_IRQHandler,           // A/D Converter
  WDT_IRQHandler,           // Watchdog timer
  BOD_IRQHandler,           // Brown Out Detect
  FMC_IRQHandler,           // IP2111 Flash Memory Controller
  OSCFAIL_IRQHandler,       // OSC FAIL
  PVTCIRCUIT_IRQHandler,   // PVT CIRCUIT
  USBWakeup_IRQHandler,     // USB wake up
  0,
};

//*****************************************************************************
// Reset entry point for your code.
// Sets up a simple runtime environment and initializes the C/C++
// library.
//*****************************************************************************
__attribute__ ((section(".after_vectors")))
void Reset_Handler(void)
{
  /*
   * Only Initialize Internal SRAM
   * USB RAM is used for USB purpose
   */
  unsigned int *src, *dst;

  /* Copy data section from flash to RAM */
  src = &__data_load_addr;
  dst = &__data_start;
  while (dst < &__data_end)
    *dst++ = *src++;

  /* Zero fill the bss section */
  dst = &__bss_start;
  while (dst < &__bss_end)
    *dst++ = 0;

  SystemInit();

#if defined (__cplusplus)
  //
  // Call C++ library initialisation
  //
  __libc_init_array();
#endif

#if defined (__REDLIB__)
  // Call the Redlib library, which in turn calls main()
  __main() ;
#else
  main();
#endif
	//
	// main() shouldn't return, but if it does, we'll just enter an infinite loop
	//
	while (1) {
		;
	}
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a NMI.  This
// simply enters an infinite loop, preserving the system state for examination
// by a debugger.
//
//*****************************************************************************
__attribute__ ((section(".after_vectors")))
void NMI_Handler(void)
{
    while(1)
    {
    }
}

__attribute__ ((section(".after_vectors")))
void HardFault_Handler(void)
{
    while(1)
    {
    }
}

__attribute__ ((section(".after_vectors")))
void MemManage_Handler(void)
{
    while(1)
    {
    }
}

__attribute__ ((section(".after_vectors")))
void BusFault_Handler(void)
{
    while(1)
    {
    }
}

__attribute__ ((section(".after_vectors")))
void UsageFault_Handler(void)
{
    while(1)
    {
    }
}

__attribute__ ((section(".after_vectors")))
void SVCall_Handler(void)
{
    while(1)
    {
    }
}
__attribute__ ((section(".after_vectors")))
void DebugMon_Handler(void)
{
    while(1)
    {
    }
}

__attribute__ ((section(".after_vectors")))
void PendSV_Handler(void)
{
    while(1)
    {
    }
}
__attribute__ ((section(".after_vectors")))
void SysTick_Handler(void)
{
    while(1)
    {
    }
}

//*****************************************************************************
//
// Processor ends up here if an unexpected interrupt occurs or a handler
// is not present in the application code.
//
//*****************************************************************************
__attribute__ ((section(".after_vectors")))
void IntDefaultHandler(void)
{
    while(1)
    {
    }
}
