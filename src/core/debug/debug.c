/**************************************************************************/
/*!
    @file     debug.c
    @author   K. Townsend (microBuilder.eu)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, K. Townsend (microBuilder.eu)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#include "projectconfig.h"

#include "debug.h"

#include <string.h>

REGISTER_STACK_FRAME Last_Fault_Point;

void Get_Fault_Point(uint32_t stackpointer);

/**************************************************************************/
/*!
    @brief      Dumps the NVIC priority level of every interrupt
*/
/**************************************************************************/
void debugDumpNVICPriorities(void)
{
  uint32_t i = 0;

  #if defined CFG_MCU_FAMILY_LPC11UXX
    printf("IRQ  Priority%s", CFG_PRINTF_NEWLINE);
    printf("---  --------%s", CFG_PRINTF_NEWLINE);
    for (i = 0; i < 32; i++)
    {
      printf("%3u: %u %s", (unsigned int)i, (unsigned int)NVIC_GetPriority(i), CFG_PRINTF_NEWLINE);
    }
  #elif defined CFG_MCU_FAMILY_LPC13UXX
    printf("IRQ  Priority%s", CFG_PRINTF_NEWLINE);
    printf("---  --------%s", CFG_PRINTF_NEWLINE);
    for (i = 0; i < 32; i++)
    {
      printf("%3u: %u %s", (unsigned int)i, (unsigned int)NVIC_GetPriority(i), CFG_PRINTF_NEWLINE);
    }
  #else
    #error "debug.c: No MCU defined"
  #endif
}

/**************************************************************************/
/*!
    @brief      HardFault handler.
*/
/**************************************************************************/
#if defined (__GNUC__)
__attribute__((naked)) void HardFault_Handler(void){
	register uint32_t tempSP;
	__asm volatile(" tst lr, #4	\n");
	__asm volatile(" ite eq		\n");
	__asm volatile(" mrseq %0, msp	\n" : "=r" (tempSP));
	__asm volatile(" mrsne %0, psp	\n" : "=r" (tempSP));
	__asm volatile(" push {lr}	\n");
	Get_Fault_Point(tempSP);
	//while(1);
	__asm volatile(" pop {pc}	\n");
}
#endif

/**************************************************************************/
/*!
    @brief      Get_Fault_Point. This is stack safe function. Any activities
    			inside this function will not modify stack when exits.
*/
/**************************************************************************/
void Get_Fault_Point(uint32_t stackpointer)
{
	uint32_t Last_SP;
	CFSR_T ConfigFaultStatus;
	uint32_t BusFaultAddress = 0;

	memcpy((void*)&Last_Fault_Point, (void*)stackpointer, sizeof(REGISTER_STACK_FRAME));

	if(Last_Fault_Point.xPSR & (1<<9))
	{
		Last_SP = stackpointer + 36;
	}else
	{
		Last_SP = stackpointer + 32;
	}

	/* retrieve Fault Status */
	ConfigFaultStatus.DWORDVALUE = (*((volatile unsigned long *)(0xE000ED28)));

	/*===== PROCESS BUS FAULT STATUS ======*/
	if(ConfigFaultStatus.BIT.BFARVALID)
		BusFaultAddress = (*((volatile unsigned long *)(0xE000ED38)));

	if(ConfigFaultStatus.BIT.IMPRECISERR)
	{
		/* Imprecise data access error. Write to invalid address.
		 * Look back few instructions to find exactly Fault Point PC. */
		while(1);
	}
	if(ConfigFaultStatus.BIT.PRECISERR)
	{
		/* precise data access error. Check BusFaultAddress if available. */
		while(1);
	}
	if(ConfigFaultStatus.BIT.IBUSERR)
	{
		/* a bus fault on an instruction prefetch. */
		while(1);
	}

	/*===== PROCESS USAGE FAULT STATUS ======*/
	if(ConfigFaultStatus.BIT.UNDEFINSTR)
	{
		/* Undefined instruction error. */
		while(1);
	}
	if(ConfigFaultStatus.BIT.INVSTATE)
	{
		/* Invalid EPSR.T bit or illegal EPSR.IT bits for executing instruction. */
		while(1);
	}
	if(ConfigFaultStatus.BIT.UNALIGNED)
	{
		/* Unaligned access error. */
		while(1);
	}
	if(ConfigFaultStatus.BIT.DIVBYZERO)
	{
		/* Divide by zero. */
		while(1);
	}
	/* print Fault Point data here or just watch it. */
}

