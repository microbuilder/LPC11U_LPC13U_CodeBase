/**************************************************************************/
/*!
    @file    debug.c
    @author  K. Townsend (microBuilder.eu)
             Huynh Duc Hau (huynhduchau86@gmail.com)

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

#ifdef DEBUG_BUILD_RT_CALLSTACK
  #ifdef __CODE_RED
    #define __STACKTOP__ _vStackTop
  #elif  __CROSSWORKS_ARM
    #define __STACKTOP__ __stack_start__
  #else
    #define __STACKTOP__ _StackTop
  #endif
  extern unsigned int __STACKTOP__;
  extern void ResetISR(void);
  static CALLSTACK_FRAME RTCallStack[MAX_CALLSTACK_FRAME];
  static uint32_t RTCallStackIndex;
#endif

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
__attribute__((naked)) void HardFault_Handler(void)
{
  __asm volatile(" tst lr, #4     \n");
  __asm volatile(" ite eq         \n");
  __asm volatile(" mrseq r0, msp  \n");
  __asm volatile(" mrsne r0, psp  \n");
  __asm volatile(" push {lr}      \n");
  __asm volatile(" bl Get_Fault_Point \n");
  __asm volatile(" pop {pc}       \n");
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
  } else
  {
    Last_SP = stackpointer + 32;
  }

#ifdef DEBUG_BUILD_RT_CALLSTACK
  /* Build callstack. */
  debugTraverseStack(Last_SP);
#endif

  /* Retrieve Fault Status */
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
    /* Precise data access error. Check BusFaultAddress if available. */
    while(1);
  }

  if(ConfigFaultStatus.BIT.IBUSERR)
  {
    /* A bus fault on an instruction prefetch. */
    while(1);
  }

  if(ConfigFaultStatus.BIT.UNSTKERR)
  {
    /* Exception return. Check Last_SP value. */
    while(1);
  }

  if(ConfigFaultStatus.BIT.STKERR)
  {
    /* Exception entry. Check Last_SP value. */
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

  /* Wait here for any other unhandled errors */
  while(1);
}

#ifdef DEBUG_BUILD_RT_CALLSTACK
/**************************************************************************/
/*!
    @brief  Checks if a given address is in Flash Memory region.
*/
/**************************************************************************/
bool debugInFlashRegion(uint32_t address)
{
  return ((address >= (uint32_t)&ResetISR) && (address < 0x10000)) ? true : false;
}

/**************************************************************************/
/*!
    @brief  Traverses the stack from the current stack position.

            This function moves back in the stack through previous
            contexts in an attempt to build a runtime callstack.
*/
/**************************************************************************/
void debugTraverseStack(uint32_t StackPos)
{
  uint32_t CurrentStackPos;

  CurrentStackPos = StackPos;

  for(RTCallStackIndex = 0; RTCallStackIndex < 5; RTCallStackIndex++)
  {
    uint32_t CurrentStackTop;
    uint32_t *stackentry;

    if( (CurrentStackPos + MAX_STACK_SIZE_IN_FUNC) >= (uint32_t)&__STACKTOP__ )
    {
      CurrentStackTop = (uint32_t)&__STACKTOP__;
    }
    else
    {
      CurrentStackTop = (CurrentStackPos + MAX_STACK_SIZE_IN_FUNC);
    }

    for(stackentry = (uint32_t *)CurrentStackPos; stackentry < (uint32_t *)CurrentStackTop; stackentry++)
    {
      if((*stackentry >= CurrentStackPos) &&          /*|*/
         (*stackentry < CurrentStackTop) &&           /*|=> In Stack Region. */
         ((*stackentry & 0x03) == 0))                 /* Aligned by 4. */
      {
        /* R7 detected. */
        if(debugInFlashRegion(*(stackentry+1)) &&     /* In Flash Region. */
                             (*(stackentry+1) & 1))   /* Is thumb address. */
        {
          /*LR detected. */
          RTCallStack[RTCallStackIndex].R7 = *stackentry;
          RTCallStack[RTCallStackIndex].LR = *(stackentry+1);
          CurrentStackPos = *stackentry;
          break;
        }
        else if(RTCallStackIndex == 0)
        {
          RTCallStack[RTCallStackIndex].R7 = *stackentry;
          RTCallStack[RTCallStackIndex].LR = Last_Fault_Point.LR;
          CurrentStackPos = *stackentry;
          break;
        }
      }
    }

    if(stackentry == (uint32_t *)CurrentStackTop)
    {
      /* reached MAX_STACK_SIZE_IN_FUNC or grown out of stack memory. */
      /* no more information or need to adjust MAX_STACK_SIZE_IN_FUNC. */
      break;
    }
  }
}
#endif // DEBUG_BUILD_RT_CALLSTACK
