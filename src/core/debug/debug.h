/**************************************************************************/
/*!
    @file     debug.h
    @author   K. Townsend (microBuilder.eu)
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
#ifndef _DEBUG_H_
#define _DEBUG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

/*=======================================================================
  CONFIGURABLE FAULT STATUS REGISTER
  -----------------------------------------------------------------------
  For further details see the ARM Technical Reference Manual:
  http://infocenter.arm.com/help/topic/com.arm.doc.ddi0337e/DDI0337E_cortex_m3_r1p1_trm.pdf
  -----------------------------------------------------------------------*/  
  enum 
  {
    /* Memory Management Fault Status Bit Definitions */
    CFSR_MM_IACCVIOL = 0,
    CFSR_MM_DACCVIOL,
    CFSR_MM_MUNSTKERR = 3,
    CFSR_MM_MSTKERR,
    CFSR_MM_MMARVALID = 7,
    /* Bus Fault Status Bit Definitions */
    CFSR_BF_IBUSERR,
    CFSR_BF_PRECISERR,
    CFSR_BF_IMPRECISERR,
    CFSR_BF_UNSTKERR,
    CFSR_BF_STKERR,
    CFSR_BF_BFARVALID = 15,
    /* Usage Fault Status Bit Definitions */
    CFSR_UF_UNDEFINSTR,
    CFSR_UF_INVSTATE,
    CFSR_UF_INVPC,
    CFSR_UF_NOCP,
    CFSR_UF_UNALIGNED = 24,
    CFSR_UF_DIVBYZERO
  };
  
  typedef union 
  {
    uint32_t DWORDVALUE;
    struct 
    {
      uint8_t IACCVIOL :1;      /**< Attempting to fetch an instruction from a location that does not permit execution */
      uint8_t DACCVIOL :1;      /**< Attempting to load or store at a location that does not permit the operation */
      uint8_t ReservedBit0 :1;
      uint8_t MUNSTKERR :1;     /**< Unstack from exception return has caused one or more access violations */
      uint8_t MSTKERR :1;       /**< Stacking from exception has caused one or more access violations */
      uint8_t ReservedBit1 :2;
      uint8_t MMARVALID :1;     /**< Memory Manage Address Register (MMAR) address valid flag. 1 = valid fault address in MMAR, 0 = no valid fault address in MMAR. */
      uint8_t IBUSERR :1;       /**< Instruction bus error flag: 1 = instruction bus error, 0 = no instruction bus error */
      uint8_t PRECISERR :1;     /**< Precise data bus error */
      uint8_t IMPRECISERR :1;   /**< Imprecise data bus error */
      uint8_t UNSTKERR :1;      /**< Unstack from exception return has caused one or more bus faults */
      uint8_t STKERR :1;        /**< Stacking from exception has caused one or more bus faults */
      uint8_t ReservedBit2 :2;
      uint8_t BFARVALID :1;     /**< This bit is set if the Bus Fault Address Register (BFAR) contains a valid address */
      uint8_t UNDEFINSTR :1;
      uint8_t INVSTATE :1;      /**< Invalid combination of EPSR and instruction, for reasons other than UNDEFINED instruction */
      uint8_t INVPC :1;         /**< Attempt to load EXC_RETURN into PC illegally. Invalid instruction, invalid context, invalid value. */
      uint8_t NOCP :1;          /**< Attempt to use a coprocessor instruction. The processor does not support coprocessor instructions */
      uint8_t ReservedBit3 :4;
      uint8_t UNALIGNED :1;     /**< Unaligned memory access */
      uint8_t DIVBYZERO :1;     /**< SDIV or UDIV instruction used with a divisor of 0 */
    } __attribute__((packed)) BIT;
  } CFSR_T;
/*=========================================================================*/


/*=========================================================================
    STACK FRAMES
    -----------------------------------------------------------------------
    This struct is used to encapsulate a single stack frame
    -----------------------------------------------------------------------*/
    typedef struct 
    {
      uint32_t R0, R1, R2, R3;
      uint32_t R12;
      uint32_t LR;		  /**< Last Context Link Register */
      uint32_t PC;		  /**< Last Context PC address that generated the HardFault */
      uint32_t xPSR;		/**< Last Context Program Status Register */
    } __attribute__((packed)) REGISTER_STACK_FRAME;
/*=========================================================================*/


/*=========================================================================
  CALL STACK RECONSTRUCTION
  -----------------------------------------------------------------------  
  If DEBUG_BUILD_RT_CALLSTACK is enabled, and you are using GCC with no
  optimization (-O0) GCC may (or may not) write the frame pointer to
  R7 register, which tells us where the next stack frame is located in
  the stack.
  
  Without this frame pointer, we would require access to the
  debug files to know where the next stack frame is located since local
  variables also get written into the stack.
  
  This code attempts to roll back that stack and create a call stack
  based on the R7 register contents, but this will only work with GCC
  set to -O0 (no optimisation) and when '-fomit-frame-pointer' is not
  defined!
 
  WARNING: This may or may not work! There is no guaranteee that we can
           go back more than one level in the stack during an error
           condition, and this code should be considered experimental. 
  -----------------------------------------------------------------------*/
  // #define DEBUG_BUILD_RT_CALLSTACK  
  
  #ifdef DEBUG_BUILD_RT_CALLSTACK
  typedef struct {
    uint32_t R7;    /**< Frame pointer location with GCC */
    uint32_t LR;
  } CALLSTACK_FRAME;
  
  /* Maximum stack size that compiler uses for local variables  and the */
  /* stack pointer in a function. */
  #define MAX_STACK_SIZE_IN_FUNC		(4*64)

  /* Maximum number of call stack framew to read */
  #define MAX_CALLSTACK_FRAME			  (5)
  #endif
/*=========================================================================*/

void debugDumpNVICPriorities (void);

#ifdef DEBUG_BUILD_RT_CALLSTACK
bool debugInFlashRegion ( uint32_t address );
void debugTraverseStack ( uint32_t StackPos );
#endif

#ifdef __cplusplus
}
#endif

#endif
