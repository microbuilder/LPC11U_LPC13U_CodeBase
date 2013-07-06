/**************************************************************************/
/*!
    @file     sysdefs.h
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
#ifndef _SYSDEFS_H_
#define _SYSDEFS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

typedef unsigned char byte_t;

/* Stay compatible with ugly "windows" style for bool */
#define BOOL bool

#ifndef TRUE
  #define TRUE true
#endif

#ifndef FALSE
  #define FALSE false
#endif

/* ASM and inline function placeholders */
#ifndef ASM
  #define ASM __asm volatile
#endif

#ifndef INLINE
  #if __GNUC__ && !__GNUC_STDC_INLINE__
    #define INLINE extern inline
  #else
    #define INLINE inline
  #endif
#endif

/* GCC does not inline any functions when not optimizing unless you specify
   the 'always_inline' attribute for the function */
#ifndef INLINE_POST
  #define INLINE_POST __attribute__((always_inline))
#endif

/* SRAM placement for critical functions depends on the linker script */
#ifdef __GNUC__
  #ifdef __CROSSWORKS_ARM
    #define RAMFUNC __attribute__ ((long_call, section (".fast")))
  #else
    /* ToDo: Throws 'ignoring changed section attributes for .data' */
    // #define RAMFUNC __attribute__ ((long_call, section (".data")))
    /* Hmm ... not working from the makefile ... need to debug! */
    /* Leave it blank for now unless we're in Crossworks */
    #define RAMFUNC
  #endif
#else
  #error "No section defined for RAMFUNC in sysdefs.h"
#endif

/* NULL placeholder */
#ifndef NULL
  #define NULL ((void *) 0)
#endif

#ifdef __cplusplus
}
#endif

#endif
