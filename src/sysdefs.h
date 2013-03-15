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

/* NULL placeholder */
#ifndef NULL
  #define NULL ((void *) 0)
#endif

/* Fixed point math functions
 * Source: http://forums.arm.com/index.php?/topic/14281-arm-fixed-point-vs-floating-point-cortex-m-3/ */

/* Example code
 *
 * fx_t a, b, c, d, add, sub, mul, div;
 *
 * a = fx_make(1.0f);
 * b = fx_make(2.0f);
 * c = fx_make(3.0f);
 * d = fx_make(-2023.621f);
 *
 * printf("a=%f, b=%f, c=%f, d=%f\n",
 *        fx_float(a),
 *        fx_float(b),
 *        fx_float(c),
 *        fx_float(d));
 *
 * add = fx_add(d, b);
 * sub = fx_sub(a, c);
 * mul = fx_mul(d, c);
 * div = fx_div(a, c);
 *
 * printf("d+b=%f, a-c=%f, d*c=%f, a/c=%f\n",
 *        fx_float(add),
 *        fx_float(sub),
 *        fx_float(mul),
 *        fx_float(div)); */

typedef int32_t fx_t;

/* Adjust FX_FRAC to set range
 *
 * FX_FRAC=1  (Q31.1)  : min = -1.07374e+09, max =  1.07374e+09, step =  0.5
 * FX_FRAC=8  (Q24.8)  : min = -8.38861e+06, max =  8.38861e+06, step =  0.00390625
 * FX_FRAC=16 (Q16.16) : min = -32768,       max =  32768,       step =  1.52588e-05
 * FX_FRAC=24 (Q8.24)  : min = -128,         max =  128,         step =  5.96046e-08
 * FX_FRAC=31 (Q1.31)  : min =  1,           max = -1,           step = -4.65661e-10
 */
#define FX_FRAC 16

#define fx_float(a) (a / (float)(1LL<<FX_FRAC))
#define fx_make(a)  ((fx_t)(a * (1LL<<FX_FRAC)))
#define fx_add(a,b) (a + b)
#define fx_sub(a,b) (a - b)
#define fx_mul(a,b) ((fx_t)(((int64_t)a * b) >> FX_FRAC))
#define fx_div(a,b) ((fx_t)(((int64_t)a << FX_FRAC) / b))

#ifdef __cplusplus
}
#endif

#endif
