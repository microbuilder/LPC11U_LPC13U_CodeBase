/**************************************************************************/
/*!
    @file     fixed.h
*/
/**************************************************************************/
#ifndef _FIXED_H_
#define _FIXED_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Fixed point math functions
 * Source: http://forums.arm.com/index.php?/topic/14281-arm-fixed-point-vs-floating-point-cortex-m-3/ */

/* Example code
 *
 * fixed_t a, b, c, d, add, sub, mul, div;
 *
 * a = fixed_make(1.0f);
 * b = fixed_make(2.0f);
 * c = fixed_make(3.0f);
 * d = fixed_make(-2023.621f);
 *
 * printf("a=%f, b=%f, c=%f, d=%f\n",
 *        fixed_float(a),
 *        fixed_float(b),
 *        fixed_float(c),
 *        fixed_float(d));
 *
 * add = fixed_add(d, b);
 * sub = fixed_sub(a, c);
 * mul = fixed_mul(d, c);
 * div = fixed_div(a, c);
 *
 * printf("d+b=%f, a-c=%f, d*c=%f, a/c=%f\n",
 *        fixed_float(add),
 *        fixed_float(sub),
 *        fixed_float(mul),
 *        fixed_float(div)); */

typedef int32_t fixed_t;

/* Adjust fixed_FRAC to set range
 *
 * fixed_FRAC=1  (Q31.1)  : min = -1.07374e+09, max =  1.07374e+09, step =  0.5
 * fixed_FRAC=8  (Q24.8)  : min = -8.38861e+06, max =  8.38861e+06, step =  0.00390625
 * fixed_FRAC=16 (Q16.16) : min = -32768,       max =  32768,       step =  1.52588e-05
 * fixed_FRAC=24 (Q8.24)  : min = -128,         max =  128,         step =  5.96046e-08
 * fixed_FRAC=31 (Q1.31)  : min =  1,           max = -1,           step = -4.65661e-10
 */
#define fixed_FRAC 16

#define fixed_float(a) (a / (float)(1LL<<fixed_FRAC))
#define fixed_make(a)  ((fixed_t)(a * (1LL<<fixed_FRAC)))
#define fixed_add(a,b) (a + b)
#define fixed_sub(a,b) (a - b)
#define fixed_mul(a,b) ((fixed_t)(((int64_t)a * b) >> fixed_FRAC))
#define fixed_div(a,b) ((fixed_t)(((int64_t)a << fixed_FRAC) / b))

#ifdef __cplusplus
}
#endif

#endif
