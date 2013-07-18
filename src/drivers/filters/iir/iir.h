/**************************************************************************/
/*!
    @file     iir.h
*/
/**************************************************************************/
#ifndef __IIR_H__
#define __IIR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

/* Single-pole IIR filter data structure */
typedef struct 
{
  float32_t x1;
  float32_t out;
  float32_t y1;
  float32_t a1;
  float32_t b0;
  float32_t b1;
} iir_filt_1p_instance;

/* 2-pole IIR filter data structure */
typedef struct 
{
  float32_t x1;
  float32_t x2;
  float32_t out;
  float32_t y1;
  float32_t y2;
  float32_t a1;
  float32_t a2;
  float32_t b0;
  float32_t b1;
  float32_t b2;
} iir_filt_2p_instance;

/* 3-pole IIR filter data structure */
typedef struct 
{
  float32_t x1;
  float32_t x2;
  float32_t x3;
  float32_t out;
  float32_t y1;
  float32_t y2;
  float32_t y3;
  float32_t a1;
  float32_t a2;
  float32_t a3;
  float32_t b0;
  float32_t b1;
  float32_t b2;
  float32_t b3;
} iir_filt_3p_instance;

float32_t iir_filt_1p(iir_filt_1p_instance* filt, float32_t in);
float32_t iir_filt_2p(iir_filt_2p_instance* filt, float32_t in);
float32_t iir_filt_3p(iir_filt_3p_instance* filt, float32_t in);

#ifdef __cplusplus
}
#endif 

#endif
