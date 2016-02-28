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

/* 2-pole integer IIR filter data structure */
typedef struct
{
  int32_t x1;
  int32_t x2;
  int32_t out;
  int32_t y1;
  int32_t y2;
  uint8_t af;
  int32_t a1;
  int32_t a2;
  uint8_t bf;
  int32_t b0;
  int32_t b1;
  int32_t b2;
} iir_filt_i_2p_instance;

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

/* 4-pole IIR filter data structure */
typedef struct
{
  float32_t x1;
  float32_t x2;
  float32_t x3;
  float32_t x4;
  float32_t out;
  float32_t y1;
  float32_t y2;
  float32_t y3;
  float32_t y4;
  float32_t a1;
  float32_t a2;
  float32_t a3;
  float32_t a4;
  float32_t b0;
  float32_t b1;
  float32_t b2;
  float32_t b3;
  float32_t b4;
} iir_filt_4p_instance;

float32_t iir_filt_1p   ( iir_filt_1p_instance* filt, float32_t in );
float32_t iir_filt_2p   ( iir_filt_2p_instance* filt, float32_t in );
int32_t   iir_filt_i_2p ( iir_filt_i_2p_instance* filt, int32_t in );
float32_t iir_filt_3p   ( iir_filt_3p_instance* filt, float32_t in );
float32_t iir_filt_4p   ( iir_filt_4p_instance* filt, float32_t in );

void      iir_butter2 ( iir_filt_2p_instance* filt, float32_t fs, float32_t fc );

#ifdef __cplusplus
}
#endif

#endif
