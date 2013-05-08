/**************************************************************************/
/*!
    @file     iir_f.h
*/
/**************************************************************************/
#ifndef __IIR_F_H__
#define __IIR_F_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

typedef struct iir_f_s
{
  float alpha;
  size_t k;       /**< Sample count */
  float  avg;
} iir_f_t;

void  iir_f_init ( iir_f_t *iir, float alpha );
void  iir_f_add  ( iir_f_t *iir, float x );

#ifdef __cplusplus
}
#endif 

#endif
