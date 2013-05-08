/**************************************************************************/
/*!
    @file     iir_i.h
*/
/**************************************************************************/
#ifndef __IIR_I_H__
#define __IIR_I_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

typedef struct iir_i_s
{
  uint8_t  alpha;
  size_t   k;       /**< Sample count */
  int32_t  avg;
} iir_i_t;

void  iir_i_init ( iir_i_t *iir, uint8_t alpha );
void  iir_i_add  ( iir_i_t *iir, int32_t x );

#ifdef __cplusplus
}
#endif 

#endif
