/**************************************************************************/
/*!
    @file     iir_u16.h
*/
/**************************************************************************/
#ifndef __IIR_U16_H__
#define __IIR_U16_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

typedef struct iir_u16_s
{
  uint8_t  alpha;
  size_t   k;       /**< Sample count */
  uint16_t avg;
} iir_u16_t;

void  iir_u16_init ( iir_u16_t *iir, uint8_t alpha );
void  iir_u16_add  ( iir_u16_t *iir, uint16_t x );

#ifdef __cplusplus
}
#endif 

#endif
