/**************************************************************************/
/*!
    @file     ma_f.h
*/
/**************************************************************************/
#ifndef __MA_F_H__
#define __MA_F_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

typedef struct ma_f_s
{
  uint32_t k;       /**< Total number of samples processed so far     */
  uint16_t  size;   /**< Window size (number of samples to average)   */
  float    avg;     /**< Current average                              */
  // fifo_t*  buffer;  /**< Pointer to the circular buffer (size=window) */
} ma_f_t;

error_t ma_f_init ( ma_f_t *ma );
void    ma_f_add  ( ma_f_t *ma, float x );

#ifdef __cplusplus
}
#endif 

#endif
