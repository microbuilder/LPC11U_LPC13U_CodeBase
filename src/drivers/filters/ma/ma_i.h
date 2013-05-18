/**************************************************************************/
/*!
    @file     ma_i.h
*/
/**************************************************************************/
#ifndef __MA_I_H__
#define __MA_I_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"
#include "core/fifo/fifo.h"

typedef struct ma_i_s
{
  uint32_t k;       /**< Total number of samples processed so far     */
  uint16_t size;    /**< Window size (number of samples to average)   */
  int32_t  avg;     /**< Current average                              */
  fifo_t*  buffer;  /**< Pointer to the circular buffer (size=window) */
} ma_i_t;

error_t ma_i_init ( ma_i_t *ma );
void    ma_i_add  ( ma_i_t *ma, int32_t x );

#ifdef __cplusplus
}
#endif

#endif
