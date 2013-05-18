/**************************************************************************/
/*!
    @file     ma_u16.h
*/
/**************************************************************************/
#ifndef __MA_U16_H__
#define __MA_U16_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"
#include "core/fifo/fifo.h"

typedef struct ma_u16_s
{
  uint32_t k;       /**< Total number of samples processed so far     */
  uint16_t size;    /**< Window size (number of samples to average)   */
  uint16_t  avg;    /**< Current average                              */
  fifo_t*  buffer;  /**< Pointer to the circular buffer (size=window) */
} ma_u16_t;

error_t ma_u16_init ( ma_u16_t *ma );
void    ma_u16_add  ( ma_u16_t *ma, uint16_t x );

#ifdef __cplusplus
}
#endif

#endif
