/**************************************************************************/
/*!
    @file     avg_i.h
    @author   http://www.strchr.com/standard_deviation_in_one_pass
*/
/**************************************************************************/
#ifndef __avg_I_H__
#define __avg_I_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

typedef struct avg_i_s
{
  size_t k;    /**< Sample count */
  int32_t Mk;  /**< Mean/Average */
  int32_t Qk;  /**< Standard variance */
} avg_i_t;

void    avg_i_init ( avg_i_t *stats );
void    avg_i_record ( avg_i_t *stats, int32_t x );
int32_t avg_i_stdev ( avg_i_t *stats );
int32_t avg_i_variance ( avg_i_t *stats );
int32_t avg_i_stdvar ( avg_i_t *stats );

#ifdef __cplusplus
}
#endif 

#endif
