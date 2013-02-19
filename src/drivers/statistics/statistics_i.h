/**************************************************************************/
/*!
    @file     statistics_i.h
    @author   http://www.strchr.com/standard_deviation_in_one_pass
*/
/**************************************************************************/
#ifndef __STATISTICS_I_H__
#define __STATISTICS_I_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

typedef struct statistics_i_s
{
  size_t k;    /**< Sample count */
  int32_t Mk;  /**< Mean/Average */
  int32_t Qk;  /**< Standard variance */
} statistics_i_t;

void    statistics_i_init ( statistics_i_t *stats );
void    statistics_i_record ( statistics_i_t *stats, int32_t x );
int32_t statistics_i_stdev ( statistics_i_t *stats );
int32_t statistics_i_variance ( statistics_i_t *stats );
int32_t statistics_i_stdvar ( statistics_i_t *stats );

#ifdef __cplusplus
}
#endif 

#endif
