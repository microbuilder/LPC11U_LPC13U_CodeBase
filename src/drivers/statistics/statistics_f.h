/**************************************************************************/
/*!
    @file     statistics_f.h
    @author   http://www.strchr.com/standard_deviation_in_one_pass
*/
/**************************************************************************/
#ifndef __STATISTICS_F_H__
#define __STATISTICS_F_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

typedef struct statistics_f_s
{
  size_t k;    /**< Sample count */
  float  Mk;   /**< Mean/Average */
  float  Qk;   /**< Standard variance */
} statistics_f_t;

void  statistics_f_init ( statistics_f_t *stats );
void  statistics_f_record ( statistics_f_t *stats, float x );
float statistics_f_stdev ( statistics_f_t *stats );
float statistics_f_variance ( statistics_f_t *stats );
float statistics_f_stdvar ( statistics_f_t *stats );

#ifdef __cplusplus
}
#endif 

#endif
