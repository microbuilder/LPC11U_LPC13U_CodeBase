/**************************************************************************/
/*!
    @file     statistics_d.h
    @author   http://www.strchr.com/standard_deviation_in_one_pass
*/
/**************************************************************************/
#ifndef __STATISTICS_D_H__
#define __STATISTICS_D_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

typedef struct statistics_d_s
{
  size_t k;    /**< Sample count */
  double Mk;   /**< Mean/Average */
  double Qk;   /**< Standard variance */
} statistics_d_t;

void   statistics_d_init ( statistics_d_t *stats );
void   statistics_d_record ( statistics_d_t *stats, double x );
double statistics_d_stdev ( statistics_d_t *stats );
double statistics_d_variance ( statistics_d_t *stats );
double statistics_d_stdvar ( statistics_d_t *stats );

#ifdef __cplusplus
}
#endif 

#endif
