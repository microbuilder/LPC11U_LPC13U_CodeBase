/**************************************************************************/
/*!
    @file     avg_d.h
    @author   http://www.strchr.com/standard_deviation_in_one_pass
*/
/**************************************************************************/
#ifndef __avg_D_H__
#define __avg_D_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

typedef struct avg_d_s
{
  size_t k;    /**< Sample count */
  double Mk;   /**< Mean/Average */
  double Qk;   /**< Standard variance */
} avg_d_t;

void   avg_d_init ( avg_d_t *stats );
void   avg_d_record ( avg_d_t *stats, double x );
double avg_d_stdev ( avg_d_t *stats );
double avg_d_variance ( avg_d_t *stats );
double avg_d_stdvar ( avg_d_t *stats );

#ifdef __cplusplus
}
#endif 

#endif
