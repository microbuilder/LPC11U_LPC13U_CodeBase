/**************************************************************************/
/*!
    @file     avg_f.h
    @author   http://www.strchr.com/standard_deviation_in_one_pass
*/
/**************************************************************************/
#ifndef __avg_F_H__
#define __avg_F_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

typedef struct avg_f_s
{
  size_t k;    /**< Sample count */
  float  Mk;   /**< Mean/Average */
  float  Qk;   /**< Standard variance */
} avg_f_t;

void  avg_f_init ( avg_f_t *stats );
void  avg_f_record ( avg_f_t *stats, float x );
float avg_f_stdev ( avg_f_t *stats );
float avg_f_variance ( avg_f_t *stats );
float avg_f_stdvar ( avg_f_t *stats );

#ifdef __cplusplus
}
#endif 

#endif
