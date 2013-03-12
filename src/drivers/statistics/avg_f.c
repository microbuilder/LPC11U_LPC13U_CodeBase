/**************************************************************************/
/*!
    @file     avg_f.c

    @code
    avg_f_t fs;

    avg_f_init(&fs);
    avg_f_record(&fs, 10);
    avg_f_record(&fs, 20);
    avg_f_record(&fs, 30);
    avg_f_record(&fs, 35);

    while(1)
    {
      printf("SAMPLES        : %d\n", fs.k);
      printf("MEAN (Average) : %f\n", fs.Mk);
      printf("STDEV          : %f\n", avg_f_stdev(&fs));
      printf("STVARIANCE     : %f\n", avg_f_stdvar(&fs));
      printf("\n");
    }
    @endcode
 */
/**************************************************************************/
#include "avg_f.h"

#include <math.h>

/**************************************************************************/
/*!
     @brief Initialises the avg_f_t instance

     @param[in]  stats
                 Pointer to the avg_f_t instances
*/
/**************************************************************************/
void avg_f_init(avg_f_t *stats)
{
  stats->k = 0;
}

/**************************************************************************/
/*!
     @brief Adds a new record to the avg_f_t instances

     @param[in]  stats
                 Pointer to the avg_f_t instances
     @param[in]  x
                 Value to insert
*/
/**************************************************************************/
void avg_f_record(avg_f_t *stats, float x)
{
  stats->k++;
  if (1 == stats->k)
  {
    stats->Mk = x;
    stats->Qk = 0;
  }
  else
  {
    float d = x - stats->Mk;   // Actually xk - M_{k-1},
                               // as Mk was not yet updated
    stats->Qk += (stats->k - 1) * d * d / stats->k;
    stats->Mk += d / stats->k;
  }
}

/**************************************************************************/
/*!
     @brief Calculates the population standard deviation

     @param[in]  stats
                 Pointer to the avg_f_t instances
*/
/**************************************************************************/
float avg_f_stdev(avg_f_t *stats)
{
  return((float)sqrt((double)(stats->Qk / stats->k)));
}

/**************************************************************************/
/*!
     @brief Calculates the standard deviation variance

     @param[in]  stats
                 Pointer to the avg_f_t instances
*/
/**************************************************************************/
float avg_f_variance(avg_f_t *stats)
{
  return stats->Qk / (stats->k - 1);
}

/**************************************************************************/
/*!
     @brief Calculates the population standard deviation variance

     @param[in]  stats
                 Pointer to the avg_f_t instances
*/
/**************************************************************************/
float avg_f_stdvar(avg_f_t *stats)
{
  return stats->Qk / stats->k;
}
