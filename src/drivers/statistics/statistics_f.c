/**************************************************************************/
/*!
    @file     statistics_f.c

    @code
    statistics_f_t fs;

    statistics_f_init(&fs);
    statistics_f_record(&fs, 10);
    statistics_f_record(&fs, 20);
    statistics_f_record(&fs, 30);
    statistics_f_record(&fs, 35);

    while(1)
    {
      printf("SAMPLES        : %d\n", fs.k);
      printf("MEAN (Average) : %f\n", fs.Mk);
      printf("STDEV          : %f\n", statistics_f_stdev(&fs));
      printf("STVARIANCE     : %f\n", statistics_f_stdvar(&fs));
      printf("\n");
    }
    @endcode
 */
/**************************************************************************/
#include "statistics_f.h"

#include <math.h>

/**************************************************************************/
/*!
     @brief Initialises the statistics_f_t instance

     @param[in]  stats
                 Pointer to the statistics_f_t instances
*/
/**************************************************************************/
void statistics_f_init(statistics_f_t *stats)
{
  stats->k = 0;
}

/**************************************************************************/
/*!
     @brief Adds a new record to the statistics_f_t instances

     @param[in]  stats
                 Pointer to the statistics_f_t instances
     @param[in]  x
                 Value to insert
*/
/**************************************************************************/
void statistics_f_record(statistics_f_t *stats, float x)
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
                 Pointer to the statistics_f_t instances
*/
/**************************************************************************/
float statistics_f_stdev(statistics_f_t *stats)
{
  return((float)sqrt((double)(stats->Qk / stats->k)));
}

/**************************************************************************/
/*!
     @brief Calculates the standard deviation variance

     @param[in]  stats
                 Pointer to the statistics_f_t instances
*/
/**************************************************************************/
float statistics_f_variance(statistics_f_t *stats)
{
  return stats->Qk / (stats->k - 1);
}

/**************************************************************************/
/*!
     @brief Calculates the population standard deviation variance

     @param[in]  stats
                 Pointer to the statistics_f_t instances
*/
/**************************************************************************/
float statistics_f_stdvar(statistics_f_t *stats)
{
  return stats->Qk / stats->k;
}
