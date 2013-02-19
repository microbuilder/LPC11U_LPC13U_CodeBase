/**************************************************************************/
/*!
    @file     statistics_d.c

    @code
    statistics_d_t ds;

    statistics_d_init(&ds);
    statistics_d_record(&ds, 10);
    statistics_d_record(&ds, 20);
    statistics_d_record(&ds, 30);
    statistics_d_record(&ds, 35);

    while(1)
    {
      printf("SAMPLES        : %d\n", ds.k);
      printf("MEAN (Average) : %f\n", ds.Mk);
      printf("STDEV          : %f\n", statistics_d_stdev(&ds));
      printf("STVARIANCE     : %f\n", statistics_d_stdvar(&ds));
      printf("\n");
    }
    @endcode
 */
/**************************************************************************/
#include "statistics_d.h"

#include <math.h>

/**************************************************************************/
/*!
     @brief Initialises the statistics_d_t instance

     @param[in]  stats
                 Pointer to the statistics_d_t instances
*/
/**************************************************************************/
void statistics_d_init(statistics_d_t *stats)
{
  stats->k = 0;
}

/**************************************************************************/
/*!
     @brief Adds a new record to the statistics_d_t instances

     @param[in]  stats
                 Pointer to the statistics_d_t instances
     @param[in]  x
                 Value to insert
*/
/**************************************************************************/
void statistics_d_record(statistics_d_t *stats, double x)
{
  stats->k++;
  if (1 == stats->k)
  {
    stats->Mk = x;
    stats->Qk = 0;
  }
  else
  {
    double d = x - stats->Mk;  // Actually xk - M_{k-1},
                               // as Mk was not yet updated
    stats->Qk += (stats->k - 1) * d * d / stats->k;
    stats->Mk += d / stats->k;
  }
}

/**************************************************************************/
/*!
     @brief Calculates the population standard deviation

     @param[in]  stats
                 Pointer to the statistics_d_t instances
*/
/**************************************************************************/
double statistics_d_stdev(statistics_d_t *stats)
{
  return(sqrt(stats->Qk / stats->k));
}

/**************************************************************************/
/*!
     @brief Calculates the standard deviation variance

     @param[in]  stats
                 Pointer to the statistics_d_t instances
*/
/**************************************************************************/
double statistics_d_variance(statistics_d_t *stats)
{
  return stats->Qk / (stats->k - 1);
}

/**************************************************************************/
/*!
     @brief Calculates the population standard deviation variance

     @param[in]  stats
                 Pointer to the statistics_d_t instances
*/
/**************************************************************************/
double statistics_d_stdvar(statistics_d_t *stats)
{
  return stats->Qk / stats->k;
}
