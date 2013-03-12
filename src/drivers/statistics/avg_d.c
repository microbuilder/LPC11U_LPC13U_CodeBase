/**************************************************************************/
/*!
    @file     avg_d.c

    @code
    avg_d_t ds;

    avg_d_init(&ds);
    avg_d_record(&ds, 10);
    avg_d_record(&ds, 20);
    avg_d_record(&ds, 30);
    avg_d_record(&ds, 35);

    while(1)
    {
      printf("SAMPLES        : %d\n", ds.k);
      printf("MEAN (Average) : %f\n", ds.Mk);
      printf("STDEV          : %f\n", avg_d_stdev(&ds));
      printf("STVARIANCE     : %f\n", avg_d_stdvar(&ds));
      printf("\n");
    }
    @endcode
 */
/**************************************************************************/
#include "avg_d.h"

#include <math.h>

/**************************************************************************/
/*!
     @brief Initialises the avg_d_t instance

     @param[in]  stats
                 Pointer to the avg_d_t instances
*/
/**************************************************************************/
void avg_d_init(avg_d_t *stats)
{
  stats->k = 0;
}

/**************************************************************************/
/*!
     @brief Adds a new record to the avg_d_t instances

     @param[in]  stats
                 Pointer to the avg_d_t instances
     @param[in]  x
                 Value to insert
*/
/**************************************************************************/
void avg_d_record(avg_d_t *stats, double x)
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
                 Pointer to the avg_d_t instances
*/
/**************************************************************************/
double avg_d_stdev(avg_d_t *stats)
{
  return(sqrt(stats->Qk / stats->k));
}

/**************************************************************************/
/*!
     @brief Calculates the standard deviation variance

     @param[in]  stats
                 Pointer to the avg_d_t instances
*/
/**************************************************************************/
double avg_d_variance(avg_d_t *stats)
{
  return stats->Qk / (stats->k - 1);
}

/**************************************************************************/
/*!
     @brief Calculates the population standard deviation variance

     @param[in]  stats
                 Pointer to the avg_d_t instances
*/
/**************************************************************************/
double avg_d_stdvar(avg_d_t *stats)
{
  return stats->Qk / stats->k;
}
