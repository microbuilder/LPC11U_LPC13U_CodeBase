/**************************************************************************/
/*!
    @file     statistics_i.c

    @code
    statistics_i_t is;

    statistics_i_init(&is);
    statistics_i_record(&is, 10);
    statistics_i_record(&is, 20);
    statistics_i_record(&is, 30);
    statistics_i_record(&is, 35);

    while(1)
    {
      printf("SAMPLES        : %d\n", is.k);
      printf("MEAN (Average) : %i\n", is.Mk);
      printf("STDEV          : %i\n", statistics_i_stdev(&is));
      printf("STVARIANCE     : %i\n", statistics_i_stdvar(&is));
      printf("\n");
    }
    @endcode
 */
/**************************************************************************/
#include "statistics_i.h"

#include <math.h>

/**************************************************************************/
/*!
     @brief Initialises the statistics_i_t instance

     @param[in]  stats
                 Pointer to the statistics_i_t instances
*/
/**************************************************************************/
void statistics_i_init(statistics_i_t *stats)
{
  stats->k = 0;
}

/**************************************************************************/
/*!
     @brief Adds a new record to the statistics_i_t instances

     @param[in]  stats
                 Pointer to the statistics_i_t instances
     @param[in]  x
                 Value to insert
*/
/**************************************************************************/
void statistics_i_record(statistics_i_t *stats, int32_t x)
{
  stats->k++;
  if (1 == stats->k)
  {
    stats->Mk = x;
    stats->Qk = 0;
  }
  else
  {
    int32_t d = x - stats->Mk; // Actually xk - M_{k-1},
                               // as Mk was not yet updated
    stats->Qk += (stats->k - 1) * d * d / stats->k;
    stats->Mk += d / stats->k;
  }
}

/**************************************************************************/
/*!
     @brief Calculates the population standard deviation

     @param[in]  stats
                 Pointer to the statistics_i_t instances
*/
/**************************************************************************/
int32_t statistics_i_stdev(statistics_i_t *stats)
{
  return((int32_t)sqrt((double)(stats->Qk / stats->k)));
}

/**************************************************************************/
/*!
     @brief Calculates the standard deviation variance

     @param[in]  stats
                 Pointer to the statistics_i_t instances
*/
/**************************************************************************/
int32_t statistics_i_variance(statistics_i_t *stats)
{
  return stats->Qk / (stats->k - 1);
}

/**************************************************************************/
/*!
     @brief Calculates the population standard deviation variance

     @param[in]  stats
                 Pointer to the statistics_i_t instances
*/
/**************************************************************************/
int32_t statistics_i_stdvar(statistics_i_t *stats)
{
  return stats->Qk / stats->k;
}
