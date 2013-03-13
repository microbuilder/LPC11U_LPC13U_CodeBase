/**************************************************************************/
/*!
    @file     avg_i.c

    @code
    avg_i_t is;

    avg_i_init(&is);
    avg_i_record(&is, 10);
    avg_i_record(&is, 20);
    avg_i_record(&is, 30);
    avg_i_record(&is, 35);

    printf("SAMPLES        : %d\n", is.k);
    printf("MEAN (Average) : %i\n", is.Mk);
    printf("STDEV          : %i\n", avg_i_stdev(&is));
    printf("STVARIANCE     : %i\n", avg_i_stdvar(&is));
    printf("\n");
    @endcode
 */
/**************************************************************************/
#include "avg_i.h"

#include <math.h>

/**************************************************************************/
/*!
     @brief Initialises the avg_i_t instance

     @param[in]  stats
                 Pointer to the avg_i_t instances
*/
/**************************************************************************/
void avg_i_init(avg_i_t *stats)
{
  stats->k = 0;
}

/**************************************************************************/
/*!
     @brief Adds a new record to the avg_i_t instances

     @param[in]  stats
                 Pointer to the avg_i_t instances
     @param[in]  x
                 Value to insert
*/
/**************************************************************************/
void avg_i_record(avg_i_t *stats, int32_t x)
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
                 Pointer to the avg_i_t instances
*/
/**************************************************************************/
int32_t avg_i_stdev(avg_i_t *stats)
{
  return((int32_t)sqrt((double)(stats->Qk / stats->k)));
}

/**************************************************************************/
/*!
     @brief Calculates the standard deviation variance

     @param[in]  stats
                 Pointer to the avg_i_t instances
*/
/**************************************************************************/
int32_t avg_i_variance(avg_i_t *stats)
{
  return stats->Qk / (stats->k - 1);
}

/**************************************************************************/
/*!
     @brief Calculates the population standard deviation variance

     @param[in]  stats
                 Pointer to the avg_i_t instances
*/
/**************************************************************************/
int32_t avg_i_stdvar(avg_i_t *stats)
{
  return stats->Qk / stats->k;
}
