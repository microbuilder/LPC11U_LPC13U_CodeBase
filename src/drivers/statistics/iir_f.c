/**************************************************************************/
/*!
    @file     iir_f.c

    @code
    iir_f_t iir;

    iir_f_init(&iir);
    iir_f_add(&iir, 10);
    iir_f_add(&iir, 20);
    iir_f_add(&iir, 30);
    iir_f_add(&iir, 35);

    while(1)
    {
      printf("SAMPLES  : %d       \n", iir.k);
      printf("AVG      : %f       \n", iir.avg);
      printf("\n");
    }
    @endcode
 */
/**************************************************************************/
#include "iir_f.h"

/**************************************************************************/
/*!
     @brief Initialises the iir_f_t instance

     @param[in]  iir
                 Pointer to the iir_f_t instances
     @param[in]  alpha
                 alpha value to adjust the 'effect' of the filter
                 (smaller value = slower response)
*/
/**************************************************************************/
void iir_f_init(iir_f_t *iir, float alpha)
{
  iir->k = 0;
  iir->alpha = alpha;
}

/**************************************************************************/
/*!
     @brief Adds a new record to the iir_f_t instances

     @param[in]  iir
                 Pointer to the iir_f_t instances
     @param[in]  x
                 Value to insert
*/
/**************************************************************************/
void iir_f_add(iir_f_t *iir, float x)
{
  iir->k++;
  if (1 == iir->k)
  {
    iir->avg = x;
  }
  else
  {
    /* IIR Filter */
    iir->avg = iir->alpha * x + (1.0 - iir->alpha) * iir->avg;
  }
}
