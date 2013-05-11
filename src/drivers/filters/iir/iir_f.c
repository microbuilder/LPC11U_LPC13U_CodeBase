/**************************************************************************/
/*!
    @file     iir_f.c
    @brief    A memory efficient single pole low pass filter using float
              values

    @code
    iir_f_t iir;

    iir_f_init(&iir, 0.01);

    iir_f_add(&iir, 10);
    iir_f_add(&iir, 20);
    iir_f_add(&iir, 30);
    iir_f_add(&iir, 35);

    printf("SAMPLES  : %d       \n", iir.k);
    printf("AVG      : %f       \n", iir.avg);
    printf("\n");

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
                 (smaller value = slower response).
                 
     @note       An alpha of 1.0 effectively disables the filter (no
                 filtering occurs!), and an alpha of 0.0 is infinitely
                 'heavy', in the sense that the original value will never
                 change.
*/
/**************************************************************************/
void iir_f_init(iir_f_t *iir, float alpha)
{
  if (alpha > 1.0F)
    alpha = 1.0F;
  if (alpha < 0.0F)
    alpha = 0.0F;
    
  iir->k = 0;
  iir->alpha = alpha;
  iir->avg = 0.0F;
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
