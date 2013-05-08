/**************************************************************************/
/*!
    @file     iir_i.c

    @code
    iir_i_t iir;

    iir_i_init(&iir, 20);

    iir_i_add(&iir, 10);
    iir_i_add(&iir, 20);
    iir_i_add(&iir, 30);
    iir_i_add(&iir, 35);

    printf("SAMPLES  : %d       \n", iir.k);
    printf("AVG      : %d       \n", iir.avg);
    printf("\n");

    @endcode
 */
/**************************************************************************/
#include "iir_i.h"

/**************************************************************************/
/*!
     @brief Initialises the iir_i_t instance

     @param[in]  iir
                 Pointer to the iir_i_t instances
     @param[in]  alpha
                 8-bit (0..255) alpha value to adjust the 'effect' of the
                 filter(smaller value = slower response).

     @note       Use an x^2 value for alpha for best results, since the
                 division operation can be swapped out with a shift (ex.:
                 set alpha to 1, 2, 4, 8, 16, 32, 64, or 128).

     @note       An alpha of 255 effectively disables the filter (no
                 filtering occurs!), and an alpha of 0 is infinitely
                 'heavy', in the sense that the original value will never
                 change.
*/
/**************************************************************************/
void iir_i_init(iir_i_t *iir, uint8_t alpha)
{
  iir->k = 0;
  iir->alpha = alpha;
  iir->avg = 0;
}

/**************************************************************************/
/*!
     @brief Adds a new record to the iir_i_t instances

     @param[in]  iir
                 Pointer to the iir_i_t instances
     @param[in]  x
                 Value to insert
*/
/**************************************************************************/
void iir_i_add(iir_i_t *iir, int32_t x)
{
  iir->k++;
  if (1 == iir->k)
  {
    iir->avg = x;
  }
  else
  {
    /* IIR Filter */
    iir->avg = (x * iir->alpha + iir->avg * (256 - iir->alpha)) / 256;
  }
}
