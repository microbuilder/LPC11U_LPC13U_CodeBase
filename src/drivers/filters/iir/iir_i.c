/**************************************************************************/
/*!
    @file     iir_i.c
    @brief    A memory efficient single pole low pass filter using integer
              values

    @code
    iir_i_t iir;

    // Initialise the IIR filter with an alpha of 64 (=0.0625)
    iir_i_init(&iir, 64);

    // Add four samples, with the first sample used at the starting value
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
                 Pointer to the iir_i_t instance
     @param[in]  alpha
                 8-bit (0..255) alpha value to adjust the 'effect' of the
                 filter(smaller value = slower response).

     @note       Use a ^2 value for alpha for best results, since the
                 division operation can be swapped out with a shift.

                 For example:

                 8-bit Alpha  Float equivalent
                 -----------  ----------------
                           1  0.00390625
                           2  0.0078125
                           4  0.015625
                           8  0.03125
                          16  0.0625
                          32  0.125
                          64  0.25
                         128  0.5

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
  int64_t xl = x;     /* Promote to 64-bit to avoid overflow issues */
  iir->k++;
  if (1 == iir->k)
  {
    iir->avg = x;
  }
  else
  {
    /* IIR Filter */
    iir->avg = (int32_t)((xl * iir->alpha + iir->avg * (256 - iir->alpha)) / 256);
  }
}
