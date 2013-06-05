/**************************************************************************/
/*!
    @file     iir_u16.c
    @brief    A memory efficient single pole low pass filter using integer
              values

    @code
    iir_u16_t iir;

    // Initialise the IIR filter with an alpha of 64 (=0.0625)
    iir_u16_init(&iir, 64);

    // Add four samples, with the first sample used at the starting value
    iir_u16_add(&iir, 10);
    iir_u16_add(&iir, 20);
    iir_u16_add(&iir, 30);
    iir_u16_add(&iir, 35);

    printf("SAMPLES  : %d       \n", iir.k);
    printf("AVG      : %d       \n", iir.avg);
    printf("\n");

    @endcode
 */
/**************************************************************************/
#include "iir_u16.h"

/**************************************************************************/
/*!
     @brief Initialises the iir_u16_t instance

     @param[in]  iir
                 Pointer to the iir_u16_t instance
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
void iir_u16_init(iir_u16_t *iir, uint8_t alpha)
{
  iir->k = 0;
  iir->alpha = alpha;
  iir->avg = 0;
}

/**************************************************************************/
/*!
     @brief Adds a new record to the iir_u16_t instances

     @param[in]  iir
                 Pointer to the iir_u16_t instances
     @param[in]  x
                 Value to insert
*/
/**************************************************************************/
void iir_u16_add(iir_u16_t *iir, uint16_t x)
{
  uint32_t xl = x;     /* Promote to 32-bit to avoid overflow issues */
  iir->k++;
  if (1 == iir->k)
  {
    iir->avg = x;
  }
  else
  {
    /* IIR Filter */
    iir->avg = (uint16_t)((xl * iir->alpha + iir->avg * (256 - iir->alpha)) / 256);
  }
}
