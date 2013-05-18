/**************************************************************************/
/*!
    @file     ma_f.c
    @brief    A basic moving average filter using float values

    @code
    ma_f_t ma;

    // Initialise a moving average filter with an 8 sample window
    ma_f_init(&ma, 8);

    ma_f_add(&ma, 10.0F);
    ma_f_add(&ma, 20.0F);
    ma_f_add(&ma, 30.0F);
    ma_f_add(&ma, 35.0F);
    ma_f_add(&ma, 35.0F);
    ma_f_add(&ma, 35.0F);
    ma_f_add(&ma, 30.0F);
    ma_f_add(&ma, 20.0F);
    ma_f_add(&ma, 15.0F);
    ma_f_add(&ma, 10.0F);

    printf("WINDOW SIZE : %d\n", ma.k);
    printf("CURRENT AVG : %f\n", ma.avg);
    printf("\n");

    @endcode
 */
/**************************************************************************/
#include "ma_f.h"

/**************************************************************************/
/*!
     @brief Initialises the ma_f_t instance

     @param[in]  ma
                 Pointer to the ma_f_t instance that includes the
                 window size, a pointer to the circular buffer (fifo_t),
                 and the current average (the output value).
*/
/**************************************************************************/
error_t ma_f_init ( ma_f_t *ma )
{
  if (0 == ma->size) return ERROR_UNEXPECTEDVALUE;

  ma->avg = 0.0F;
  ma->k = 0;

  // ToDo: Initialise circular buffer of size 'size' ... how to handle
  // this to allow a variable size, but without using something like
  // malloc (yuck)?  Always use the maximum buffer size (0xFF)?

  return ERROR_NONE;
}

/**************************************************************************/
/*!
     @brief Adds a new record to the ma_f_t instances

     @param[in]  ma
                 Pointer to the ma_f_t instances
     @param[in]  x
                 Value to insert
*/
/**************************************************************************/
void ma_f_add(ma_f_t *ma, float x)
{
  ma->k++;

  // ToDo:
  // 1. Increment ma->k (total sample count)
  // 2. Add value x into the circular buffer, deplacing first value if
  //    necessary (the circular buffer handles the wrap-around)
  // 2. If (ma->k < ma->size), exit function and wait until we have 'size'
  //    samples
  // 3. If (ma->k >= ma->size), calculate average of the last 'size'
  //    values from the circular buffer, and assign the value to
  //    ma->avg
}
