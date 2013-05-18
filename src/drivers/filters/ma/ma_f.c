/**************************************************************************/
/*!
    @file     ma_f.c
    @brief    A basic moving average filter using float values

    @code

    // Create a circular buffer 5 values wide
    FIFO_DEF(ffMavg, 5, float, true, NULL);

    // Now declare the filter with the window size and a FIFO pointer
    ma_f_t ma = { .k = 0,
                  .size = 5,
                  .avg = 0.0F,
                  .buffer = &ffMavg };

    // Initialise the moving average filter (mostly error checks)
    if (ma_f_init(&ma))
    {
      printf("Something failed during filter init!\n");
    }

    // Add some values
    ma_f_add(&ma, 10.0F);
    ma_f_add(&ma, 20.0F);
    ma_f_add(&ma, 30.0F);
    ma_f_add(&ma, 35.0F);
    ma_f_add(&ma, 35.0F);  // We should have an avg value starting here
    ma_f_add(&ma, 35.0F);
    ma_f_add(&ma, 30.0F);
    ma_f_add(&ma, 20.0F);
    ma_f_add(&ma, 15.0F);
    ma_f_add(&ma, 10.0F);

    printf("WINDOW SIZE   : %d\n", ma.size);
    printf("TOTAL SAMPLES : %d\n", ma.k);
    printf("CURRENT AVG   : %f\n", ma.avg);
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
  // Increment the total sample count
  ma->k++;

  // Add value into the circular buffer
  fifo_write(ma->buffer, &x);

  // Wait for 'window-size' worth of samples before averaging
  if (ma->k < ma->size)
  {
    return;
  }

  // Recalculate the average over the entire buffer
  int16_t i;
  double total = 0;                   // Overflow prevention!
  for (i = 0; i < ma->size; i++)
  {
    float val = 0;
    fifo_peek(ma->buffer, i, &val);   // Peak since read is destructive!
    total += val;
  }

  // Update the current average value
  ma->avg = (float)(total / ma->size);
}
