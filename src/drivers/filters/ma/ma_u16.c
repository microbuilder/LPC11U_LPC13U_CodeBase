/**************************************************************************/
/*!
    @file     ma_u16.c
    @brief    A basic moving average filter using uint16_t values

    @code

    // Create a circular buffer 5 values wide
    FIFO_DEF(ffMavg, 5, uint16_t, true, NULL);

    // Now declare the filter with the window size and a FIFO pointer
    ma_u16_t ma = { .k = 0,
                    .size = 5,
                    .avg = 0,
                    .buffer = &ffMavg };

    // Initialise the moving average filter (mostly error checks)
    if (ma_u16_init(&ma))
    {
      printf("Something failed during filter init!\n");
    }

    // Add some values
    ma_u16_add(&ma, 10);
    ma_u16_add(&ma, 20);
    ma_u16_add(&ma, 30);
    ma_u16_add(&ma, 35);
    ma_u16_add(&ma, 11);  // We should have an avg value starting here
    ma_u16_add(&ma, 35);
    ma_u16_add(&ma, 30);
    ma_u16_add(&ma, 20);
    ma_u16_add(&ma, 3);
    ma_u16_add(&ma, 10);

    printf("WINDOW SIZE   : %d\n", ma.size);
    printf("TOTAL SAMPLES : %d\n", ma.k);
    printf("CURRENT AVG   : %d\n", ma.avg);
    printf("\n");

    @endcode
 */
/**************************************************************************/
#include "ma_u16.h"

/**************************************************************************/
/*!
     @brief Initialises the ma_u16_t instance

     @param[in]  ma
                 Pointer to the ma_u16_t instance that includes the
                 window size, a pointer to the circular buffer (fifo_t),
                 and the current average (the output value).
*/
/**************************************************************************/
error_t ma_u16_init ( ma_u16_t *ma )
{
  if (0 == ma->size) return ERROR_UNEXPECTEDVALUE;

  ma->avg = 0;
  ma->k = 0;

  return ERROR_NONE;
}

/**************************************************************************/
/*!
     @brief Adds a new record to the ma_u16_t instances

     @param[in]  ma
                 Pointer to the ma_u16_t instances
     @param[in]  x
                 Value to insert
*/
/**************************************************************************/
void ma_u16_add(ma_u16_t *ma, uint16_t x)
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
  uint16_t i;
  uint32_t total = 0;                 // Overflow prevention!
  for (i = 0; i < ma->size; i++)
  {
    uint16_t val = 0;
    fifo_peek(ma->buffer, i, &val);   // Peak since read is destructive!
    total += val;
  }

  // Update the current average value
  ma->avg = (uint16_t)(total / ma->size);
}
