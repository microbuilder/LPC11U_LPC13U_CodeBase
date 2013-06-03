/**************************************************************************/
/*!
    @file     sma_i.c
    @author   Nguyen Quang Huy, Nguyen Thien Tin
    @brief    A simple moving average filter using int32_t values

    @code

    // Declare a data buffer 8 values wide
    int32_t sma_buffer[8];

    // Now declare the filter with the window size and a buffer pointer
    sma_i_t sma = { .k = 0,
                    .size = 8,
                    .avg = 0,
                    .buffer = sma_buffer };

    // Initialise the moving average filter
    if (sma_i_init(&sma))
    {
      printf("Something failed during filter init!\n");
    }

    // Add some values
    sma_i_add(&sma, 10);
    sma_i_add(&sma, 20);
    sma_i_add(&sma, -30);
    sma_i_add(&sma, -35);
    sma_i_add(&sma, 11);
    sma_i_add(&sma, 35);
    sma_i_add(&sma, 30);
    sma_i_add(&sma, 20); // We should have an avg value starting here
    sma_i_add(&sma, 3);
    sma_i_add(&sma, 10);

    printf("WINDOW SIZE   : %d\n", sma.size);
    printf("TOTAL SAMPLES : %d\n", sma.k);
    printf("CURRENT AVG   : %d\n", sma.avg);
    printf("\n");

    @endcode
 */
/**************************************************************************/
#include "sma_i.h"
#include "core/dwt/dwt.h"

/**************************************************************************/
/*!
     @brief Initialises the sma_i_t instance

     @param[in]  sma
                 Pointer to the sma_i_t instance that includes the
                 window size, a pointer to the ring buffer (ringbuffer_t),
                 and the current average (the output value).
*/
/**************************************************************************/
error_t sma_i_init ( sma_i_t *sma )
{
  // check if the window size is valid (!= 0 and is a power of 2)
  if ((0 == sma->size) || ( sma->size & (sma->size - 1) )) return ERROR_UNEXPECTEDVALUE;

  sma->avg = 0;
  sma->k = 0;
  sma->total = 0;

  // update the exponential number
  sma->power_num = 0;
  int windowSize = sma->size;
  while (windowSize > 1)
  {
    windowSize = windowSize >> 1;
    sma->power_num++;
  }

  // Fill the buffer with zero value
  for (int i = 0; i < sma->size; i++)
  {
    *(sma->buffer + i) = 0;
  }
  return ERROR_NONE;
}

/**************************************************************************/
/*!
     @brief Adds a new value to the sma_i_t instances

     @param[in]  sma
                 Pointer to the sma_i_t instances
     @param[in]  x
                 Value to insert
*/
/**************************************************************************/
void sma_i_add(sma_i_t *sma, int32_t x)
{
  int32_t *pSource = sma->buffer + (sma->k % sma->size);

  // Subtract oldest value from the total sum
  sma->total -= *pSource;

  // Add new value into the data buffer of the filter
  *pSource = x;

  // Add new value into the total value of current window
  sma->total += x;

  // increase the total sample processed
  sma->k++;

  // Wait for 'window-size' worth of samples before averaging
  if (sma->k < sma->size)
    return;

  // Update the current average value
  sma->avg = (int32_t)(sma->total >> sma->power_num);
}
