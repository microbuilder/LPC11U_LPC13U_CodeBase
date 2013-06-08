/**************************************************************************/
/*!
    @file     wma_i.c
    @author   Nguyen Quang Huy, Nguyen Thien Tin
    @brief    A simple moving average filter using int32_t values

    @code

    // Declare a data buffer 8 values wide
    int32_t wma_buffer[8];
    uint8_t wma_weight[8] = {8, 8, 16, 16, 32, 32, 128, 255};

    // Now declare the filter with the window size, a buffer pointer and a weighted array
    wma_i_t wma = { .k = 0,
                    .size = 8,
                    .avg = 0,
                    .weight = wma_weight,
                    .buffer = wma_buffer };

    // Initialise the moving average filter
    if (wma_i_init(&wma))
    {
      printf("Something failed during filter init!\n");
    }

    // Add some values
    wma_i_add(&wma, 10);
    wma_i_add(&wma, 20);
    wma_i_add(&wma, -30);
    wma_i_add(&wma, -35);
    wma_i_add(&wma, 11);
    wma_i_add(&wma, 35);
    wma_i_add(&wma, 30);
    wma_i_add(&wma, 20); // We should have an avg value starting here
    wma_i_add(&wma, 3);
    wma_i_add(&wma, 10);

    printf("WINDOW SIZE   : %d\n", wma.size);
    printf("TOTAL SAMPLES : %d\n", wma.k);
    printf("CURRENT AVG   : %d\n", wma.avg);
    printf("\n");

    @endcode
 */
/**************************************************************************/
#include "wma_i.h"

/**************************************************************************/
/*!
     @brief Initialises the wma_i_t instance

     @param[in]  wma
                 Pointer to the wma_i_t instance that includes the
                 window size, a pointer to the data buffer,
                 the current average (the output value), etc.
*/
/**************************************************************************/
error_t wma_i_init ( wma_i_t *wma )
{
  // check if the window size is valid (!= 0 and is a power of 2)
  if (0 == wma->size) return ERROR_UNEXPECTEDVALUE;

  wma->avg = 0;
  wma->k = 0;

  wma->sum_weight = 0;
  for (uint16_t i = 0; i < wma->size; i++)
  {
    wma->sum_weight += wma->weight[i];
  }
  return ERROR_NONE;
}

/**************************************************************************/
/*!
     @brief Adds a new value to the wma_i_t instances

     @param[in]  wma
                 Pointer to the wma_i_t instances
     @param[in]  x
                 Value to insert
*/
/**************************************************************************/
void wma_i_add(wma_i_t *wma, int32_t x)
{
  int32_t *pSource = wma->buffer + wma->k % wma->size;

  // Add new value into the data buffer of the filter
  *pSource = x;

  // increase the total sample processed
  wma->k++;

  // Wait for 'window-size' worth of samples before averaging
  if (wma->k < wma->size)
    return;

  // Recalculate the total value over the entire buffer
  int64_t total = 0;
  uint16_t current_pos = wma->k % wma->size;
  for (uint16_t i = 0; i < wma->size; i++)
  {
    total += wma->buffer[(i + current_pos) % wma->size] * wma->weight[i];
  }

  // Update the current average value
  wma->avg = (int32_t)(total / wma->sum_weight);
}
