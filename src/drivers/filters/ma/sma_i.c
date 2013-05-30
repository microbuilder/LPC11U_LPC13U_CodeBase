/**************************************************************************/
/*!
    @file     sma_i.c
    @author   Nguyen Quang Huy, Nguyen Thien Tin
    @brief    A simple moving average filter using int32_t values

    @code

    // Create a circular buffer 5 values wide
    RINGBUFFER_DEF(ffsmavg, 5, int32_t, true, NULL);

    // Now declare the filter with the window size and a FIFO pointer
    sma_i_t sma = { .k = 0,
                  .size = 5,
                  .avg = 0,
                  .buffer = &ffsmavg };

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
    sma_i_add(&sma, 11);  // We should have an avg value starting here
    sma_i_add(&sma, 35);
    sma_i_add(&sma, 30);
    sma_i_add(&sma, 20);
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
  if (0 == sma->size) return ERROR_UNEXPECTEDVALUE;

  sma->avg = 0;
  sma->k = 0;
  sma->total = 0;

  /* check if the window size is a power of 2 */
  sma->isPowerOf2 = !( sma->size & (sma->size - 1) );

  /* Calculate the exponential number */
  if (sma->isPowerOf2)
  {
  	sma->power_num = 0;
  	int window_size = sma->size;
  	while (window_size > 1)
  	{
  		window_size = window_size >> 1;
  		sma->power_num ++;
  	}
  }

  // fill the buffer with zero value
  int tmp = 0;
  for (int i = 0; i < sma->rBuffer->depth; i++)
  {
  	ringbuffer_write(sma->rBuffer, &tmp);
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
  // increase the total sample processed
  sma->k++;

#ifdef CFG_USE_REFERENCE	// Use pointer instead of copying the value
  int32_t* oldVal;

  ringbuffer_ref( sma->rBuffer, sma->rBuffer->wr_idx, &oldVal );

  // Subtract oldest value from the total sum
  sma->total -= *oldVal;

#else
  int32_t oldVal;

  ringbuffer_peek(sma->rBuffer, sma->rBuffer->wr_idx, &oldVal);	// Peek the oldest value

  // Subtract oldest value from the total sum
  sma->total -= oldVal;

#endif /* CFG_USE_REFERENCE */

  // Add new value into the circular buffer
  ringbuffer_write(sma->rBuffer, &x);

  // Add new value into the total value of current window
  sma->total += x;

  // Wait for 'window-size' worth of samples before averaging
  if (sma->k < sma->size)
	  return;

  // Update the current average value
  if (sma->isPowerOf2)
  	sma->avg = (int32_t)(sma->total >> (sma->power_num));
  else
  	sma->avg = (int32_t)(sma->total / sma->size);

}
