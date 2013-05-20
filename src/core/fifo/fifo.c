/**************************************************************************/
/*!
    @file     fifo.c
    @author   Thach Ha (tinyusb.org)

    @section DESCRIPTION

    Light-weight FIFO buffer with basic mutex support

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2012, K. Townsend (microBuilder.eu)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#include <string.h>
#include "fifo.h"

static inline void mutex_lock   (fifo_t* f) INLINE_POST;
static inline void mutex_unlock (fifo_t* f) INLINE_POST;
static inline bool is_fifo_initalized(fifo_t* f) INLINE_POST;

/**************************************************************************/
/*!
    @brief Initialises the FIFO buffer

    @param[in]  f
                Pointer to the fifo_t object to intiialize
    @param[in]  buffer
                Pointer to the buffer's location in memory
    @param[in]  size
                The buffer size in bytes
    @param[in]  overwritable
                Set to TRUE is the FIFO is overwritable when the FIFO
                is full (the first element will be overwritten)
    @param[in]  irq
                The IRQ number to disable for MUTEX protection.
                Set the -1 if not required.

    @returns TRUE if the initialisation was successful

    @code
    static uint8_t buffer[512];  // Create FIFO buffer in memory
    static fifo_t ff;

    // Initialise a non-overwriteable buffer with MUTEX on USB IRQ
    fifo_init (&ff, buffer, 512, false, USB_IRQn);
    @endcode
*/
/**************************************************************************/
//bool fifo_init(fifo_t* f, uint8_t* buffer, uint16_t size, bool overwritable, IRQn_Type irq)
//{
//  ASSERT(size > 0, false);
//
//  f->buf = buffer;
//  f->depth = size;
//  f->rd_idx = f->wr_idx = f->count = 0;
//  f->overwritable = overwritable;
//  f->irq = irq;
//
//  return true;
//}

/**************************************************************************/
/*!
    @brief Read one byte out of the RX buffer.

    This function will return the byte located at the array index of the
    read pointer, and then increment the read pointer index.  If the read
    pointer exceeds the maximum buffer size, it will roll over to zero.

    @param[in]  f
                Pointer to the FIFO buffer to manipulate
    @param[in]  data
                Pointer to the place holder for data read from the buffer

    @returns TRUE if the queue is not empty
*/
/**************************************************************************/
bool fifo_read(fifo_t* f, void * p_buffer)
{
  if( !is_fifo_initalized(f) || fifo_isEmpty(f) )
  {
    return false;
  }

  mutex_lock(f);

  memcpy(p_buffer,
         f->buffer + (f->rd_idx * f->item_size),
         f->item_size);
  f->rd_idx = (f->rd_idx + 1) % f->depth;
  f->count--;

  mutex_unlock(f);

  return true;
}

/**************************************************************************/
/*!
    @brief Read a byte array from FIFO

    @param[in]  f
                Pointer to the FIFO buffer to manipulate
    @param[in]  rx
                Pointer to the place holder for data read from the buffer
    @param[in]  maxlen
                The maximum number of bytes to read from the FIFO

    @returns The actual number of bytes read from the FIFO
 */
/**************************************************************************/
uint16_t fifo_readArray(fifo_t* f, void * p_buffer, uint16_t maxlen)
{
  uint16_t count = 0;
  
  while ( count < maxlen && fifo_read(f, p_buffer) )
  {
    count++;
    p_buffer += f->item_size;
  }
  
  return count;
}

bool fifo_peek(fifo_t* f, uint16_t position, void * p_buffer)
{
  if( !is_fifo_initalized(f) || fifo_isEmpty(f) || (position >= f->count) )
  {
    return false;
  }

  uint16_t index = (f->rd_idx + position) % f->depth; // rd_idx is position=0
  memcpy(p_buffer,
         f->buffer + (index * f->item_size),
         f->item_size);

  return true;
}

/**************************************************************************/
/*!
    @brief Write one byte into the RX buffer.

    This function will write one byte into the array index specified by
    the write pointer and increment the write index. If the write index
    exceeds the max buffer size, then it will roll over to zero.

    @param[in]  f
                Pointer to the FIFO buffer to manipulate
    @param[in]  data
                The byte to add to the FIFO

    @returns TRUE if the data was written to the FIFO (overwrittable
             FIFO will always return TRUE)
*/
/**************************************************************************/
bool fifo_write(fifo_t* f, void const * p_data)
{
  if ( !is_fifo_initalized(f) || (fifo_isFull(f) && !f->overwritable) )
  {
    return false;
  }

  mutex_lock(f);

  memcpy( f->buffer + (f->wr_idx * f->item_size),
          p_data,
          f->item_size);

  f->wr_idx = (f->wr_idx + 1) % f->depth;

  if (fifo_isFull(f))
  {
    f->rd_idx = f->wr_idx; // keep the full state (rd == wr && len = size)
  }else
  {
    f->count++;
  }

  mutex_unlock(f);

  return true;
}

/**************************************************************************/
/*!
    @brief Clear the fifo read and write pointers and set length to zero

    @param[in]  f
                Pointer to the FIFO buffer to manipulate
*/
/**************************************************************************/
void fifo_clear(fifo_t *f)
{
  mutex_lock(f);

  f->rd_idx = f->wr_idx = f->count = 0;

  mutex_unlock(f);
}

//--------------------------------------------------------------------+
// HELPER FUNCTIONS
//--------------------------------------------------------------------+

/**************************************************************************/
/*!
    @brief Disables the IRQ specified in the FIFO's 'irq' field
           to prevent reads/write issues with interrupts

    @param[in]  f
                Pointer to the FIFO that should be protected
*/
/**************************************************************************/
static inline void mutex_lock (fifo_t* f)
{
  if (f->irq > 0)
  {
    #ifndef _TEST_
    NVIC_DisableIRQ(f->irq);
    #endif
  }
}

/**************************************************************************/
/*!
    @brief Re-enables the IRQ specified in the FIFO's 'irq' field

    @param[in]  f
                Pointer to the FIFO that should be protected
*/
/**************************************************************************/
static inline void mutex_unlock (fifo_t* f)
{
  if (f->irq > 0)
  {
    #ifndef _TEST_
    NVIC_EnableIRQ(f->irq);
    #endif
  }
}

static inline bool is_fifo_initalized(fifo_t* f)
{
  if( f->buffer == NULL || f->depth == 0 || f->item_size == 0)
  {
    return false;
  }else
  {
    return true;
  }
}
