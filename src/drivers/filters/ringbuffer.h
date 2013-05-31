/**************************************************************************/
/*!
    @file     ringbuffer.h
    @author   Nguyen Quang Huy, Nguyen Thien Tin

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, K. Townsend (microBuilder.eu)
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

#ifndef __RINGBUFFER_H__
#define __RINGBUFFER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

typedef struct _ringbuffer_t
{
           void *   const buffer    ; ///< buffer pointer
           uint16_t const depth     ; ///< max items
           uint16_t const item_size ; ///< size of each item
  volatile uint16_t wr_idx          ; ///< write pointer
} ringbuffer_t;

#define RINGBUFFER_DEF(name, ff_depth, type) \
  type name##_buffer[ff_depth];              \
  ringbuffer_t name = {                      \
      .buffer       = name##_buffer,         \
      .depth        = ff_depth,              \
      .item_size    = sizeof(type),          \
  }

static inline void ring_memcpy(void *pDestination, const void *pSource, size_t num) INLINE_POST;
static inline void ring_memcpy(void *pDestination, const void *pSource, size_t num)
{
  unsigned char *pByteDestination;
  unsigned char *pByteSource;
  unsigned int  *pAlignedSource = (unsigned int *) pSource;
  unsigned int  *pAlignedDestination = (unsigned int *) pDestination;

  // If "rBuffer->item_size" is more than 4 bytes, and both dest. and source are aligned,
  // then copy dwords
  if ((((unsigned int) pAlignedDestination & 0x3) == 0)
    && (((unsigned int) pAlignedSource & 0x3) == 0)
    && (num >= 4))
  {
    while (num)
    {
      *pAlignedDestination++ = *pAlignedSource++;
      num -= 4;
    }
  }

  // Copy remaining bytes
  pByteDestination = (unsigned char *) pAlignedDestination;
  pByteSource = (unsigned char *) pAlignedSource;
  while (num--)
  {
    *pByteDestination++ = *pByteSource++;
  }
}
/**************************************************************************/
/*!
    @brief Write one byte into the ring buffer.

    This function will write one byte into the array index specified by
    the write pointer and increment the write index with no error checking

    @param[in]  rBuffer
                Pointer to the ring buffer to manipulate
    @param[in]  pData
                Data to add to the ring buffer
*/
/**************************************************************************/
static inline void ringbuffer_write(ringbuffer_t *rBuffer, void const *pData) INLINE_POST;
static inline void ringbuffer_write(ringbuffer_t *rBuffer, void const *pData)
{
  ring_memcpy( rBuffer->buffer + (rBuffer->wr_idx * rBuffer->item_size),
               pData,
               rBuffer->item_size );

  rBuffer->wr_idx = (rBuffer->wr_idx + 1) % rBuffer->depth;
}

/**************************************************************************/
/*!
     @brief Copy the value at "position" of ring buffer to pBuffer

     @param[in]  rBuffer
                 Pointer to the ring buffer to manipulate
     @param[in]  position
                 Position of ring buffer to be read the value
     @param[in]  pBuffer
                 Pointer to place holder for data read from ring buffer
*/
/**************************************************************************/
static inline void ringbuffer_peek(ringbuffer_t *rBuffer, uint16_t position, void * pBuffer) INLINE_POST;
static inline void ringbuffer_peek(ringbuffer_t *rBuffer, uint16_t position, void * pBuffer)
{
  ring_memcpy( pBuffer,
               rBuffer->buffer + position * rBuffer->item_size,
               rBuffer->item_size );
}

/**************************************************************************/
/*!
     @brief Assign pointer "pBuffer" to the element at "position" of ring buffer
            instead of copying its value

     @param[in]  rBuffer
                 Pointer to the ring buffer to manipulate
     @param[in]  position
                 Position of ring buffer to be read the value
     @param[in]  pBuffer
                 Returned pointer
*/
/**************************************************************************/
static inline void ringbuffer_ref(ringbuffer_t *rBuffer, uint16_t position, void ** pBuffer) INLINE_POST;
static inline void ringbuffer_ref(ringbuffer_t *rBuffer, uint16_t position, void ** pBuffer)
{
  *pBuffer = rBuffer->buffer + position * rBuffer->item_size;
}

#ifdef __cplusplus
}
#endif

#endif /* __RINGBUFFER_H__ */
