/**************************************************************************/
/*!
    @file     fifo.h
    @author   Thach Ha

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
#ifndef __FIFO_H__
#define __FIFO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

typedef struct _fifo_t
{
           void *  const buffer    ; ///< buffer pointer
           uint16_t const depth     ; ///< max items
           uint16_t const item_size ; ///< size of each item
  volatile uint16_t count           ; ///< number of items in queue
  volatile uint16_t wr_idx          ; ///< write pointer
  volatile uint16_t rd_idx          ; ///< read pointer
  bool overwritable;
  IRQn_Type irq;
} fifo_t;

#define FIFO_DEF(name, ff_depth, type, is_overwritable, irq_mutex)\
  type name##_buffer[ff_depth];\
  fifo_t name = {\
      .buffer       = name##_buffer,\
      .depth        = ff_depth,\
      .item_size    = sizeof(type),\
      .overwritable = is_overwritable,\
      .irq          = irq_mutex\
  }

//bool fifo_init(fifo_t* f, uint8_t* buffer, uint16_t size, bool overwritable, IRQn_Type irq);
bool fifo_write(fifo_t* f, void const * p_data);
bool fifo_read(fifo_t* f, void * p_buffer);
bool fifo_peek(fifo_t* f, uint16_t position, void * p_buffer);
uint16_t fifo_readArray(fifo_t* f, void * p_buffer, uint16_t maxlen);
void fifo_clear(fifo_t *f);

static inline bool fifo_isEmpty(fifo_t* f) INLINE_POST;
static inline bool fifo_isEmpty(fifo_t* f)
{
  return (f->count == 0);
}

static inline bool fifo_isFull(fifo_t* f) INLINE_POST;
static inline bool fifo_isFull(fifo_t* f)
{
  return (f->count == f->depth);
}

static inline uint16_t fifo_getLength(fifo_t* f) INLINE_POST;
static inline uint16_t fifo_getLength(fifo_t* f)
{
  return f->count;
}

#ifdef __cplusplus
}
#endif 

#endif
