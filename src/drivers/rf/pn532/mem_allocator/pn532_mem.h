/**************************************************************************/
/*!
    @file pn532_mem.h

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
#ifndef PN532_MEM_H_
#define PN532_MEM_H_
#include "bget.h"

/**************************************************************************/
/*!
    Initialise memory management block using internal memory pool; this function need to be called when system
    start up

    @param  none

    @note                  A buffer need to be statically allocated or a memory
                           map area. If a memory area is used, make sure it is
                           free to use
*/
pn532_error_t pn532_mem_initLocal(void);

/**************************************************************************/
/*!
    Initialise memory management block; this function need to be called when system
    start up

    @param  mem_pool      Memory pool to pass to the memory management module

    @param  pool_size     Size of the pool

    @note                  A buffer need to be statically allocated or a memory
                           map area. If a memory area is used, make sure it is
                           free to use

                           PN532_ERROR_MEM_INVALID_PARAM is returned when mem_pool
                           input is null
*/
pn532_error_t pn532_mem_init(uint32_t * mem_pool, uint16_t pool_size);

/**************************************************************************/
/*!
    Tries to allocate a memory block with the input size

    @param  size number of byte to try to allocate

    @return   null: when failed to allocate
              address of the memory: when allocate successfully
*/
/**************************************************************************/
void * pn532_mem_alloc(uint16_t size);

/**************************************************************************/
/*!
    Tries to free a prealocated memory block

    @param  mem : the preallocated block to be free

*/
/**************************************************************************/
void pn532_mem_free(void* mem);

#endif /* PN532_MEM_H_ */

/*@}*/
