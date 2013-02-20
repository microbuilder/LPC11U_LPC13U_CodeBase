/**************************************************************************/
/*!
    @file pn532_mem.c

    @brief Define the memory allocation interface within the PN532 NFC
    library.  This module use bget for memory mamagement by default, but
    it can be ported to any other memory management by keeping the same
    interface.

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
#include "projectconfig.h"

#ifdef CFG_PN532

#include "../pn532.h"
#include "pn532_mem.h"

/* Memory pool size in bytes */
#define MEM_POOL_BYTES_SIZE     (512)

/* Memory pool for dynamic memory allocator */
static uint32_t g_mem_pool[MEM_POOL_BYTES_SIZE/4];
static BOOL g_mem_initialised = FALSE;

pn532_error_t pn532_mem_initLocal(void)
{
    bpool((void*)g_mem_pool, (bufsize)(sizeof(g_mem_pool)));
    g_mem_initialised = TRUE;

    return PN532_ERROR_NONE;
}

pn532_error_t pn532_mem_init(uint32_t * mem_pool, uint16_t pool_size)
{
    if ((!mem_pool)||(pool_size==0))
    {
        return PN532_ERROR_MEM_INVALID_PARAM;
    }
    bpool((void*)mem_pool, (bufsize)pool_size);
    g_mem_initialised = TRUE;

    return PN532_ERROR_NONE;
}

void * pn532_mem_alloc(uint16_t size)
{
    void * alloc_mem = NULL;
    if (!g_mem_initialised)
    {
      pn532_mem_initLocal();
    }

    alloc_mem = bget(size);

    /* Debugging: Enable this to troubleshoot if mem_pool is overrun
     * during debugging.  This while() loop should be removed in
     * release builds! */
     
    // #if defined (DEBUG)
    //   while (alloc_mem == NULL);
    // #endif

    return alloc_mem;
}

void pn532_mem_free(void* mem)
{
        if (g_mem_initialised)
        {
                brel(mem);
        }

        return;
}

#endif  // #ifdef CFG_PN532
