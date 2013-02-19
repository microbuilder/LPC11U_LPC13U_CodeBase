
/**
 * @defgroup pn532_mem.c       pn532_mem.c
 * @brief define the interface for memory allocation use within pn532 NFC library
 * This module use bget libary for memory mamagement for now, but it can be ported
 * to any other memory management by keeping its interface
 *
 * Add more details about module
 * @{
 */

/**
 * @file       pn532_mem.c
 *
 * @date       Jan 9, 2013
 * @author       mlsusr32001
 */

#include "projectconfig.h"

#ifdef CFG_PN532

#include "../pn532.h"
#include "pn532_mem.h"

/*Set memory pool size, in number of byte*/
#define MEM_POOL_BYTES_SIZE 512
//#define MEM_POOL_BYTES_SIZE 4

//memory pool for dynamic memory allocator
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

        /*Debugging purpose, to troubleshoot if mem_pool is overrun during debug time.
         * this while() loop is removed in release build. Note: release build with NDEBUG
         * preprocessor is defined in project setting.
         */
//#if defined (DEBUG)
//        while (alloc_mem == NULL);
//#endif

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

/*@}*/

#endif  // #ifdef CFG_PN532
