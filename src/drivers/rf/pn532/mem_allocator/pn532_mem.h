/**
 * @defgroup pn532_mem.h	pn532_mem.h
 * @brief define the interface for memory allocation use within pn532 NFC library
 * This module use bget libary for memory mamagement for now, but it can be ported
 * to any other memory management by keeping its interface
 *
 *
 * Add more details about module
 * @{
 */

/**
 * @file	pn532_mem.h
 *
 * @date 	Jan 9, 2013
 * @author	mlsusr32001
 */

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
