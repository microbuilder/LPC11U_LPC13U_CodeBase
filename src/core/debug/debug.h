/**************************************************************************/
/*!
    @file     debug.h
    @author   K. Townsend (microBuilder.eu)

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
#ifndef _DEBUG_H_
#define _DEBUG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

enum {
	/* Memory Management Fault Status Bit Definitions */
	CFSR_MM_IACCVIOL = 0,
	CFSR_MM_DACCVIOL,
	CFSR_MM_MUNSTKERR = 3,
	CFSR_MM_MSTKERR,
	CFSR_MM_MMARVALID = 7,
	/* Bus Fault Status Bit Definitions */
	CFSR_BF_IBUSERR,
	CFSR_BF_PRECISERR,
	CFSR_BF_IMPRECISERR,
	CFSR_BF_UNSTKERR,
	CFSR_BF_STKERR,
	CFSR_BF_BFARVALID = 15,
	/* Usage Fault Status Bit Definitions */
	CFSR_UF_UNDEFINSTR,
	CFSR_UF_INVSTATE,
	CFSR_UF_INVPC,
	CFSR_UF_NOCP,
	CFSR_UF_UNALIGNED = 24,
	CFSR_UF_DIVBYZERO
};

typedef union {
	uint32_t DWORDVALUE;
	struct {
		uint8_t IACCVIOL :1;
		uint8_t DACCVIOL :1;
		uint8_t ReservedBit0 :1;
		uint8_t MUNSTKERR :1;
		uint8_t MSTKERR :1;
		uint8_t ReservedBit1 :2;
		uint8_t MMARVALID :1;
		uint8_t IBUSERR :1;
		uint8_t PRECISERR :1;
		uint8_t IMPRECISERR :1;
		uint8_t UNSTKERR :1;
		uint8_t STKERR :1;
		uint8_t ReservedBit2 :2;
		uint8_t BFARVALID :1;
		uint8_t UNDEFINSTR :1;
		uint8_t INVSTATE :1;
		uint8_t INVPC :1;
		uint8_t NOCP :1;
		uint8_t ReservedBit3 :4;
		uint8_t UNALIGNED :1;
		uint8_t DIVBYZERO :1;
	}BIT;
} CFSR_T;

typedef struct {
	uint32_t R0, R1, R2, R3;
	uint32_t R12;
	uint32_t LR;		/** Last Context Link Register. */
	uint32_t PC;		/** Last Context PC address that generated HardFault. */
	uint32_t xPSR;		/** Last Context Program Status Register. */
} __attribute__((packed)) REGISTER_STACK_FRAME;

void     debugDumpNVICPriorities (void);

#ifdef __cplusplus
}
#endif

#endif
