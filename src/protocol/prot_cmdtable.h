/**************************************************************************/
/*!
    @file     prot_cmdtable.h
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

#ifndef _PROT_CMDTABLE_H_
#define _PROT_CMDTABLE_H_

#ifdef __cplusplus
 extern "C" {
#endif

/**************************************************************************/
/*!
    This enumeration is used to make sure that each command has a unique
    ID, and is used to create the command lookup table enum further down
*/
/**************************************************************************/
typedef enum {
  PROT_CMDTYPE_LED      = 0x0001, /**< Enables/disables the on board LED */
  PROT_CMDTYPE_SYSINFO  = 0x0002, /**< Gets system properties */
  PROT_CMDTYPE_COUNT              /**< Total number of commands */
} protCmdType_t;

/**************************************************************************/
/*
    The command lookup table is constructed based on this macro containing
    the command ID (as defined in protCmdType_t) and the actual callback
    function to associate with it (in the format defined by protCmdFunc_t)
*/
/**************************************************************************/
#define PROTOCOL_COMMAND_TABLE(ENTRY)            \
    ENTRY(PROT_CMDTYPE_LED, protcmd_led)         \
    ENTRY(PROT_CMDTYPE_SYSINFO, protcmd_sysinfo) \

#ifdef __cplusplus
 }
#endif

#endif /* _PROT_CMDTABLE_H_ */

/** @} */
