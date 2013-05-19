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

typedef enum {
  PROT_CMDTYPE_LED = 0x0001,
  PROT_CMDTYPE_COUNT /**< Number of commands */
}protCmdType_t;

//------------- X macros for create consistent command enum, function prototyp & cmd table -------------//
// This table serves as a central database where each entry is consisted of all information of a command.
// Define your own 'Expansion Macro' and expand the table to form a specific list/table. The following example
// example defines a macro that make each ENTRY of PROTOCOL_COMMAND_TABLE become an assignment with ',' ending.
// Putting that in the enum declaration will form a enumeration of command type.
/*
#define CMDTYPE_EXPAND(command, id, function) \
  command = id,\

typedef enum {
  PROTOCOL_COMMAND_TABLE(CMDTYPE_EXPAND)
  PROT_CMDTYPE_COUNT
}protCmdType_t; */

// command enum, command id, command function
#define PROTOCOL_COMMAND_TABLE(ENTRY) \
    ENTRY(PROT_CMDTYPE_LED, protcmd_led)\



#ifdef __cplusplus
 }
#endif

#endif /* _PROT_CMDTABLE_H_ */

/** @} */
