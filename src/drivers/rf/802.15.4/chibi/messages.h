/**************************************************************************/
/*!
    @file     messages.h
    @author   K. Townsend (microBuilder.eu)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2012 K. Townsend
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
#ifndef __MESSAGES_H__
#define __MESSAGES_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"
#include "drivers/sensors/sensors.h"

/* This type is used to indicate the message format/contents, and
   determines how the message will be parsed by the messaging engine.
   Note: Message Type is uint8_t ... keep the IDs within an unsigned
   8-bit limit!                                                           */
typedef enum msg_MessageType_s
{
  MSG_MESSAGETYPE_NONE          = 0,
  MSG_MESSAGETYPE_ALERT         = 1,
  MSG_MESSAGETYPE_PROTOCOLDATA  = 10,
  MSG_MESSAGETYPE_SENSORDETAILS = 20,
  MSG_MESSAGETYPE_SENSOREVENT   = 21,
  MSG_MESSAGETYPE_FILEDETAILS   = 30,
  MSG_MESSAGETYPE_FILEDATA      = 31
} msg_MessageType_t;

/* Message Structs */

/* Alert Frame: 12 bytes */
typedef struct
{
  uint32_t    uniqueID;           // Unique ID to track this alert
  uint16_t    alertType;          // ID indicating the type of alert
  uint8_t     reserved1;
  uint8_t     reserved2;
  uint8_t     payload[4];         // Alert payload (max 4 bytes)
} msg_Alert_t;

/* File Details Frame: 15-78 bytes */
typedef struct
{
  uint16_t    fileId;             // Unique file ID to associate FileData
  uint32_t    size;               // Total file size in bytes
  uint8_t     filetype;           // TBD: Binary, text, etc.
  uint32_t    checksum;           // 32-bit CRC value for entire file
  uint8_t     reserved1;
  uint8_t     reserved2;
  uint8_t     filenameLength;     // Length of the filename (max 64 chars)
  uint8_t     filename[64];       // Filename string
} msg_FileDetails_t;

/* File Data Frame: 9-72 bytes */
typedef struct
{
  uint16_t    fileID;             // Associates data chunk with FileDetails
  uint32_t    chunkID;            // Sequential ID of data chunk
  uint8_t     reserved1;
  uint8_t     payloadLength;      // Length in bytes of .payload
  uint8_t     payload[64];        // Up to 64 bytes of data
} msg_FileData_t;

err_t msgSend ( uint16_t targetAddr, msg_MessageType_t msgType, uint8_t *payload, uint8_t payloadLength );
void    msgCreateAlert ( msg_Alert_t *msg );

#ifdef __cplusplus
}
#endif 

#endif
