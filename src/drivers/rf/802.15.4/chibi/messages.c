/**************************************************************************/
/*!
    @file     messages.c
    @author   K. Townsend (microBuilder.eu)

    @section DESCRIPTION

    Common message handling functions (send, receive, etc.)

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
#include "projectconfig.h"

#ifdef CFG_CHIBI

#include <string.h>

#include "messages.h"
#include "core/delay/delay.h"
#include "chb.h"
#include "chb_drvr.h"

static uint8_t  _msg[CHB_MAX_PAYLOAD];     // Buffer used when sending messages
static uint16_t _msg_msgID;                // Auto-incrementing ID for message envelopes
static uint32_t _msg_alert_uniqueID;       // Auto-incrementing ID for alert messages

/**************************************************************************/
/*!
    @brief Sends the specified message over the air

    @note  Possible error IDs:

           ERROR_CHIBI_NOACK
           ERROR_CHIBI_CHANACCESSFAILURE
           ERROR_CHIBI_PAYLOADOVERFLOW

    @section EXAMPLE

    @code

    // Read some sensor data
    err_t error;
    sensors_event_t event;
    error = mpl115a2GetSensorEvent(&event);

    if (!error)
    {
      // Serialize the data before transmitting
      uint8_t msgbuf[sizeof(event)];
      sensorsSerializeSensorsEvent(msgbuf, &event);

      // Broadcase the sensor event data over the air
      if(msgSend(0xFFFF, MSG_MESSAGETYPE_SENSOREVENT, msgbuf, sizeof(event)))
      {
        printf("Message TX failure%s", CFG_PRINTF_NEWLINE);
      }
    }

    @endcode
*/
/**************************************************************************/
err_t msgSend(uint16_t targetAddr, msg_MessageType_t msgType, uint8_t *payload, uint8_t payloadLength)
{
  uint8_t msgLength = payloadLength + 9;
  uint8_t results;
  uint32_t timestamp;

  /* Make sure payload is within limits */
  /* ToDo: Throw proper error code/msg */
  if (msgLength > CHB_MAX_PAYLOAD)
    return ERROR_CHIBI_PAYLOADOVERFLOW;

  /* Clear message buffer */
  memset(&_msg, 0x00, CHB_MAX_PAYLOAD);

  /* Message Envelope (9 bytes) + Payload
  ==========================================================================
  U16   Message ID        Sequential message ID
  U8    Message Type      Sensor results, Alert, File, etc.
  U32   Timestamp         Current second tick count
  U8    Reserved          Reserved
  U8    Payload Length    Payload length in bytes
  ...   Payload           Message payload
  ------------------------------------------------------------------------*/
  /* Sequential Message ID (U16) */
  *(uint16_t*)&_msg = _msg_msgID++;

  /* Message Type (U8) */
  _msg[2] = msgType;

  /* Timestamp (U32) */
  /* Need to use memcpy here since the M0 can't do unaligned accesses! */
  timestamp = delayGetSecondsActive();
  memcpy(&_msg[3], &timestamp, 4);

  /* Reserved (U8) */
  _msg[7] = 0x00;

  /* Payload Length (U8) */
  _msg[8] = payloadLength;

  /* Message Payload (91 bytes max) */
  memcpy(&_msg[9], payload, payloadLength);
  /*===================================================================== */

  do
  {
    /* Send message via Chibi */
    // chb_pcb_t *pcb = chb_get_pcb();
    GPIOSetBitValue(CFG_LED_PORT, CFG_LED_PIN, CFG_LED_ON);
    results = chb_write(targetAddr, _msg, msgLength);
    GPIOSetBitValue(CFG_LED_PORT, CFG_LED_PIN, CFG_LED_OFF);
  } while(0);

  /* Return an appropriate error code depending on the write status */
  switch(results)
  {
    case CHB_NO_ACK:
      return ERROR_CHIBI_NOACK;
    case CHB_CHANNEL_ACCESS_FAILURE:
      return ERROR_CHIBI_CHANACCESSFAILURE;
    default:
      return ERROR_NONE;
  }
}

/**************************************************************************/
/*!
    @brief Creates a new 'Alert Message' instance (msg_Alert_t)
*/
/**************************************************************************/
void msgCreateAlert(msg_Alert_t *msg)
{
  // Clear memory and assign unique ID
  memset(msg, 0, sizeof(msg_Alert_t));
  msg->uniqueID = _msg_alert_uniqueID++;
}

#endif
