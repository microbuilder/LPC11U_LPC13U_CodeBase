/**************************************************************************/
/*!
    @file pn532_bus_i2c.c

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
#include <string.h>

#include "projectconfig.h"

#ifdef CFG_PN532

#include "pn532.h"
#include "pn532_bus.h"

#ifdef PN532_BUS_I2C

#include "core/delay/delay.h"
#include "core/gpio/gpio.h"
#include "core/i2c/i2c.h"

extern volatile uint8_t   I2CMasterBuffer[I2C_BUFSIZE];
extern volatile uint8_t   I2CSlaveBuffer[I2C_BUFSIZE];
extern volatile uint32_t  I2CReadLength, I2CWriteLength;

// Don't use a timeout for now since it complicates things (set to 0)
#define PN532_I2C_TIMEOUT (0)

/* ======================================================================
   PRIVATE FUNCTIONS
   ====================================================================== */

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C

    @note   Possible error messages are:

            - PN532_ERROR_I2C_NACK
*/
/**************************************************************************/
pn532_error_t pn532_bus_i2c_WriteData (const byte_t * pbtData, const size_t szData)
{
  uint32_t i;
  uint32_t i2cState;

  // Send the specified bytes
  I2CWriteLength = szData+1;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = PN532_I2C_ADDRESS;         // I2C device address
  for ( i = 0; i < szData; i++ )
  {
    I2CMasterBuffer[i+1] = pbtData[i];
  }
  i2cState = i2cEngine();

  // Check if we got an ACK
  if ((i2cState == I2CSTATE_NACK) || (i2cState == I2CSTATE_SLA_NACK))
  {
    // I2C slave didn't acknowledge the master transfer
    // The PN532 probably isn't connected properly or the
    // bus select pins are in the wrong state
    return PN532_ERROR_I2C_NACK;
  }
  if (i2cState == I2CSTATE_TIMEOUT)
  {
        // The most likely cause of this is missing pullups on I2C
        return PN532_ERROR_I2C_TIMEOUT;
  }

  return PN532_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief    Checks the 'IRQ' pin to know if the PN532 is ready to send
              a response or not

    @note     The IRQ bit may stay high intentionally, and this isn't
              always an error condition.  When PN532_COMMAND_INLISTPASSIVETARGET
              is sent, for example, the PN532 will wait until a card
              enters the magnetic field, and IRQ will remain high since
              there is no response ready yet.  The IRQ pin will go low
              as soon as a card enters the magnetic field and the data
              has been retrieved from it.

    @returns  1 if a response is ready, 0 if the PN532 is still busy or a
              timeout occurred
*/
/**************************************************************************/
bool pn532_bus_i2c_WaitForReady(uint32_t timeout)
{
  uint8_t busy = 1;
  uint8_t busyTimeout = 0;

  if (timeout)
  {
    /* Wait up to the specified number of ms for the IRQ */
    while (busy)
    {
      busy = GPIOGetPinValue(CFG_PN532_I2C_IRQPORT, CFG_PN532_I2C_IRQPIN);
      delay(1);
      busyTimeout++;
      if (busyTimeout == PN532_I2C_READYTIMEOUT)
      {
         return false;
       }
    }
  }
  else
  {
    /* Wait forever for the IRQ */
    while (busy)
    {
      busy = GPIOGetPinValue(CFG_PN532_I2C_IRQPORT, CFG_PN532_I2C_IRQPIN);
      delay(1);
    }
  }

  return true;
}

/**************************************************************************/
/*!
    @brief  Builds a standard PN532 frame using the supplied data

    @param  pbtFrame  Pointer to the field that will hold the frame data
    @param  pszFrame  Pointer to the field that will hold the frame length
    @param  pbtData   Pointer to the data to insert in a frame
    @param  swData    Length of the data to insert in bytes

    @note   Possible error messages are:

            - PN532_ERROR_EXTENDEDFRAME
*/
/**************************************************************************/
pn532_error_t pn532_bus_i2c_BuildFrame(byte_t * pbtFrame, size_t * pszFrame, const byte_t * pbtData, const size_t szData)
{
  size_t szPos;
  byte_t btDCS;

  if (szData > PN532_NORMAL_FRAME__DATA_MAX_LEN)
  {
    // Extended frames currently unsupported
    return PN532_ERROR_EXTENDEDFRAME;
  }

  // LEN - Packet length = data length (len) + checksum (1) + end of stream marker (1)
  pbtFrame[3] = szData + 1;
  // LCS - Packet length checksum
  pbtFrame[4] = 256 - (szData + 1);
  // TFI
  pbtFrame[5] = 0xD4;
  // DATA - Copy the PN53X command into the packet buffer
  memcpy (pbtFrame + 6, pbtData, szData);

  // DCS - Calculate data payload checksum
  btDCS = (256 - 0xD4);
  for (szPos = 0; szPos < szData; szPos++)
  {
    btDCS -= pbtData[szPos];
  }
  pbtFrame[6 + szData] = btDCS;

  // 0x00 - End of stream marker
  pbtFrame[szData + 7] = 0x00;

  (*pszFrame) = szData + PN532_NORMAL_FRAME__OVERHEAD;

  return PN532_ERROR_NONE;
}

/* ======================================================================
   PUBLIC FUNCTIONS
   ====================================================================== */

/**************************************************************************/
/*!
    @brief  Initialises I2C and configures the PN532 HW
*/
/**************************************************************************/
err_t pn532_bus_HWInit(void)
{
  #ifdef PN532_DEBUGMODE
  PN532_DEBUG("Initialising I2C%s", CFG_PRINTF_NEWLINE);
  #endif
  i2cInit(I2CMASTER);

  // Set reset pin as output and reset device
  LPC_GPIO->DIR[CFG_PN532_RSTPD_PORT] |= (1 << CFG_PN532_RSTPD_PIN);
  #ifdef PN532_DEBUGMODE
  PN532_DEBUG("Resetting the PN532%s", CFG_PRINTF_NEWLINE);
  #endif
  LPC_GPIO->CLR[CFG_PN532_RSTPD_PORT] = (1 << CFG_PN532_RSTPD_PIN);
  delay(400);
  LPC_GPIO->SET[CFG_PN532_RSTPD_PORT] = (1 << CFG_PN532_RSTPD_PIN);

  // Wait for the PN532 to finish booting
  delay(100);

  // Ping the I2C device first to see if it exists!
  if (i2cCheckAddress(PN532_I2C_ADDRESS) == false)
  {
    #ifdef PN532_DEBUGMODE
    PN532_DEBUG("Can't find PN532 on the I2C bus%s", CFG_PRINTF_NEWLINE);
    #endif
    return ERROR_I2C_DEVICENOTFOUND;
  }

  // Set IRQ pin to input
  LPC_GPIO->DIR[CFG_PN532_I2C_IRQPORT] &= ~(1 << CFG_PN532_I2C_IRQPIN);

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Sends the specified command to the PN532, automatically
            creating an appropriate frame for it

    @param  pdbData   Pointer to the byte data to send
    @param  szData    Length in bytes of the data to send

    @note   Possible error messages are:

            - PN532_ERROR_EXTENDEDFRAME       // Extended frames not supported
            - PN532_ERROR_BUSY                // Already busy with a command
            - PN532_ERROR_I2C_NACK            // No ACK on I2C
            - PN532_ERROR_READYSTATUSTIMEOUT  // Timeout waiting for ready bit
            - PN532_ERROR_INVALIDACK          // No ACK frame received
*/
/**************************************************************************/
pn532_error_t pn532_bus_SendCommand(const byte_t * pbtData, const size_t szData)
{
  pn532_error_t error = PN532_ERROR_NONE;
  byte_t abtFrame[PN532_BUFFER_LEN] = { 0x00, 0x00, 0xff };
  size_t szFrame = 0;
  pn532_pcb_t *pn532 = pn532GetPCB();
  uint32_t i;

  // Check if we're busy
  if (pn532->state == PN532_STATE_BUSY)
  {
    return PN532_ERROR_BUSY;
  }

  // Flag the stack as busy
  pn532->state = PN532_STATE_BUSY;

  // --------------------------------------------------------------------
  // Send the command frame
  // --------------------------------------------------------------------
  // Build the frame
  pn532_bus_i2c_BuildFrame (abtFrame, &szFrame, pbtData, szData);

  // Keep track of the last command that was sent
  pn532->lastCommand = pbtData[0];

  // Output the frame data for debugging if requested
  #ifdef PN532_DEBUGMODE
  PN532_DEBUG("Sending  (%02d): ", szFrame);
  pn532PrintHex(abtFrame, szFrame);
  #endif

  // Send data to the PN532
  error = pn532_bus_i2c_WriteData(abtFrame, szFrame);

  if (error == PN532_ERROR_I2C_NACK)
  {
    // Most likely error is PN532_ERROR_I2C_NACK
    // meaning no I2C ACK received from the PN532
    #ifdef PN532_DEBUGMODE
    PN532_DEBUG ("No ACK received on I2C bus%s", CFG_PRINTF_NEWLINE);
    #endif
    pn532->state = PN532_STATE_READY;
    return error;
  }

  // --------------------------------------------------------------------
  // Wait for the IRQ/Ready flag
  // --------------------------------------------------------------------
  if (!(pn532_bus_i2c_WaitForReady(PN532_I2C_TIMEOUT)))
  {
    pn532->state = PN532_STATE_READY;
    #ifdef PN532_DEBUGMODE
    PN532_DEBUG ("Timed out waiting for IRQ/Ready%s", CFG_PRINTF_NEWLINE);
    #endif
    return PN532_ERROR_READYSTATUSTIMEOUT;
  }

  // --------------------------------------------------------------------
  // Read the ACK frame
  // --------------------------------------------------------------------
  I2CWriteLength = 0;
  I2CReadLength = 7;  // ACK + Ready bit = 7
  I2CMasterBuffer[0] = PN532_I2C_ADDRESS | PN532_I2C_READBIT;
  i2cEngine();

  // Make sure the received ACK matches the prototype
  do
  {
    const byte_t abtAck[6] = { 0x00, 0x00, 0xff, 0x00, 0xff, 0x00 };
    byte_t abtRxBuf[6];
    // memcpy(abtRxBuf, I2CSlaveBuffer+1, 6);
    for ( i = 0; i < 6; i++ )
    {
      abtRxBuf[i] = I2CSlaveBuffer[i+1];
    }
    if (0 != (memcmp (abtRxBuf, abtAck, 6)))
    {
      #ifdef PN532_DEBUGMODE
      PN532_DEBUG ("Invalid ACK: ");
      pn532PrintHex(abtRxBuf, 6);
      PN532_DEBUG("%s", CFG_PRINTF_NEWLINE);
      #endif
      pn532->state = PN532_STATE_READY;
      return PN532_ERROR_INVALIDACK;
    }

    // --------------------------------------------------------------------
    // Wait for the post-ACK IRQ/Ready flag
    // --------------------------------------------------------------------
    if (!(pn532_bus_i2c_WaitForReady(PN532_I2C_TIMEOUT)))
    {
      pn532->state = PN532_STATE_READY;
      #ifdef PN532_DEBUGMODE
      PN532_DEBUG ("Timed out waiting for IRQ/Ready%s", CFG_PRINTF_NEWLINE);
      #endif
      return PN532_ERROR_READYSTATUSTIMEOUT;
    }
  } while(0);

  pn532->state = PN532_STATE_READY;
  return PN532_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads a response from the PN532

    @note   Possible error message are:

            - PN532_ERROR_BUSY
            - PN532_ERROR_RESPONSEBUFFEREMPTY
            - PN532_ERROR_PREAMBLEMISMATCH
            - PN532_ERROR_APPLEVELERROR
            - PN532_ERROR_EXTENDEDFRAME
            - PN532_ERROR_LENCHECKSUMMISMATCH
*/
/**************************************************************************/
pn532_error_t pn532_bus_ReadResponse(byte_t * pbtResponse, size_t * pszRxLen)
{
  uint8_t i;
  pn532_pcb_t *pn532 = pn532GetPCB();

  // Check if we're busy
  if (pn532->state == PN532_STATE_BUSY)
  {
    return PN532_ERROR_BUSY;
  }

  // Flag the stack as busy
  pn532->state = PN532_STATE_BUSY;

  // Reset the app error flag
  pn532->appError = PN532_APPERROR_NONE;

  for ( i = 0; i < I2C_BUFSIZE; i++ )
  {
    I2CMasterBuffer[i] = 0x00;
  }
  I2CWriteLength = 0;
  I2CReadLength = I2C_BUFSIZE;
  I2CMasterBuffer[0] = PN532_I2C_ADDRESS | PN532_I2C_READBIT;
  i2cEngine();

  // Use the full I2C buffer size for now (until we're sure we have a good frame)
  *pszRxLen = I2C_BUFSIZE - 1;

  // Display the raw response data for debugging if requested
  #ifdef PN532_DEBUGMODE
  PN532_DEBUG("Received (%02d): ", I2C_BUFSIZE-1);
  pn532PrintHex(I2CSlaveBuffer+1, I2C_BUFSIZE-1);
  #endif

  // Check the frame type
  if ((0x01 == I2CSlaveBuffer[4]) && (0xff == I2CSlaveBuffer[5]))
  {
    // Error frame
    #ifdef PN532_DEBUGMODE
    PN532_DEBUG("Application level error (0x%02x)%s", I2CSlaveBuffer[6], CFG_PRINTF_NEWLINE);
    #endif
    // Set application error message ID
    pn532->appError = I2CSlaveBuffer[6];
    pn532->state = PN532_STATE_READY;
    return PN532_ERROR_APPLEVELERROR;
  }
  else if ((0xff == I2CSlaveBuffer[4]) && (0xff == I2CSlaveBuffer[5]))
  {
    // Extended frame
    #ifdef PN532_DEBUGMODE
    PN532_DEBUG("Extended frames currently unsupported%s", CFG_PRINTF_NEWLINE);
    #endif
    pn532->state = PN532_STATE_READY;
    return PN532_ERROR_EXTENDEDFRAME;
  }
  else
  {
    // Normal frame
    if (256 != ((I2CSlaveBuffer[4]) + (I2CSlaveBuffer[5])))
    {
      // TODO: Retry
      #ifdef PN532_DEBUGMODE
      PN532_DEBUG("Length checksum mismatch%s", CFG_PRINTF_NEWLINE);
      #endif
      pn532->state = PN532_STATE_READY;
      return PN532_ERROR_LENCHECKSUMMISMATCH;
    }
  }

  // Figure out how large the response really is
  // Response Frame Len = pbtResponse[4] + 7 (00 00 FF LEN LCS TFI [DATA] DCS)
  *pszRxLen = (I2CSlaveBuffer[4]) + 7;

  // TODO: Find a solution for this horribly ugly Mifare Classic block write hack!
  // Some responses to command 0x40 report the incorrect len, and don't take into
  // account the 16 byte payload when working with Mifare Classic sectors.
  // The response frame indicates len 10 (0x0A) in I2CSlaveBuffer[4] but it should be
  // 10+16 = 26 (0x1A)
  if ((*pszRxLen == 10) && (I2CSlaveBuffer[7] == 0x41) && (I2CSlaveBuffer[26] != 0x00))
  {
    // For some reason, the PN532 reports len 10 for responses to
    // command 0x40 which includes the command data but does not
    // take into account the response/payload data in the len byte
    *pszRxLen+=16;
  }

  // Fill the response buffer
  // memcpy(pbtResponse, I2CSlaveBuffer+1, *pszRxLen);
  for ( i = 0; i < *pszRxLen; i++ )
  {
    pbtResponse[i] = I2CSlaveBuffer[i+1];
  }

  pn532->state = PN532_STATE_READY;
  return PN532_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief      Sends the wakeup sequence to the PN532.

    @note   Possible error message are:

            - PN532_ERROR_BUSY
            - PN532_ERROR_I2C_NACK            // No I2C ACK
            - PN532_ERROR_READYSTATUSTIMEOUT  // Timed out waiting for ready bit
*/
/**************************************************************************/
pn532_error_t pn532_bus_Wakeup(void)
{
  pn532_error_t error = PN532_ERROR_NONE;
  byte_t abtWakeUp[] = { 0x55,0x55,0x00,0x00,0x00,0x00,0x00,0xff,0x03,0xfd,0xd4,0x14,0x01,0x17,0x00,0x00,0xff,0x03,0xfd,0xd4,0x14,0x01,0x17,0x00 };

  pn532_pcb_t *pn532 = pn532GetPCB();

  #ifdef PN532_DEBUGMODE
  PN532_DEBUG("Sending Wakeup Sequence%s", CFG_PRINTF_NEWLINE);
  #endif
  error = pn532_bus_i2c_WriteData(abtWakeUp,sizeof(abtWakeUp));
  if (error)
  {
    #ifdef PN532_DEBUGMODE
      PN532_DEBUG("Wakeup Failed (Error: %d)%s", error, CFG_PRINTF_NEWLINE);
    #endif
    return error;
  }

  delay(100);

  // Wait for the IRQ/Ready flag to indicate a response is ready
  if (!(pn532_bus_i2c_WaitForReady(PN532_I2C_TIMEOUT)))
  {
    #ifdef PN532_DEBUGMODE
    PN532_DEBUG ("Timed out waiting for IRQ/Ready%s", CFG_PRINTF_NEWLINE);
    #endif
    error = PN532_ERROR_READYSTATUSTIMEOUT;
  }

  // Read and discard the ACK frame
  I2CWriteLength = 0;
  I2CReadLength = 7;  // ACK + Ready bit = 7
  I2CMasterBuffer[0] = PN532_I2C_ADDRESS | PN532_I2C_READBIT;
  i2cEngine();
  delay(1);

  // Wait for the IRQ/Ready flag to indicate a response is ready
  if (!(pn532_bus_i2c_WaitForReady(PN532_I2C_TIMEOUT)))
  {
    error = PN532_ERROR_READYSTATUSTIMEOUT;
  }
  #ifdef PN532_DEBUGMODE
  PN532_DEBUG("Wakeup Complete%s", CFG_PRINTF_NEWLINE);
  #endif

  pn532->state = PN532_STATE_READY;
  return error;
}

#endif  // #ifdef PN532_BUS_I2C
#endif  // #ifdef CFG_PN532
