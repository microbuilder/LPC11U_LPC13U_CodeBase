/**************************************************************************/
/*!
    @file     pn532_config.c
*/
/**************************************************************************/
#include "projectconfig.h"

#ifdef CFG_PN532

#include <string.h>

#include "../pn532.h"
#include "../pn532_bus.h"
#include "pn532_config.h"

#include "core/systick/systick.h"

/**************************************************************************/
/*!
    @brief     Sets the MxRtyPassiveActivation byte of the
               RFConfiguration register

    @param     maxRetries    0xFF to wait forever, 0x00..0xFE to timeout
                             after maxRetries
*/
/**************************************************************************/
pn532_error_t pn532_config_SetPassiveActivationRetries(uint8_t maxRetries)
{
  pn532_error_t error;
  byte_t abtCommand[5];
  byte_t abtResponse[16];
  size_t szLen;

  #ifdef PN532_DEBUGMODE
    PN532_DEBUG("Updating Passive Activation Max Retries: 0x%02X (%d)%s", maxRetries, maxRetries, CFG_PRINTF_NEWLINE);
  #endif

  /* Send RFConfiguration command to adjust config item 5 (MaxRetries)    */
  abtCommand[0] = PN532_COMMAND_RFCONFIGURATION;
  abtCommand[1] = 5;                        /* Config item 5 (MaxRetries) */
  abtCommand[2] = 0xFF;                     /* MxRtyATR (default = 0xFF)  */
  abtCommand[3] = 0x01;                     /* MxRtyPSL (default = 0x01)  */
  abtCommand[4] = maxRetries;
  error = pn532Write(abtCommand, sizeof(abtCommand));
  if (error)
    return error;

  /* Wait until we get a valid response or a timeout                      */
  do
  {
    systickDelay(25);
    error = pn532Read(abtResponse, &szLen);
  } while (error == PN532_ERROR_RESPONSEBUFFEREMPTY);
  if (error)
    return error;

  return PN532_ERROR_NONE;
}

#endif  // #ifdef CFG_PN532
