/**************************************************************************/
/*!
    @file     pn532_gpio.c
*/
/**************************************************************************/
#include "projectconfig.h"

#ifdef CFG_PN532

#include <string.h>

#include "../pn532.h"
#include "../pn532_bus.h"
#include "pn532_gpio.h"

#include "core/systick/systick.h"

/**************************************************************************/
/*!
    @brief   Writes an 8-bit value that sets the state of the PN532's
                 GPIO pins

    @warning This function is provided exclusively for board testing and
             is dangerous since it will throw an error if any pin other
             than the ones marked "Can be used as GPIO" are modified!  All
             pins that can not be used as GPIO should ALWAYS be left high
             (value = 1) or the system will become unstable and a HW reset
             will be required to recover the PN532.

             pinState[0]  = P30     Can be used as GPIO
             pinState[1]  = P31     Can be used as GPIO
             pinState[2]  = P32     *** RESERVED (Must be 1!) ***
             pinState[3]  = P33     Can be used as GPIO
             pinState[4]  = P34     *** RESERVED (Must be 1!) ***
             pinState[5]  = P35     Can be used as GPIO
*/
/**************************************************************************/
pn532_error_t pn532_gpio_WriteGPIO (uint8_t pinState)
{
  pn532_error_t error;
  byte_t _pn532_gpio_abtResponse[PN532_GPIO_RESPONSELENGTH];
  size_t szLen;

  /* Make sure pinstate does not try to toggle P32 or P34                 */
  pinState |= (1 << PN532_GPIO_P32) | (1 << PN532_GPIO_P34);

  /* Send GPIO Write command (only P3 pins used since P7 is used by I2C)  */
  byte_t abtCommand[] = { PN532_COMMAND_WRITEGPIO,             /* Command */
                          PN532_GPIO_VALIDATIONBIT | pinState, /* P3 Pins */
                          0x00 };                              /* P7 Pins */
  error = pn532Write(abtCommand, sizeof(abtCommand));
  if (error)
    return error;

  /* Wait until we get a valid response or a timeout                      */
  do
  {
    systickDelay(25);
    error = pn532Read(_pn532_gpio_abtResponse, &szLen);
  } while (error == PN532_ERROR_RESPONSEBUFFEREMPTY);
  if (error)
    return error;

  #ifdef PN532_DEBUGMODE
    PN532_DEBUG("Writing P3 GPIO: 0x%X%s", abtCommand[1], CFG_PRINTF_NEWLINE);
  #endif

  return PN532_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief   Reads the state of the PN532's GPIO pins into pinState

             pinState[0]  = P30
             pinState[1]  = P31
             pinState[2]  = P32
             pinState[3]  = P33
             pinState[4]  = P34
             pinState[5]  = P35
*/
/**************************************************************************/
pn532_error_t pn532_gpio_ReadGPIO (uint8_t * pinState)
{
  pn532_error_t error;
  byte_t _pn532_gpio_abtResponse[PN532_GPIO_RESPONSELENGTH];
  size_t szLen;

  /* Send the command (0x0C) */
  byte_t abtCommand[] = { PN532_COMMAND_READGPIO };
  error = pn532Write(abtCommand, sizeof(abtCommand));
  if (error)
    return error;

  /* Wait for a response */
  do
  {
    systickDelay(25);
    error = pn532Read(_pn532_gpio_abtResponse, &szLen);
  }
  while (error == PN532_ERROR_RESPONSEBUFFEREMPTY);
  if (error)
    return error;

  /* READGPIO response should be in the following format:

    byte            Description
    -------------   ------------------------------------------
    b0..6           Frame header and preamble
    b7              P3 GPIO Pins
    b8              P7 GPIO Pins (not used ... taken by I2C)
    b9              Interface Mode Pins (not used ... bus select pins)
    b10             checksum */

  #ifdef PN532_DEBUGMODE
    PN532_DEBUG("P3 GPIO: 0x%02X%s", _pn532_gpio_abtResponse[7], CFG_PRINTF_NEWLINE);
    PN532_DEBUG("P7 GPIO: 0x%02X%s", _pn532_gpio_abtResponse[8], CFG_PRINTF_NEWLINE);
    PN532_DEBUG("IO GPIO: 0x%02X%s", _pn532_gpio_abtResponse[9], CFG_PRINTF_NEWLINE);
    /* Note: You can use the IO GPIO value to detect the serial bus being used */
    switch(_pn532_gpio_abtResponse[9])
    {
      case 0x00:    // Using UART
        PN532_DEBUG("Using UART (IO = 0x00)%s", CFG_PRINTF_NEWLINE);
        break;
      case 0x01:    // Using I2C
        PN532_DEBUG("Using I2C (IO = 0x01)%s", CFG_PRINTF_NEWLINE);
        break;
      case 0x02:    // Using I2C
        PN532_DEBUG("Using I2C (IO = 0x02)%s", CFG_PRINTF_NEWLINE);
        break;
      default:
        break;
    }
  #endif

  /* Assign the pin state to the supplied variable */
  *pinState = _pn532_gpio_abtResponse[7];

  return PN532_ERROR_NONE;
}
#endif  // #ifdef CFG_PN532
