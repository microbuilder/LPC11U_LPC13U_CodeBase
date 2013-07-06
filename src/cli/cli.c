/*******************************************************************
    Copyright (C) 2009 FreakLabs
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
    3. Neither the name of the the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS'' AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
    OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
    OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
    SUCH DAMAGE.

    Originally written by Christopher Wang aka Akiba.
    Please post support questions to the FreakLabs forum.
*******************************************************************/

/**************************************************************************/
/*!
    @file     cli.c
    @author   Christopher Wang (Freaklabs)
              Modified by: K. Townsend (microBuilder.eu)

    @ingroup  CLI

    Original code taken from the FreakUSB Open Source USB Device Stack
    http://freaklabs.org/index.php/FreakUSB-Open-Source-USB-Device-Stack.html

    If it works well, you can thank Akiba at Freaklabs.  If it fails
    miserably, you can blame me (since parts of it it were rather
    ungraciously modified). :-)

*/
/**************************************************************************/

#include "projectconfig.h"

#ifdef CFG_INTERFACE

#include <stdio.h>
#include <string.h>

#include "cli.h"
#include "cli_tbl.h"

#ifdef CFG_PRINTF_UART
#include "core/uart/uart.h"
#endif

#ifdef CFG_PRINTF_USBCDC
  #include "core/usb/usbd.h"
  #include "core/usb/usb_cdc.h"
#endif

#if CFG_INTERFACE_ENABLEIRQ == 1
  #include "core/gpio/gpio.h"
#endif

#define KEY_CODE_ESC        (27)    /* Escape key code */
#define KEY_CODE_ENTER      (13)    /* Enter key code  */

static uint8_t cli_buffer[CFG_INTERFACE_MAXMSGSIZE];
static uint8_t *cli_buffer_ptr;

/**************************************************************************/
/*!
    @brief  Polls the relevant incoming message queue to see if anything
            is waiting to be processed.
*/
/**************************************************************************/
void cliPoll()
{
  #if defined CFG_PRINTF_UART
  while (uartRxBufferDataPending())
  {
    uint8_t c = uartRxBufferRead();
    cliRx(c);
  }
  #endif

  #if defined(CFG_USB) && defined(CFG_PRINTF_USBCDC)
  if (usb_isConfigured())
  {
    uint8_t c;
    while(usb_cdc_getc(&c))
    {
      cliRx(c);
    }
  }
  #endif
}

/**************************************************************************/
/*!
    @brief  Reads the CLI input until the 'Enter' key is detected, or
            until we reach the end of the input buffer (use with care!)
*/
/**************************************************************************/
void cliReadLine(uint8_t *str, uint16_t *strLen)
{
  uint8_t ch;
  uint16_t idx = 0;

  /* ToDo: Update this to handle UART, etc., with proper #ifdef blocks! */

  while (1)
  {
    if (usb_isConfigured())
    {
      while (!usb_cdc_getc(&ch));

      if ((ch >= 32) && (ch <= 126) && (idx < *strLen))
      {
        str[idx++] = ch;
        cliRx(ch);
      }
      if (((ch == 8) || (ch == 127)) && idx)
      {
        str[idx--] = 0;
        cliRx(ch);
      }
      if (ch == KEY_CODE_ENTER)
      {
        *strLen = idx;
        break;
      }
    }
  }
}

/**************************************************************************/
/*!
    @brief  Handles a single incoming character.  If a new line is
            detected, the entire command will be passed to the command
            parser.  If a text character is detected, it will be added to
            the message buffer until a new line is detected (up to the
            maximum queue size, CFG_INTERFACE_MAXMSGSIZE).

    @param[in]  c
                The character to parse.
*/
/**************************************************************************/
void cliRx(uint8_t c)
{
  // read out the data in the buffer and echo it back to the host.
  switch (c)
  {
    case '\r':
      #if CFG_INTERFACE_DROPCR == 1
      break;
      #endif
    case '\n':
        // terminate the cli_buffer and reset the cli_buffer ptr. then send
        // it to the handler for processing.
        *cli_buffer_ptr = '\0';
        #if CFG_INTERFACE_SILENTMODE == 0
        printf("%s", CFG_PRINTF_NEWLINE);
        #endif
        cliParse((char *)cli_buffer);
        cli_buffer_ptr = cli_buffer;
        break;

    case '\b':
        #if CFG_INTERFACE_SILENTMODE == 0
        printf("%c",c);
        #endif
        if (cli_buffer_ptr == cli_buffer)
        {
            // Send bell alert and space (to maintain position)
            printf("\a ");
        }
        else if (cli_buffer_ptr > cli_buffer)
        {
            cli_buffer_ptr--;
        }
        break;

    default:
        #if CFG_INTERFACE_SILENTMODE == 0
        printf("%c",c);
        #endif
        *cli_buffer_ptr++ = c;
        break;
  }
}

/**************************************************************************/
/*!
    @brief  Displays the command prompt.  The text that appears is defined
            in projectconfig.h.
*/
/**************************************************************************/
static void cliMenu()
{
  #if CFG_INTERFACE_SILENTMODE == 0
  printf(CFG_PRINTF_NEWLINE);
  printf(CFG_INTERFACE_PROMPT);
  #endif
  #if CFG_INTERFACE_CONFIRMREADY == 1
  printf("%s%s", CFG_INTERFACE_CONFIRMREADY_TEXT, CFG_PRINTF_NEWLINE);
  #endif
}

/**************************************************************************/
/*!
    @brief  Parse the command line. This function tokenizes the command
            input, then searches for the command table entry associated
            with the commmand. Once found, it will jump to the
            corresponding function.

    @param[in]  cmd
                The entire command string to be parsed
*/
/**************************************************************************/
void cliParse(char *cmd)
{
  size_t argc, i = 0;
  char *argv[30];

  argv[i] = strtok(cmd, " ");
  do
  {
      argv[++i] = strtok(NULL, " ");
  } while ((i < 30) && (argv[i] != NULL));

  argc = i;
  for (i=0; i < CMD_COUNT; i++)
  {
      if (!strcmp(argv[0], cli_tbl[i].command))
      {
        if ((argc == 2) && !strcmp (argv [1], "?"))
        {
          // Display parameter help menu on 'command ?'
          printf ("%s%s%s", cli_tbl[i].description, CFG_PRINTF_NEWLINE, CFG_PRINTF_NEWLINE);
          printf ("%s%s", cli_tbl[i].parameters, CFG_PRINTF_NEWLINE);
        }
        else if ((argc - 1) < cli_tbl[i].minArgs)
        {
          // Too few arguments supplied
          #if CFG_INTERFACE_SHORTERRORS == 1
          printf ("%s%s", CFG_INTERFACE_SHORTERRORS_TOOFEWARGS, CFG_PRINTF_NEWLINE);
          #else
          printf ("%s (%s %d)%s", STRING(LOCALISATION_TEXT_Too_few_arguments), STRING(LOCALISATION_TEXT_Expected), cli_tbl[i].minArgs, CFG_PRINTF_NEWLINE);
          printf ("%s'%s ?' %s%s%s", CFG_PRINTF_NEWLINE, cli_tbl[i].command, STRING(LOCALISATION_TEXT_for_more_information), CFG_PRINTF_NEWLINE, CFG_PRINTF_NEWLINE);
          #endif
        }
        else if ((argc - 1) > cli_tbl[i].maxArgs)
        {
          // Too many arguments supplied
          #if CFG_INTERFACE_SHORTERRORS == 1
          printf ("%s%s", CFG_INTERFACE_SHORTERRORS_TOOMANYARGS, CFG_PRINTF_NEWLINE);
          #else
          printf ("%s (%s %d)%s", STRING(LOCALISATION_TEXT_Too_many_arguments), STRING(LOCALISATION_TEXT_Maximum), cli_tbl[i].maxArgs, CFG_PRINTF_NEWLINE);
          printf ("%s'%s ?' %s%s%s", CFG_PRINTF_NEWLINE, cli_tbl[i].command, STRING(LOCALISATION_TEXT_for_more_information), CFG_PRINTF_NEWLINE, CFG_PRINTF_NEWLINE);
          #endif
        }
        else
        {
          #if CFG_INTERFACE_ENABLEIRQ != 0
          // Set the IRQ pin high at start of a command
          gpioSetValue(CFG_INTERFACE_IRQPORT, CFG_INTERFACE_IRQPIN, 1);
          #endif
          // Dispatch command to the appropriate function
          cli_tbl[i].func(argc - 1, &argv [1]);
          #if CFG_INTERFACE_ENABLEIRQ  != 0
          // Set the IRQ pin low to signal the end of a command
          gpioSetValue(CFG_INTERFACE_IRQPORT, CFG_INTERFACE_IRQPIN, 0);
          #endif
        }

        // Refresh the command prompt
        cliMenu();
        return;
      }
  }
  // Command not recognized
  #if CFG_INTERFACE_SHORTERRORS == 1
  printf ("%s%s", CFG_INTERFACE_SHORTERRORS_UNKNOWNCOMMAND, CFG_PRINTF_NEWLINE);
  #else
  printf("%s: '%s'%s%s", STRING(LOCALISATION_TEXT_Command_Not_Recognized), cmd, CFG_PRINTF_NEWLINE, CFG_PRINTF_NEWLINE);
  #if CFG_INTERFACE_SILENTMODE == 0
  printf("%s%s", STRING(LOCALISATION_TEXT_Type_QUESTION_for_a_list_of), CFG_PRINTF_NEWLINE);
  #endif
  #endif

  cliMenu();
}

/**************************************************************************/
/*!
    @brief Initialises the command line using the appropriate interface
*/
/**************************************************************************/
void cliInit()
{
  #if defined CFG_INTERFACE && defined CFG_PRINTF_UART
    // Check if UART is already initialised
    uart_pcb_t *pcb = uartGetPCB();
    if (!pcb->initialised)
    {
      uartInit(CFG_UART_BAUDRATE);
    }
  #endif

  #if CFG_INTERFACE_ENABLEIRQ != 0
    // Set IRQ pin as output
    LPC_GPIO->DIR[CFG_INTERFACE_IRQPORT] |= (1 << CFG_INTERFACE_IRQPIN);
    LPC_GPIO->SET[CFG_INTERFACE_IRQPORT] = (1 << CFG_INTERFACE_IRQPIN);
  #endif

  // init the cli_buffer ptr
  cli_buffer_ptr = cli_buffer;

  // Show the menu
  cliMenu();

  // Set the IRQ pin low by default
  #if CFG_INTERFACE_ENABLEIRQ  != 0
    LPC_GPIO->CLR[CFG_INTERFACE_IRQPORT] = (1 << CFG_INTERFACE_IRQPIN);
  #endif
}

/**************************************************************************/
/*!
    'help' command handler
*/
/**************************************************************************/
void cmd_help(uint8_t argc, char **argv)
{
  size_t i;

  printf("%s      %s%s", STRING(LOCALISATION_TEXT_Command), STRING(LOCALISATION_TEXT_Description), CFG_PRINTF_NEWLINE);
  printf("-------      -----------%s", CFG_PRINTF_NEWLINE);

  // Display full command list
  for (i=0; i < CMD_COUNT; i++)
  {
    if (!cli_tbl[i].hidden)
    {
      printf ("%-10s   %s%s", cli_tbl[i].command, cli_tbl[i].description, CFG_PRINTF_NEWLINE);
    }
  }

  printf("%s%s", STRING(LOCALISATION_TEXT_Command_parameters_can_be_seen), CFG_PRINTF_NEWLINE);
}

#endif
