/**************************************************************************/
/*!
    @file     cmd_wifi.c
    @author   K. Townsend (microBuilder.eu)

    @brief    Commands for the CC3000 wifi module.
    @ingroup  CLI

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, K. Townsend (microbuilder.eu)
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
#include <stdio.h>
#include <string.h>

#include "projectconfig.h"

#ifdef CFG_CC3000

#include "cli/cli.h"
#include "cli/commands.h"
#include "core/eeprom/eeprom.h"
#include "drivers/rf/wifi/cc3000/wifi.h"

/**************************************************************************/
/*!
    Helper function that displays an error message
*/
/**************************************************************************/
void cmd_wifi_helper_error(err_t error)
{
  if(error)
  {
    printf("ERROR: ");
    switch (error)
    {
      case ERROR_CC3000_CONNECT_TIMEOUT:
      printf("Timed out waiting for a connection%s", CFG_PRINTF_NEWLINE);
        break;
      default:
        printf("0x%04X%s", error, CFG_PRINTF_NEWLINE);
        break;
    }
  }
}

/**************************************************************************/
/*!
    'wifi_ssidscan' command handler
*/
/**************************************************************************/
void cmd_wifi_ssidscan(uint8_t argc, char **argv)
{
  err_t error;

  printf("Performing SSID scan (please wait a bit) ...%s%s",
    CFG_PRINTF_NEWLINE, CFG_PRINTF_NEWLINE);

  error = wifi_displaySSIDResults();
  if (error)
  {
    cmd_wifi_helper_error(error);
  }
}

/**************************************************************************/
/*!
    'wifi_connect' command handler
*/
/**************************************************************************/
void cmd_wifi_connect(uint8_t argc, char **argv)
{
  err_t error;

  int32_t sec;
  uint8_t ip[4];
  uint8_t netmask[4];
  uint8_t gateway[4];
  uint8_t dhcp[4];
  uint8_t dns[4];

  /* Retrieve the security mode */
  getNumber (argv[0], &sec);
  if (sec < 0 || sec > 3)
  {
    printf("Security mode not in range (0..3)%s", CFG_PRINTF_NEWLINE);
    return;
  }

  /* Check SSID length */
  if (strlen(argv[1]) > 32)
  {
    printf("SSID must be <= 32 characters%s", CFG_PRINTF_NEWLINE);
    return;
  }

  /* Check key length if we're using a secure network */
  if (sec)
  {
    if (strlen(argv[2]) > 16)
    {
      printf("Key must be <= 16 characters%s", CFG_PRINTF_NEWLINE);
      return;
    }
  }

  /* Try to connect to the AP */
  printf("Connecting to %s (30s timeout) ...%s", argv[1], CFG_PRINTF_NEWLINE);
  if (sec)
  {
    /* Connect to a secure network */
    error = wifi_connectSecure(sec, argv[1], (int32_t)strlen(argv[1]), argv[2], (int32_t)strlen(argv[2]));
  }
  else
  {
    /* Connect to an open network */
    error = wifi_connectSecure(0, argv[1], (int32_t)strlen(argv[1]), "", 1);
  }
  if(error)
  {
    cmd_wifi_helper_error(error);
    return;
  }

  /* Display the connection details */
  error = wifi_getConnectionDetails(ip, netmask, gateway, dhcp, dns);
  if (error)
  {
    cmd_wifi_helper_error(error);
    return;
  }

  printf(CFG_PRINTF_NEWLINE);
  printf("IP Address  : %d.%d.%d.%d %s",
    ip[0], ip[1], ip[2], ip[3], CFG_PRINTF_NEWLINE);
  printf("Netmask     : %d.%d.%d.%d %s",
    netmask[0], netmask[1], netmask[2], netmask[3], CFG_PRINTF_NEWLINE);
  printf("Gateway     : %d.%d.%d.%d %s",
    gateway[0], gateway[1], gateway[2], gateway[3], CFG_PRINTF_NEWLINE);
  printf("DHCP Server : %d.%d.%d.%d %s",
    dhcp[0], dhcp[1], dhcp[2], dhcp[3], CFG_PRINTF_NEWLINE);
  printf("DNS Server  : %d.%d.%d.%d %s",
    dns[0], dns[1], dns[2], dns[3], CFG_PRINTF_NEWLINE);
}

/**************************************************************************/
/*!
    'wifi_smartConfig' command handler
*/
/**************************************************************************/
void cmd_wifi_smartConfig(uint8_t argc, char **argv)
{
  err_t error;
  uint32_t enableAES = 0;

  if (argc == 1)
  {
    getNumber (argv[0], &enableAES);

    /* Make sure enableAES is 1 or 0 */
    if (enableAES < 0 || enableAES > 1)
    {
      printf("enableAES must be 0 or 1%s", CFG_PRINTF_NEWLINE);
      return;
    }
  }

  printf("Waiting for a SmartConfig connection ...%s", CFG_PRINTF_NEWLINE);
  error = wifi_startSmartConfig(enableAES);
  if (error)
  {
    cmd_wifi_helper_error(error);
    return;
  }
}

/**************************************************************************/
/*!
    'wifi_disconnect' command handler
*/
/**************************************************************************/
void cmd_wifi_disconnect(uint8_t argc, char **argv)
{
  wifi_disconnect();
  printf("Disconnected%s", CFG_PRINTF_NEWLINE);
}

/**************************************************************************/
/*!
    'wifi_ping' command handler
*/
/**************************************************************************/
void cmd_wifi_ping(uint8_t argc, char **argv)
{
  err_t  error;
  uint8_t  pingAttempts = 3;
  uint16_t pingTimeout  = 1000;
  uint8_t  ip8[4] = { 0, 0, 0, 0 };
  unsigned int ip[4] = { 0, 0, 0, 0 }; /* keep sscanf happy */

  /* Parse the IP address into four individual bytes */
  sscanf(argv[0], "%u.%u.%u.%u", &ip[0], &ip[1], &ip[2], &ip[3]);

  /* Validate the IP address */
  if(!((ip[0] != 0) && (ip[0] <= 255) && (ip[1] <= 255) && (ip[2] <= 255) && (ip[3] <= 255)))
  {
    printf("Invalid IP address%s", CFG_PRINTF_NEWLINE);
  }

  /* Push values in 8-bit buffer */
  ip8[0] = (uint8_t)(ip[0] & 0xFF);
  ip8[1] = (uint8_t)(ip[1] & 0xFF);
  ip8[2] = (uint8_t)(ip[2] & 0xFF);
  ip8[3] = (uint8_t)(ip[3] & 0xFF);

  /* Send the ping request */
  printf("Pinging %u.%u.%u.%u %u times (%u ms timeout)%s",
    (unsigned int)(ip[0]), (unsigned int)(ip[1]),
    (unsigned int)(ip[2]), (unsigned int)(ip[3]),
    (unsigned int)pingAttempts, (unsigned int)pingTimeout,
    CFG_PRINTF_NEWLINE);
  error = wifi_ping(ip8, pingAttempts, pingTimeout);
  if (error)
  {
    cmd_wifi_helper_error(error);
    return;
  }
}

/**************************************************************************/
/*!
    'wifi_gethostnameip' command handler
*/
/**************************************************************************/
void cmd_wifi_gethostnameip(uint8_t argc, char **argv)
{
  err_t error;
  uint8_t lookupIP[4] = { 0, 0, 0, 0 };

  error = wifi_getHostByName(argv[0], lookupIP);
  if (!error)
  {
    printf("%s => %u.%u.%u.%u %s", argv[0],
      (unsigned int)(lookupIP[0]), (unsigned int)(lookupIP[1]),
      (unsigned int)(lookupIP[2]), (unsigned int)(lookupIP[3]),
      CFG_PRINTF_NEWLINE);
  }
}

/**************************************************************************/
/*!
    'wifi_moduleinfo' command handler
*/
/**************************************************************************/
void cmd_wifi_moduleinfo(uint8_t argc, char **argv)
{
  uint8_t major, minor;
    uint8_t macAddress[6] = { 0, 0, 0, 0, 0, 0 };

  if (!wifi_getFirmwareVersion(&major, &minor))
  {
    printf("Firmware    : v%d.%d%s", major, minor, CFG_PRINTF_NEWLINE);
  }

  if (!wifi_getMacAddress(macAddress))
  {
    printf("MAC Address : %02X:%02X:%02X:%02X:%02X:%02X%s",
      macAddress[0], macAddress[1], macAddress[2],
      macAddress[3], macAddress[4], macAddress[5],
      CFG_PRINTF_NEWLINE);
  }
}

#endif /* CFG_CC3000 */
