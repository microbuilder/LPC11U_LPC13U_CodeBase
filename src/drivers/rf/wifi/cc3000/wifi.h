/**************************************************************************/
/*!
    @file     wifi.h
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
#ifndef _WIFI_APPLICATION_BASIC_H_
#define _WIFI_APPLICATION_BASIC_H_

#include "projectconfig.h"

#include <stdint.h>

#pragma pack(1)
/**! Stores the results of SSID scans  */
typedef struct Result_Struct
{
  uint32_t  num_networks;
  uint32_t  scan_status;
  uint8_t   rssiByte;
  uint8_t   Sec_ssidLen;
  uint16_t  time;
  uint8_t   ssid_name[32];
  uint8_t   bssid[6];
} ResultStruct_t;

err_t  wifi_init(unsigned short cRequestPatch);
err_t  wifi_getFirmwareVersion(uint8_t *major, uint8_t *minor);
err_t  wifi_getMacAddress(uint8_t macAddress[6]);
err_t  wifi_setMacAddress(uint8_t macAddress[6]);
err_t  wifi_ssidScan(uint32_t time);
err_t  wifi_displaySSIDResults(void);
err_t  wifi_connectSecure(int32_t sec, int8_t *ssid, int32_t ssidlen, int8_t *key, int32_t keylen);
err_t  wifi_startSmartConfig(bool enableAES);
err_t  wifi_disconnect(void);
err_t  wifi_getConnectionDetails(uint8_t ipAddress[4], uint8_t netmask[4], uint8_t gateway[4], uint8_t dhcpServer[4], uint8_t dnsServer[4]);
err_t  wifi_ping(uint8_t ip[4], uint8_t attempts, uint16_t timeout);
err_t  wifi_getHostByName(uint8_t *hostName, uint8_t ip[4]);
bool     wifi_isConnected(void);
bool     wifi_isDHCPComplete(void);

#endif
