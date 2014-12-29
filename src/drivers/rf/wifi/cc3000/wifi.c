/**************************************************************************/
/*!
    @file     wifi.c
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
#include "projectconfig.h"

#ifdef CFG_CC3000

#include "wifi.h"
#include "spi.h"
#include "hostdriver/wlan.h"
#include "hostdriver/socket.h"
#include "hostdriver/hci.h"
#include "hostdriver/nvmem.h"
#include "hostdriver/security.h"
#include "hostdriver/netapp.h"
#include "hostdriver/evnt_handler.h"
#include "core/delay/delay.h"
#include "core/gpio/gpio.h"

/*=========================================================================
    CC3000 API PARAM VALUES

    Simple wrapper defines to abstract away CC3000 API param values
    -----------------------------------------------------------------------*/
    #define WIFI_DISABLE              (0)
    #define WIFI_ENABLE               (1)
    #define WIFI_TIMEOUT_CONNECT      (45000) /* ~45s */
/*=========================================================================*/


/*=========================================================================
    CC3000 STATUS FLAGS
    -----------------------------------------------------------------------*/
    #define WIFI_STATUS_DISCONNECTED  (0)
    #define WIFI_STATUS_SCANING       (1)
    #define WIFI_STATUS_CONNECTING    (2)
    #define WIFI_STATUS_CONNECTED     (3)
/*=========================================================================*/


/*=========================================================================
    PRINTF RETARGET

    Some wifi commands generate printf output (SSID scan, etc.), so this
    simple macro can be reused to redirect the output to the correct
    peripheral or output device
    -----------------------------------------------------------------------*/
    #define WIFI_PRINTF(...)          printf(__VA_ARGS__)
/*=========================================================================*/


/*=========================================================================
    WIFI ASSERT/DEBUG OUTPUT

    Macro to check the return value of CC3000 API functions, and report an
    error if we get an unexpected return value
    -----------------------------------------------------------------------*/
    #define CC3000_SUCCESS            (0)
    #define WIFI_PRINTF_DBG(...)      printf(__VA_ARGS__)
    #define WIFI_CHECK_SUCCESS(func, msg, error) \
      do \
      {  \
        if ( (func) != CC3000_SUCCESS ) \
        { \
          WIFI_PRINTF_DBG(msg); \
          return (error); \
        } \
      } while(0)
/*=========================================================================*/


/*=========================================================================
    SMARTCONFIG VALUES
    -----------------------------------------------------------------------*/
    /* AES Key for secure connections w/SmartConfig = 0123456789012345 */
    char _wifi_ASEsecurity_key[] = { 0x30, 0x31, 0x32, 0x33,
                                     0x34, 0x35, 0x36, 0x37,
                                     0x38, 0x39, 0x30, 0x31,
                                     0x32, 0x33, 0x34, 0x35 };
    char _wifi_sc_deviceName[] = "CC3000";
    char _wifi_sc_prefix[]     = { 'T', 'T', 'T' };
    volatile unsigned char _wifi_stopSmartConfig;
    static int _wifi_smartConfigFailure = 0;
/*=========================================================================*/


/*=========================================================================
    PING VALUES
    -----------------------------------------------------------------------*/
    uint8_t _wifi_pingReportsReceived;
    netapp_pingreport_args_t _wifi_pingReport;
/*=========================================================================*/


/*=========================================================================
    ASYNC FLAGS (CONNECTED/DHCP/etc.)
    -----------------------------------------------------------------------*/
    volatile unsigned long _wifi_smartConfigFinished,
                           _wifi_connected,
                           _wifi_dhcp,
                           _wifi_okToShutdown,
                           _wifi_dhcpConfigured,
                           _wifi_socketCheck;
/*=========================================================================*/


/*=========================================================================
    MISC GUARD FLAGS
    -----------------------------------------------------------------------*/
    static int _wifi_initialised = 0;

    #define WIFI_CHECK_INIT() \
      do \
      { \
        err_t _err; \
        if (!_wifi_initialised) \
        { \
          _err = wifi_init(0); \
          if (_err) return _err; \
        } \
      } while (0)
/*=========================================================================*/

/* Callback functions for the CC3000 HAL (set in UsynchCallback) */
void  wifi_UsynchCallback       (long lEventType, char * data, unsigned char length);
char *wifi_sendDriverPatch      (unsigned long *length);
char *wifi_sendBootLoaderPatch  (unsigned long *length);
char *wifi_sendWLFWPatch        (unsigned long *length);
long  wifi_readWlanInterruptPin (void);
void  wifi_wlanInterruptEnable  (void);
void  wifi_wlanInterruptDisable (void);
void  wifi_writeWlanPin         (unsigned char val);

/**************************************************************************/
/*!
    @brief  Helper function to reverse the byte order in a 4 byte array
*/
/**************************************************************************/
void wifi_reverseByteOrder(uint8_t bytes[4])
{
  uint8_t rev[4];

  rev[0] = bytes[3];
  rev[1] = bytes[2];
  rev[2] = bytes[1];
  rev[3] = bytes[0];

  memcpy(bytes, rev, 4);
}

/**************************************************************************/
/*!
    @brief  Helper function to convert four bytes to a U32 IP value
*/
/**************************************************************************/
uint32_t wifi_ip2u32(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
  uint32_t ip = d;

  ip <<= 8;
  ip |= c;
  ip <<= 8;
  ip |= b;
  ip <<= 8;
  ip |= a;

  return ip;
}

/**************************************************************************/
/*!
    @brief  The function returns a pointer to the driver patch:
            since there is no patch in the host - it returns 0

    @param  Length    pointer to the length
*/
/**************************************************************************/
char *wifi_sendDriverPatch(unsigned long *length)
{
  *length = 0;
  return NULL ;
}

/**************************************************************************/
/*!
    @brief  The function returns a pointer to the boot loader patch:
            since there is no patch in the host - it returns 0

    @param  Length     pointer to the length
*/
/**************************************************************************/
char *wifi_sendBootLoaderPatch(unsigned long *length)
{
  *length = 0;
  return NULL ;
}

/**************************************************************************/
/*!
    @brief  The function returns a pointer to the FW patch:
            since there is no patch in the host - it returns 0

    @param  Length     pointer to the length
*/
/**************************************************************************/
char *wifi_sendWLFWPatch(unsigned long *length)
{
  *length = 0;
  return NULL ;
}

/**************************************************************************/
/*!
    @brief  Callback function to read the status of interrupt pin

    @return Returns the pin state (1 = high, 0 = low)
*/
/**************************************************************************/
long wifi_readWlanInterruptPin(void)
{
  return (long) GPIOGetPinValue(CFG_CC3000_IRQ_PORT, CFG_CC3000_IRQ_PIN);
}

/**************************************************************************/
/*!
    @brief  Callback function to enable the interrupt line on the CC3000
*/
/**************************************************************************/
void wifi_wlanInterruptEnable()
{
  GPIOPinIntEnable(2, 0);
}

/**************************************************************************/
/*!
    @brief  Callback function to disable the interrupt line on the CC3000
*/
/**************************************************************************/
void wifi_wlanInterruptDisable()
{
  GPIOPinIntDisable(2, 0);
}

/**************************************************************************/
/*!
    @brief  Callback function to set the CC3000_EN pin (enable the CC3000)

    @param  val    Enable or disable CC3000 pin
*/
/**************************************************************************/
void wifi_writeWlanPin(unsigned char val)
{
  GPIOSetBitValue(CFG_CC3000_EN_PORT, CFG_CC3000_EN_PIN, val);
}

/**************************************************************************/
/*!
    @brief  Handles asynchronous events from the CC3000 module

    @param  lEventType  Event type
    @param  data        data of event
    @param  length      data length
*/
/**************************************************************************/
void wifi_UsynchCallback(long lEventType, char * data, unsigned char length)
{
  /* SmartConfig configuration process finished */
  if (lEventType == HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE)
  {
    _wifi_smartConfigFinished = 1;
    _wifi_stopSmartConfig = 1;
  }

  /* AP connection established */
  if (lEventType == HCI_EVNT_WLAN_UNSOL_CONNECT)
  {
    _wifi_connected = 1;
  }

  /* AP disconnected */
  if (lEventType == HCI_EVNT_WLAN_UNSOL_DISCONNECT)
  {
    _wifi_connected = 0;
    _wifi_dhcp = 0;
    _wifi_dhcpConfigured = 0;
  }

  /* DHCP complete (we have an IP address!) */
  if (lEventType == HCI_EVNT_WLAN_UNSOL_DHCP)
  {
    _wifi_dhcp = 1;
  }

  /* Safe to shut down now */
  if (lEventType == HCI_EVENT_CC3000_CAN_SHUT_DOWN)
  {
    _wifi_okToShutdown = 1;
  }

  if (lEventType == HCI_EVNT_BSD_TCP_CLOSE_WAIT)
  {
    _wifi_socketCheck = 1;
  }

  /* Ping report results */
  if (lEventType == HCI_EVNT_WLAN_ASYNC_PING_REPORT)
  {
    _wifi_pingReportsReceived++;
    memcpy(&_wifi_pingReport, data, length);
  }
}

/**************************************************************************/
/*!
    @brief  Initialises the CC3000

    @note   Possible error message are:

            - ERROR_CC3000_WLAN_EVENT_MASK
            - ERROR_NONE
*/
/**************************************************************************/
err_t wifi_init(unsigned short cRequestPatch)
{
  /* Make sure the CC3000 SPI block is initialised (see spi.c) */
  init_spi();

  /* Pass in the callback functions for the CC3000 HAL */
  wlan_init(wifi_UsynchCallback,
            wifi_sendWLFWPatch,
            wifi_sendDriverPatch,
            wifi_sendBootLoaderPatch,
            wifi_readWlanInterruptPin,
            wifi_wlanInterruptEnable,
            wifi_wlanInterruptDisable,
            wifi_writeWlanPin);

  /* Start the module */
  wlan_start(cRequestPatch);

  /* Set this up here for convenience sake w/SmartConfig */
  wlan_smart_config_set_prefix((char*) _wifi_sc_prefix);

  /* Don't store the connection details */
  wlan_ioctl_set_connection_policy(WIFI_DISABLE, WIFI_DISABLE, WIFI_DISABLE);

  /* Delete any previous profiles that are stored in NVMEM */
  wlan_ioctl_del_profile(255);

  /* Setup the event masks (async events we don't want, see hci.h) */
  WIFI_CHECK_SUCCESS(wlan_set_event_mask(
                     HCI_EVNT_WLAN_KEEPALIVE         |
                     HCI_EVNT_WLAN_UNSOL_INIT        |
                     // HCI_EVNT_WLAN_ASYNC_PING_REPORT |
                     // HCI_EVNT_WLAN_TX_COMPLETE,
                     HCI_EVNT_BSD_TCP_CLOSE_WAIT),
                     "WLAN Set Event Mask FAIL",
                     ERROR_CC3000_WLAN_EVENT_MASK);

  /* Initialised! */
  _wifi_initialised = 1;

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Gets the two byte firmware version number

    @note   Possible error message are:

            - ERROR_CC3000_WLAN_EVENT_MASK (wifi_init)
            - ERROR_CC3000_NVMEM_READ
            - ERROR_NONE

    @code

    uint8_t major, minor;
    if (!wifi_getFirmwareVersion(&major, &minor))
    {
      printf("Firmware : v%d.%d%s", major, minor, CFG_PRINTF_NEWLINE);
    }

    @endcode
*/
/**************************************************************************/
err_t wifi_getFirmwareVersion(uint8_t *major, uint8_t *minor)
{
  uint8_t version[2];

  WIFI_CHECK_INIT();
  WIFI_CHECK_SUCCESS(nvmem_read_sp_version(version),
                     "Failed reading firmware version",
                     ERROR_CC3000_NVMEM_READ);

  *major = version[0];
  *minor = version[1];

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Gets the MAC address for the CC3000 module

    @note   Possible error message are:

            - ERROR_CC3000_WLAN_EVENT_MASK (wifi_init)
            - ERROR_CC3000_NVMEM_READ
            - ERROR_NONE

    @code

    uint8_t macAddress[6] = { 0, 0, 0, 0, 0, 0 };

    if (!wifi_getMacAddress(macAddress))
    {
      printf("MAC Address : %02X:%02X:%02X:%02X:%02X:%02X%s",
        macAddress[0], macAddress[1], macAddress[2],
        macAddress[3], macAddress[4], macAddress[5],
        CFG_PRINTF_NEWLINE);
    }

    @endcode
*/
/**************************************************************************/
err_t wifi_getMacAddress(uint8_t macAddress[6])
{
  WIFI_CHECK_INIT();
  WIFI_CHECK_SUCCESS(nvmem_read(NVMEM_MAC_FILEID, 6, 0, macAddress),
      "Read MAC address failed", ERROR_CC3000_NVMEM_READ);

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Sets the MAC address for the CC3000 module

    @note   Possible error message are:

            - ERROR_CC3000_WLAN_EVENT_MASK (wifi_init)
            - ERROR_CC3000_NVMEM_SET_MAC_ADDRESS
            - ERROR_NONE
*/
/**************************************************************************/
err_t wifi_setMacAddress(uint8_t macAddress[6])
{
  WIFI_CHECK_INIT();

  WIFI_CHECK_SUCCESS(netapp_config_mac_adrress(macAddress),
                     "Failed updating MAC address",
                     ERROR_CC3000_NVMEM_SET_MAC_ADDRESS);

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief   Enables or disables an SSID scan

    @param   time  Stops scanning if equal to 0, otherwise scans for the
                   specified number of milliseconds

    @note    Possible error message are

             - ERROR_CC3000_WLAN_EVENT_MASK (wifi_init)
             - ERROR_CC3000_WLAN_SET_SCAN_PARAM
             - ERROR_NONE
*/
/**************************************************************************/
err_t wifi_ssidScan(uint32_t time)
{
  const unsigned long intervalTime[16] = { 2000, 2000, 2000, 2000,
                                           2000, 2000, 2000, 2000,
                                           2000, 2000, 2000, 2000,
                                           2000, 2000, 2000, 2000  };

  WIFI_CHECK_INIT();
  WIFI_CHECK_SUCCESS(wlan_ioctl_set_scan_params(time, 20, 100, 5, 0x7FF,
                -120, 0, 300, (unsigned long *)&intervalTime),
                "Failed setting SSID scan params",
                ERROR_CC3000_WLAN_SET_SCAN_PARAM);

  delay(time + 500);

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief   Performs and SSID scan and display the results using printf

    @note    Possible error message are

             - ERROR_CC3000_WLAN_EVENT_MASK (wifi_init)
             - ERROR_CC3000_WLAN_SET_SCAN_PARAM (wifi_ssidScan)
             - ERROR_CC3000_WLAN_GET_SCAN_RESULT
             - ERROR_NONE

    @code

    printf("Performing an SSID scan (~4s)%s", CFG_PRINTF_NEWLINE);
    wifi_displaySSIDResults();

    @endcode
*/
/**************************************************************************/
err_t wifi_displaySSIDResults(void)
{
  uint16_t i, x;
  ResultStruct_t resultBuff;
  uint8_t valid, rssiValue, securityMode, ssidLen;

  WIFI_CHECK_INIT();

  /* Reboot first if there was a failed SmartConfig attempt */
  if (_wifi_smartConfigFailure)
  {
    wlan_stop();
    delay(1000);
    wlan_start(0);
    _wifi_smartConfigFailure = 0;
  }

  /* Scan for 4 seconds then stop */
  ASSERT_STATUS_MESSAGE(wifi_ssidScan(4000), "SSID scan init failed");

  /* Get the scan results */
  WIFI_CHECK_SUCCESS(wlan_ioctl_get_scan_results(0, (uint8_t*)&resultBuff),
                    "Failed reading SSID scan results",
                    ERROR_CC3000_WLAN_GET_SCAN_RESULT);

  /* Iterate over the scan results */
  i = resultBuff.num_networks;
  while (i)
  {
    i--;

    /* Pull out the individual records from the result buffer */
    valid        = (resultBuff.rssiByte & (~0xFE));
    rssiValue    = (resultBuff.rssiByte >> 1);
    securityMode = (resultBuff.Sec_ssidLen & (~0xFC));
    ssidLen      = (resultBuff.Sec_ssidLen >> 2);

    /* Display details of valid access points */
    if (valid)
    {
      WIFI_PRINTF("%-20s", "SSID Name:");
      for (x = 0; x < ssidLen; x++)
      {
        WIFI_PRINTF("%c", resultBuff.ssid_name[x]);
      }
      WIFI_PRINTF(CFG_PRINTF_NEWLINE);
      WIFI_PRINTF("%-20s%d%s", "Security Mode:", securityMode, CFG_PRINTF_NEWLINE);
      WIFI_PRINTF("%-20s%d%s", "RSSI Value:", rssiValue, CFG_PRINTF_NEWLINE);
      WIFI_PRINTF(CFG_PRINTF_NEWLINE);
    }

    /* Read the next entry */
    WIFI_CHECK_SUCCESS(wlan_ioctl_get_scan_results(0, (uint8_t*)&resultBuff),
                "Failed reading SSID scan results",
                ERROR_CC3000_WLAN_GET_SCAN_RESULT);
  };

  /* Stop the SSID scan (time = 0) */
  ASSERT_STATUS_MESSAGE(wifi_ssidScan(0), "SSID scan stop failed");

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief   Connect to a secure AP

    @param   sec     Security mode (0 = open, 1 = WEP, 2 = WPA, 3 = WPA2)
    @param   ssid    Buffer to store the SSID name
    @param   ssidlen SSID length
    @param   key     Buffer to store the AP authentication key
    @param   keylen  Key length (max 16 bytes)

    @note    The CC3000 is limited to 16 bytes for key length!

    @note    Possible error message are

             - ERROR_CC3000_WLAN_EVENT_MASK (wifi_init)
             - ERROR_INVALIDPARAMETER
             - ERROR_CC3000_WLAN_SET_CONNECT_POLICY
             - ERROR_CC3000_WLAN_CONNECT
             - ERROR_CC3000_CONNECT_TIMEOUT
             - ERROR_NONE

    @code

    uint8_t *ssid = "TESTNETWORK";
    uint8_t *key  = "abcdefghijklmnop";
    uint8_t sec   = 3;

    printf("Connecting to %s (30s timeout) ...%s", ssid, CFG_PRINTF_NEWLINE);

    error = wifi_connectSecure(3, ssid, strlen(ssid), key, strlen(key));

    if (!error)
    {
      printf("Connected!%s", CFG_PRINTF_NEWLINE);
      // ...
    }

    @endcode
*/
/**************************************************************************/
err_t wifi_connectSecure(int32_t sec, int8_t *ssid, int32_t ssidlen,
                           int8_t *key, int32_t keylen)
{
  WIFI_CHECK_INIT();

  /* Check parameter lengths to make sure they're safe for the API */
  ASSERT(sec     > -1,  ERROR_INVALIDPARAMETER);
  ASSERT(sec     <= 3,  ERROR_INVALIDPARAMETER);
  ASSERT(ssidlen <= 32, ERROR_INVALIDPARAMETER);
  ASSERT(keylen  <= 16, ERROR_INVALIDPARAMETER);

  /* Don't automatically reconnect to this AP */
  WIFI_CHECK_SUCCESS(wlan_ioctl_set_connection_policy(WIFI_DISABLE, WIFI_DISABLE, WIFI_DISABLE),
                    "Set connection policy failed", ERROR_CC3000_WLAN_SET_CONNECT_POLICY);

  /* Wait a bit for the previous call to complete */
  delay(500);

  /* Try to establish a connection */
  WIFI_CHECK_SUCCESS(wlan_connect(sec, (char *)ssid, ssidlen, NULL, (unsigned char *)key, keylen),
                    "Connection failed", ERROR_CC3000_WLAN_CONNECT);

  /* Wait for a connection and an IP address from DHCP */
  uint32_t timeout = 0;
  while ((!_wifi_connected) || (!_wifi_dhcp))
  {
    timeout++;
    if(timeout > WIFI_TIMEOUT_CONNECT / 10)
    {
      return ERROR_CC3000_CONNECT_TIMEOUT;
    }
    delay(10);
  }

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief   Starts the SmartConfig process and waits for a connection

    @param   enableAES  True is AES is used for SmartConfig

    @note    Possible error message are:

             - ERROR_CC3000_WLAN_EVENT_MASK (wifi_init)
             - ERROR_CC3000_WLAN_SET_CONNECT_POLICY
             - ERROR_CC3000_WLAN_DEL_CONNECT_PROFILE
             - ERROR_CC3000_WLAN_DISCONNECT
             - ERROR_CC3000_NVMEM_CREATE_ENTRY
             - ERROR_CC3000_NVMEM_WRITE
             - ERROR_CC3000_SMARTCONFIG_SET_PREFIX
             - ERROR_CC3000_SMARTCONFIG_START
             - ERROR_CC3000_CONNECT_TIMEOUT
             - ERROR_CC3000_SMARTCONFIG_PROCESS
             - ERROR_NONE
*/
/**************************************************************************/
err_t wifi_startSmartConfig(bool enableAES)
{
  uint8_t  loop    = 0;
  uint32_t timeout = 0;

  _wifi_smartConfigFinished = 0;
  _wifi_connected = 0;
  _wifi_dhcp = 0;
  _wifi_okToShutdown = 0;

  WIFI_CHECK_INIT();

  /* Reset the connection policy */
  WIFI_CHECK_SUCCESS(wlan_ioctl_set_connection_policy(WIFI_DISABLE, WIFI_DISABLE, WIFI_DISABLE),
                     "Set Connection policy FAIL",
                     ERROR_CC3000_WLAN_SET_CONNECT_POLICY);

  /* Erase all previous connections stored in NVMEM */
  WIFI_CHECK_SUCCESS(wlan_ioctl_del_profile(255),
                     "Delete Profile FAIL",
                     ERROR_CC3000_WLAN_DEL_CONNECT_PROFILE);

  /* Disconnect from the current AP if we're connected to something */
  while (wlan_ioctl_statusget() == WIFI_STATUS_CONNECTED)
  {
    WIFI_CHECK_SUCCESS(wlan_disconnect(),
                       "Unable to disconnect from the AP",
                       ERROR_CC3000_WLAN_DISCONNECT);
    hci_unsolicited_event_handler();
  }

  /* Reboot the CC3000 */
  wlan_stop();
  delay(1000);
  wlan_start(0);

  /* Create a new entry for the AES encryption key */
  WIFI_CHECK_SUCCESS(nvmem_create_entry(NVMEM_AES128_KEY_FILEID, 16),
                    "Unable to create the AES key entry",
                    ERROR_CC3000_NVMEM_CREATE_ENTRY);

  /* Write the AES key to NVMEM */
  WIFI_CHECK_SUCCESS(aes_write_key((uint8_t * )(&_wifi_ASEsecurity_key[0])),
                     "Unable to commit the AES key",
                     ERROR_CC3000_NVMEM_WRITE);

  /* Set the prefix */
  WIFI_CHECK_SUCCESS(wlan_smart_config_set_prefix((char * )&_wifi_sc_prefix),
                     "Unable to set the SmartConfig prefix",
                     ERROR_CC3000_SMARTCONFIG_SET_PREFIX);

  /* Start the SmartConfig process */
  WIFI_CHECK_SUCCESS(wlan_smart_config_start(enableAES),
                     "wlan_smart_config_start failed",
                     ERROR_CC3000_SMARTCONFIG_START);

  /* Wait for the SmartConfig process to complete */
  while (_wifi_smartConfigFinished == 0)
  {
    // Waiting here for the SIMPLE_CONFIG_DONE event
    timeout++;
    if (timeout > WIFI_TIMEOUT_CONNECT / 10)
    {
      _wifi_smartConfigFailure = 1;
      return ERROR_CC3000_CONNECT_TIMEOUT;
    }
    delay(10);
  }

  if (enableAES)
  {
    WIFI_CHECK_SUCCESS(wlan_smart_config_process(),
                       "wlan_smart_config_process failed",
                       ERROR_CC3000_SMARTCONFIG_PROCESS);
  }

  /* Configure the device to automatically connect to the AP */
  WIFI_CHECK_SUCCESS(wlan_ioctl_set_connection_policy(WIFI_DISABLE, WIFI_DISABLE, WIFI_ENABLE),
                     "Set connection policy FAIL",
                     ERROR_CC3000_WLAN_SET_CONNECT_POLICY);

  /* Reset the CC3000 */
  wlan_stop();
  delay(1000);
  wlan_start(0);

  /* Setup the event masks (async events we don't want, see hci.h) */
  WIFI_CHECK_SUCCESS(wlan_set_event_mask(
                     HCI_EVNT_WLAN_KEEPALIVE         |
                     HCI_EVNT_WLAN_UNSOL_INIT        |
                     // HCI_EVNT_WLAN_ASYNC_PING_REPORT |
                     // HCI_EVNT_WLAN_TX_COMPLETE,
                     HCI_EVNT_BSD_TCP_CLOSE_WAIT),
                     "WLAN Set Event Mask FAIL",
                     ERROR_CC3000_WLAN_EVENT_MASK);

  /* Small delay to wait for the CC3000 to connect to the AP */
  timeout = 0;
  delay(1000);

  /**************** Connect to the AP (time out ~30s) ***********************/

  while ((!_wifi_smartConfigFinished) || (!_wifi_connected) || (!_wifi_dhcp))
  {
    timeout ++;
    if (timeout > WIFI_TIMEOUT_CONNECT / 10)
    {
      _wifi_smartConfigFailure = 1;
      return ERROR_CC3000_CONNECT_TIMEOUT;
    }
    delay(10);
  }

  if (((_wifi_smartConfigFinished) && (_wifi_connected) && (_wifi_dhcp)))
  {
    while (loop < 5)
    {
      mdnsAdvertiser(1, (char *) _wifi_sc_deviceName, strlen(_wifi_sc_deviceName));
      loop++;
    }
  }

  _wifi_stopSmartConfig = 0;
  _wifi_smartConfigFinished = 0;

  /* Smart config was successful! */
  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Disconnect if we are connected to an AP

    @note   Possible error message are:

            - ERROR_CC3000_WLAN_EVENT_MASK (wifi_init)
            - ERROR_NONE
*/
/**************************************************************************/
err_t wifi_disconnect(void)
{
  WIFI_CHECK_INIT();

  /* Disconnect */
  wlan_disconnect();

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Gets the connection details for the module (IP, net mask,
            gateway, DHCP server and DNS server addresses)

    @note   Possible error message are:

            - ERROR_CC3000_WLAN_EVENT_MASK (wifi_init)
            - ERROR_CC3000_NETAPP_IPCONFIG
            - ERROR_NONE

    @code

    // Display the connection details
    uint8_t ip[4];
    uint8_t netmask[4];
    uint8_t gateway[4];
    uint8_t dhcp[4];
    uint8_t dns[4];

    error = wifi_getConnectionDetails(ip, netmask, gateway, dhcp, dns);

    if (!error)
    {
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

    @endcode
*/
/**************************************************************************/
err_t wifi_getConnectionDetails(uint8_t ipAddress[4], uint8_t netmask[4],
                                  uint8_t gateway[4], uint8_t dhcpServer[4],
                                  uint8_t dnsServer[4])
{
  tNetappIpconfigRetArgs ipconfig;

  WIFI_CHECK_INIT();

  /* Request the connection details */
  netapp_ipconfig(&ipconfig);

  /* Make sure we have an IP address (connection was lost, etc.) */
  ASSERT(ipconfig.aucIP[3] != 0, ERROR_CC3000_NETAPP_IPCONFIG);

  /* Push the values into the placeholder variables */
  memcpy(ipAddress,   ipconfig.aucIP,     4);
  wifi_reverseByteOrder(ipAddress);
  memcpy(netmask,     ipconfig.aucIP+4,   4);
  wifi_reverseByteOrder(netmask);
  memcpy(gateway,     ipconfig.aucIP+8,   4);
  wifi_reverseByteOrder(gateway);
  memcpy(dhcpServer,  ipconfig.aucIP+12,  4);
  wifi_reverseByteOrder(dhcpServer);
  memcpy(dnsServer,   ipconfig.aucIP+16,  4);
  wifi_reverseByteOrder(dnsServer);

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Pings the specified IP address

    @param   ip         Four byte array containg the IP address to ping
    @param   attempts   The number of times to ping the IP address
    @param   timeout    The ping timeout in milliseconds

    @note    Possible error message are:

             - ERROR_CC3000_WLAN_EVENT_MASK (wifi_init)
             - ERROR_CC3000_NETAPP_PING
             - ERROR_CC3000_NETAPP_PING_MISSINGEVENT
             - ERROR_NONE

    @code

    // Ping www.adafruit.com 3 times
    uint8_t  pingIP[4]    = { 207, 58, 139, 247 };
    uint8_t  pingAttempts = 3;
    uint16_t pingTimeout  = 1000;

    printf("Pinging %d.%d.%d.%d %d times (%d ms timeout)%s",
      pingIP[0], pingIP[1], pingIP[2], pingIP[3],
      pingAttempts, pingTimeout, CFG_PRINTF_NEWLINE);

    error = wifi_ping(pingIP, pingAttempts, pingTimeout);

    @endcode
*/
/**************************************************************************/
err_t wifi_ping(uint8_t ip[4], uint8_t attempts, uint16_t timeout)
{
  WIFI_CHECK_INIT();

  uint32_t revIP = (ip[0] | (ip[1] << 8) | (ip[2] << 16) | (ip[3] << 24));

  /* Reset the ping result placeholders */
  _wifi_pingReportsReceived = 0;
  memset(&_wifi_pingReport, 0, sizeof(netapp_pingreport_args_t));

  /* Start the async request (response handled in wifi_UsynchCallback) */
  WIFI_CHECK_SUCCESS(netapp_ping_send(&revIP, attempts, 32, timeout),
                     "Ping failed", ERROR_CC3000_NETAPP_PING);

  /* Give the ping time to execute */
  delay( (timeout * attempts) + 500);

  /* Force the async event by requesting the ping report */
  /* See: http://e2e.ti.com/support/low_power_rf/f/851/p/214148/785692.aspx */
  netapp_ping_report();

  delay(1000);

  /* Make sure the async ping event fired */
  ASSERT(_wifi_pingReportsReceived, ERROR_CC3000_NETAPP_PING_MISSINGEVENT);

  /* Display the results */
  // WIFI_PRINTF("Packets Sent     : %u\r\n", _wifi_pingReport.packets_sent);
  WIFI_PRINTF("Packets Received : %u\r\n", _wifi_pingReport.packets_received);
  WIFI_PRINTF("Minimum Time     : %u\r\n", _wifi_pingReport.min_round_time);
  WIFI_PRINTF("Maximum Time     : %u\r\n", _wifi_pingReport.max_round_time);
  WIFI_PRINTF("Average Time     : %u\r\n", _wifi_pingReport.avg_round_time);

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Looks up the IP address of the specific host name

    @param   hostName   The string containing the host name to lookup
    @param   ip         The placeholder for the host IP address

    @note    Possible error message are:

             - ERROR_CC3000_WLAN_EVENT_MASK (wifi_init)
             - ERROR_CC3000_SOCKET_GETHOSTBYNAME
             - ERROR_NONE

    @code

    // Lookup the IP address for www.adafruit.com
    uint8_t lookupIP[4] = { 0, 0, 0, 0 };

    error = wifi_getHostByName("www.adafruit.com", lookupIP);

    if (!error)
    {
      printf("www.adafruit.com = %d.%d.%d.%d %s",
        lookupIP[0], lookupIP[1], lookupIP[2], lookupIP[3],
        CFG_PRINTF_NEWLINE);
    }

    @endcode
*/
/**************************************************************************/
err_t wifi_getHostByName(uint8_t *hostName, uint8_t ip[4])
{
  uint32_t revIP;

  WIFI_CHECK_INIT();

  /* gethostbyname returns a positive number is successful */
  ASSERT_MESSAGE(gethostbyname(hostName, (unsigned short int)strlen(hostName), &revIP) > 0,
                 ERROR_CC3000_SOCKET_GETHOSTBYNAME,
                 "Host name lookup failed");

  /* Assign IP contents */
  memcpy(ip, &revIP, 4);
  wifi_reverseByteOrder(ip);

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief   Checks if we are connected to an access point or not
*/
/**************************************************************************/
bool wifi_isConnected(void)
{
  return _wifi_connected ? true : false;
}

/**************************************************************************/
/*!
    @brief   Checks if DHCP is complete or not (do we have an IP address?)
*/
/**************************************************************************/
bool wifi_isDHCPComplete(void)
{
  return _wifi_dhcp ? true : false;
}

#endif
