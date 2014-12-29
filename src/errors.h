/**************************************************************************/
/*!
    @defgroup Errors Error Handling

    @brief    System wide error codes and error-handling

    @details

    In order to improve the stability of the system the code base uses a
    global error type that any critical function returns upon completion.

    Functions that execute properly return ERROR_NONE, which equals '0' so
    that we can perform a simple check like 'if(error) { ... }'.
*/
/**************************************************************************/

/**************************************************************************/
/*!
    @file     errors.h
    @author   K. Townsend (microBuilder.eu)

    @ingroup  Errors

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
#ifndef _ERRORS_H_
#define _ERRORS_H_

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************/
/*!
    Common error messages used across the system
*/
/**************************************************************************/
typedef enum
{
  /*=======================================================================
    GENERIC ERRORS                                         0x0000 .. 0x00FF
    -----------------------------------------------------------------------
    These error codes can be used anywhere in the system
    -----------------------------------------------------------------------*/
    ERROR_NONE                                  = 0x0,  /**< Indicates no error occurred */
    ERROR_OPERATIONTIMEDOUT                     = 0x1,  /**< Operation timed out before completion */
    ERROR_ADDRESSOUTOFRANGE                     = 0x2,  /**< The supplied address is out of range */
    ERROR_BUFFEROVERFLOW                        = 0x3,  /**< The proposed action will cause a buffer overflow */
    ERROR_INVALIDPARAMETER                      = 0x4,  /**< An invalid parameter value was provided */
    ERROR_DEVICENOTINITIALISED                  = 0x5,  /**< Attempting to execute a function on an uninitialised peripheral */
    ERROR_UNEXPECTEDVALUE                       = 0x6,  /**< An unexpected value was found inside a function */
  /*=======================================================================*/


  /*=======================================================================
    I2C ERRORS                                             0x0100 .. 0x010F
    -----------------------------------------------------------------------
    Errors related to the I2C bus
    -----------------------------------------------------------------------*/
    ERROR_I2C_DEVICENOTFOUND                    = 0x101,  /**< Device didn't ACK after an I2C transfer */
    ERROR_I2C_NOACK                             = 0x102,  /**< No ACK signal received during an I2C transfer */
    ERROR_I2C_TIMEOUT                           = 0x103,  /**< Device timed out waiting for response (missing pullups?) */
  /*=======================================================================*/


  /*=======================================================================
    CHIBI ERRORS                                           0x0110 .. 0x011F
    -----------------------------------------------------------------------
    Errors relating the Chibi/802.15.4 wireless communication
    -----------------------------------------------------------------------*/
    ERROR_CHIBI_NOACK                           = 0x111,  /**< No ACK from destination node (bad address or offline?) */
    ERROR_CHIBI_CHANACCESSFAILURE               = 0x112,  /**< Channel access failure */
    ERROR_CHIBI_PAYLOADOVERFLOW                 = 0x113,  /**< Payload exceeds buffer size */
  /*=======================================================================*/


  /*=======================================================================
    SIMPLE BINARY PROTOCOL ERRORS                          0x0120 .. 0x013F
    -----------------------------------------------------------------------
    Errors relating to the simple binary protocol (/src//protocol)
    -----------------------------------------------------------------------*/
    ERROR_PROT_INVALIDMSGTYPE                   = 0x121,  /**< Unexpected msg type encountered */
    ERROR_PROT_INVALIDCOMMANDID                 = 0x122,  /**< Unknown or out of range command ID */
    ERROR_PROT_INVALIDPAYLOAD                   = 0x123,  /**< Message payload has a problem (invalid len, etc.) */
  /*=======================================================================*/


  /*=======================================================================
    RTC ERRORS                                             0x0140 .. 0x014F
    -----------------------------------------------------------------------
    Real time clock (RTC) errors
    -----------------------------------------------------------------------*/
    ERROR_RTC_OUTOFEPOCHRANGE                   = 0x141,  /**< RTC time must be kept in epoch range */
  /*=======================================================================*/


  /*=======================================================================
    TIMESPAN ERRORS                                        0x0150 .. 0x015F
    -----------------------------------------------------------------------
    Errors relating to the timespan.c helper class
    -----------------------------------------------------------------------*/
    ERROR_TIMESPAN_OUTOFRANGE                   = 0x151,  /**< timespan_t must be kept within int64_t nanosecond range */
  /*=======================================================================*/


  /*=======================================================================
    FATFS ERRORS                                           0x0160 .. 0x016F
    -----------------------------------------------------------------------
    These error codes can be used anywhere in the system
    -----------------------------------------------------------------------*/
    ERROR_FATFS_NODISK                          = 0x161,  /**< No SD card present (based on polling the CD pin) */
    ERROR_FATFS_INITFAILED                      = 0x162,  /**< Failed trying to initialise FatFS */
    ERROR_FATFS_FAILEDTOMOUNTDRIVE              = 0x163,  /**< Failed trying to mount the drive (not FAT formatted?) */
    ERROR_FATFS_UNABLETOCREATEFILE              = 0x164,  /**< Failed trying to create a file on the SD card */
    ERROR_FATFS_UNABLETOOPENFILE                = 0x165,  /**< Failed trying to open the specified file */
    ERROR_FATFS_WRITEFAILED                     = 0x166,  /**< Failed trying to write to the SD card */
  /*=======================================================================*/


  /*=======================================================================
    USB ERRORS                                             0x0200 .. 0x02FF
    -----------------------------------------------------------------------
    USB error codes
    TODO: Harmonise LPC codes with error IDs here!
    -----------------------------------------------------------------------*/

  /*=======================================================================*/


  /*=======================================================================
    CC3000 ERRORS                                          0x0400 .. 0x04FF
    -----------------------------------------------------------------------
    Errors relating to the CC3000 Wifi module and wifi communication
    -----------------------------------------------------------------------*/
    ERROR_CC3000_CONNECT_TIMEOUT                = 0x401,  /**< Timed out waiting for a connection or DHCP */
    ERROR_CC3000_SMARTCONFIG_SET_PREFIX         = 0x411,  /**< Smart config failed calling 'wlan_smart_config_set_prefix' */
    ERROR_CC3000_SMARTCONFIG_START              = 0x412,  /**< Smart config failed calling 'wlan_smart_config_start' */
    ERROR_CC3000_SMARTCONFIG_PROCESS            = 0x413,  /**< Smart config failed called 'wlan_smart_config_process' */
    ERROR_CC3000_NVMEM_READ                     = 0x421,  /**< Failed reading from NVMEM */
    ERROR_CC3000_NVMEM_WRITE                    = 0x422,  /**< Failed writing to NVMEM */
    ERROR_CC3000_NVMEM_CREATE_ENTRY             = 0x423,  /**< Failed creating an NVMEM entry via 'nvmem_create_entry' */
    ERROR_CC3000_NVMEM_SET_MAC_ADDRESS          = 0x424,  /**< Failed creating an NVMEM entry via 'netapp_config_mac_adrress' */
    ERROR_CC3000_WLAN_EVENT_MASK                = 0x431,  /**< Failed calling 'wlan_set_event_mask' */
    ERROR_CC3000_WLAN_SET_CONNECT_POLICY        = 0x432,  /**< Failed calling 'wlan_ioctl_set_connection_policy' */
    ERROR_CC3000_WLAN_DEL_CONNECT_PROFILE       = 0x433,  /**< Failed calling 'wlan_ioctl_del_profile' */
    ERROR_CC3000_WLAN_DISCONNECT                = 0x434,  /**< Failed calling 'wlan_disconnect' */
    ERROR_CC3000_WLAN_CONNECT                   = 0x435,  /**< Failed calling 'wlan_connect' */
    ERROR_CC3000_WLAN_SET_SCAN_PARAM            = 0x436,  /**< Failed calling 'wlan_ioctl_set_scan_params' */
    ERROR_CC3000_WLAN_GET_SCAN_RESULT           = 0x437,  /**< Failed calling 'wlan_ioctl_get_scan_results' */
    ERROR_CC3000_NETAPP_MACADDRESS_CONFIG       = 0x441,  /**< Failed calling 'netapp_config_mac_adrress' */
    ERROR_CC3000_NETAPP_GETHOSTBYNAMEFAILED     = 0x442,  /**< Failed calling 'gethostbyname' */
    ERROR_CC3000_NETAPP_IPCONFIG                = 0x443,  /**< Failed calling 'netapp_ipconfig' */
    ERROR_CC3000_NETAPP_PING                    = 0x444,  /**< Failed calling 'netapp_ping_send' */
    ERROR_CC3000_NETAPP_PING_MISSINGEVENT       = 0x445,  /**< No ping response was received in the async event handler */
    ERROR_CC3000_SOCKET_CREATE                  = 0x451,  /**< Error creating a socket */
    ERROR_CC3000_SOCKET_BIND                    = 0x452,  /**< Failed calling 'bind' for socket */
    ERROR_CC3000_SOCKET_LISTEN                  = 0x453,  /**< Failed calling 'listen' for socket */
    ERROR_CC3000_SOCKET_CLOSE                   = 0x454,  /**< Failed calling 'closesocket' */
    ERROR_CC3000_SOCKET_RECEIVE                 = 0x455,  /**< Socket receive error */
    ERROR_CC3000_SOCKET_SEND                    = 0x456,  /**< Failed calling 'send' for socket */
    ERROR_CC3000_SOCKET_SEND_LENMISMATCH        = 0x457,  /**< Length mismatch during 'send' for socket (data len != send len) */
    ERROR_CC3000_SOCKET_GETHOSTBYNAME           = 0x458,  /**< Failed calling 'gethostbyname' */
  /*=======================================================================*/
} err_t;

#ifdef __cplusplus
}
#endif

#endif
