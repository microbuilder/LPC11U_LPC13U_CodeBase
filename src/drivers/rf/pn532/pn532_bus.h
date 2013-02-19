/**************************************************************************/
/*!
    @file     pn532_bus.h
*/
/**************************************************************************/

#ifndef __PN532_BUS_H__
#define __PN532_BUS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"
#include "pn532.h"

#if defined(CFG_ENABLE_UART)
  // #define PN532_BUS_UART
#endif

#if defined(CFG_ENABLE_I2C)
  #define PN532_BUS_I2C
#endif

#if defined PN532_BUS_UART && defined PN532_BUS_I2C
  #error "Only one target can be defined for the PN532 (PN532_BUS_I2C or PN532_BUS_UART)"
#endif

#define PN532_NORMAL_FRAME__DATA_MAX_LEN      (254)
#define PN532_NORMAL_FRAME__OVERHEAD          (8)
#define PN532_EXTENDED_FRAME__DATA_MAX_LEN    (264)
#define PN532_EXTENDED_FRAME__OVERHEAD        (11)
#define PN532_BUFFER_LEN                      (PN532_EXTENDED_FRAME__DATA_MAX_LEN + PN532_EXTENDED_FRAME__OVERHEAD)

#define PN532_UART_BAUDRATE                   (115200)

#define PN532_I2C_ADDRESS                     (0x48)
#define PN532_I2C_READBIT                     (0x01)
#define PN532_I2C_READYTIMEOUT                (20)    // Max number of attempts to read Ready bit (see UM 5-Nov-2007 Section 6.2.4)

// Generic interface for the different serial buses available on the PN532
error_t       pn532_bus_HWInit(void);
pn532_error_t pn532_bus_SendCommand(const byte_t * pbtData, const size_t szData);
pn532_error_t pn532_bus_ReadResponse(byte_t * pbtResponse, size_t * pszRxLen);
pn532_error_t pn532_bus_Wakeup(void);

#ifdef __cplusplus
}
#endif 

#endif
