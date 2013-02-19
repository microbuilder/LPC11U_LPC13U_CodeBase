/**************************************************************************/
/*!
    @file     pn532_config.h
*/
/**************************************************************************/
#ifndef __PN532_CONFIG_H__
#define __PN532_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

pn532_error_t pn532_config_SetPassiveActivationRetries(uint8_t maxRetries);

#ifdef __cplusplus
}
#endif 

#endif
