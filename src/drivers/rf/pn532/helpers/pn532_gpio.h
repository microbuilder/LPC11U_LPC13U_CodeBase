/**************************************************************************/
/*!
    @file     pn532_gpio.h
*/
/**************************************************************************/
#ifndef __PN532_GPIO_H__
#define __PN532_GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"

#define PN532_GPIO_RESPONSELENGTH           (16)
#define PN532_GPIO_VALIDATIONBIT            (0x80)

typedef enum pn532_gpio_e
{
  PN532_GPIO_P30 = 0,
  PN532_GPIO_P31,
  PN532_GPIO_P32,
  PN532_GPIO_P33,
  PN532_GPIO_P34,
  PN532_GPIO_P35
} pn532_gpio_t;

pn532_error_t pn532_gpio_WriteGPIO (uint8_t pinState);
pn532_error_t pn532_gpio_ReadGPIO (uint8_t * pinState);

#ifdef __cplusplus
}
#endif 

#endif
