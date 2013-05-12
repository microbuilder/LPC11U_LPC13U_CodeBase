/**************************************************************************/
/*! 
    @file     swspi.c
    @author   K. Townsend (microBuilder.eu)

    @section DESCRIPTION

    Basic bit-banged SPI driver.

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2012, K. Townsend
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
#include "swspi.h"
#include "core/gpio/gpio.h"

/**************************************************************************/
/*!

*/
/**************************************************************************/
void swspiDelay(uint32_t ticks)
{
  uint32_t i;

  for (i = 0; i < ticks/4; i++)
  {
    ASM("nop");
  }
}

/**************************************************************************/
/*!
    @section EXAMPLE

    @code

    static swspi_config_t devicename_swspi_cfg;
    ...
    // Setup SW SPI settings and HW
    devicename_swspi_cfg.miso_port  = 1;
    devicename_swspi_cfg.miso_pin   = 26;
    devicename_swspi_cfg.mosi_port  = 1;
    devicename_swspi_cfg.mosi_pin   = 27;
    devicename_swspi_cfg.sck_port   = 1;
    devicename_swspi_cfg.sck_pin    = 28;
    devicename_swspi_cfg.cpol       = 1;      // Clock is high between frames
    devicename_swspi_cfg.cpha       = 1;      // Slave reads on rising clock edge, Master on falling edge
    devicename_swspi_cfg.sck_delay  = 0;      // Delay between SCK state changes (high to low, etc.) in ticks

    // Setup the SW SPI pin (assumes they are already set to GPIO)
    swspiInit(&devicename_swspi_cfg);

    uint8_t results = 0x00;

    DEVICENAME_SPI_ENABLE;                                      // User-defined macro to Enable SPI
    results = swspiTransferByte(&devicename_swspi_cfg, 0xFF);   // Transfer 0xFF and return read to 'results'
    DEVICENAME_SPI_DISABLE;                                     // User-defined macro to Disable SPI

    @endcode

*/
/**************************************************************************/
void swspiInit(swspi_config_t *config)
{
  // Note: This function assumes that the IOCON registers have already
  // been configured to set the pins to GPIO!
  LPC_GPIO->DIR[config->miso_port] &= ~(1 << config->miso_pin);
  LPC_GPIO->DIR[config->mosi_port] |=  (1 << config->mosi_pin);
  LPC_GPIO->DIR[config->sck_port] |=  (1 << config->sck_pin);
}

/**************************************************************************/
/*!
    This function both reads and writes data. For write operations,
    include the data to be written as an argument. For read operations,
    use dummy data as an argument. The returned value is the data read
    on the MISO pin.
*/
/**************************************************************************/
uint8_t swspiTransferByte(swspi_config_t *config, uint8_t byte)
{
  int8_t i, b;
  uint8_t output = 0x00;

  // ToDo: This can be significantly optimised by only checking the
  // mode once and writing optimised code for each of the four modes
  // Current code yields <1MHz @ 72MHz!
  for (i=8; i>0; i--)
  {
    // Drive SCK early for CPHA = 1
    if (config->cpha)
    {
      // Drive SCK low if CPOL == 1, high if CPOL == 0
      if (config->cpol)
      {
        LPC_GPIO->CLR[config->sck_port] = (1 << config->sck_pin);
      }
      else
      {
        LPC_GPIO->SET[config->sck_port] = (1 << config->sck_pin);
      }
      if (config->sck_delay)
      {
        swspiDelay(config->sck_delay);
      }
    }

    // Write current bit
    if (byte & (1 << (i-1)))
    {
      LPC_GPIO->SET[config->mosi_port] = (1 << config->mosi_pin);
    }
    else
    {
      LPC_GPIO->CLR[config->mosi_port] = (1 << config->mosi_pin);
    }

    // Mid SCK
    if (config->cpha)
    {
      // Drive SCK high if CPOL == 1, low if CPOL == 0
      if (config->cpol)
      {
        LPC_GPIO->SET[config->sck_port] = (1 << config->sck_pin);
      }
      else
      {
        LPC_GPIO->CLR[config->sck_port] = (1 << config->sck_pin);
      }
    }
    else
    {
      // Drive SCK low if CPOL == 1, high if CPOL = 0
      if (config->cpol)
      {
        LPC_GPIO->CLR[config->sck_port] = (1 << config->sck_pin);
      }
      else
      {
        LPC_GPIO->SET[config->sck_port] = (1 << config->sck_pin);
      }
    }
    if (config->sck_delay)
    {
      swspiDelay(config->sck_delay);
    }

    // Read current bit and shift it into output at the appropriate location
    b = GPIOGetPinValue(config->miso_port, config->miso_pin);
    output |= ((b & 0x01) << (i-1));

    // Drive SCK later for CPHA = 0
    if (!(config->cpha))
    {
      // Drive SCK high if CPOL == 1, low if CPOL == 0
      if (config->cpol)
      {
        LPC_GPIO->SET[config->sck_port] = (1 << config->sck_pin);
      }
      else
      {
        LPC_GPIO->CLR[config->sck_port] = (1 << config->sck_pin);
      }
      if (config->sck_delay)
      {
        swspiDelay(config->sck_delay);
      }
    }
  }

  return output;
}
