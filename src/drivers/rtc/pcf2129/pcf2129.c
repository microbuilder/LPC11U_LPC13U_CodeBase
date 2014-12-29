/**************************************************************************/
/*!
    @file     pcf2129.c
    @author   K. Townsend (microBuilder.eu)

    @brief    Drivers for NXP's PCF2129A temp-compensated RTC

    @section DESCRIPTION

    The PCF2129A is a CMOS Real Time Clock (RTC) and calendar with an
    integrated Temperature Compensated Crystal (Xtal) Oscillator (TCXO)
    and a 32.768 kHz quartz crystal optimized for very high accuracy and
    very low power consumption.

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
#include "projectconfig.h"

#ifdef CFG_RTC

#include <string.h>
#include "pcf2129.h"
#include "core/gpio/gpio.h"

extern volatile uint8_t   I2CMasterBuffer[I2C_BUFSIZE];
extern volatile uint8_t   I2CSlaveBuffer[I2C_BUFSIZE];
extern volatile uint32_t  I2CReadLength, I2CWriteLength;

static bool _pcf2129Initialised = false;
static void (*_pcf2129Callback)(void) = NULL;

/**************************************************************************/
/*!
    @brief  Writes the specified number of bytes over I2C
*/
/**************************************************************************/
err_t pcf2129WriteBytes(uint8_t reg, uint8_t *buffer, size_t length)
{
  uint32_t i;

  /* Try to avoid buffer overflow */
  ASSERT(length <= I2C_BUFSIZE - 2, ERROR_BUFFEROVERFLOW);

  /* Fill write buffer */
  for ( i = 2; i < length+2; i++ )
  {
    I2CMasterBuffer[i] = buffer[i-2];
  }

  /* Write transaction */
  I2CWriteLength = 2+length;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = PCF2129_ADDRESS;
  I2CMasterBuffer[1] = reg;
  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Reads the specified number of bytes over I2C
*/
/**************************************************************************/
err_t pcf2129ReadBytes(uint8_t reg, uint8_t *buffer, size_t length)
{
  uint32_t i;

  /* Try to avoid buffer overflow */
  ASSERT(length <= I2C_BUFSIZE, ERROR_BUFFEROVERFLOW);

  /* Read and write need to be handled in separate transactions or the
     PCF2129 increments the current register one step ahead of where
     we should be. */

  /* Write transaction */
  I2CWriteLength = 2;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = PCF2129_ADDRESS;
  I2CMasterBuffer[1] = reg;
  i2cEngine();

  /* Read transaction */
  I2CWriteLength = 0;
  I2CReadLength = length;
  I2CMasterBuffer[0] = PCF2129_ADDRESS | PCF2129_READBIT;
  /* Check if we got an ACK or TIMEOUT error */
  ASSERT_I2C_STATUS(i2cEngine());

  /* Fill the buffer with the I2C response */
  for ( i = 0; i < length; i++ )
  {
    buffer[i] = I2CSlaveBuffer[i];
  }

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C
*/
/**************************************************************************/
err_t pcf2129Write8 (uint8_t reg, uint8_t value)
{
  uint8_t buffer = value;
  return pcf2129WriteBytes(reg, &buffer, 1);
}

/**************************************************************************/
/*!
    @brief  Reads a single byte over I2C
*/
/**************************************************************************/
err_t pcf2129Read8(uint8_t reg, uint8_t *result)
{
  return pcf2129ReadBytes(reg, result, 1);
}

/**************************************************************************/
/*!
    @brief  Initialises the I2C block
*/
/**************************************************************************/
err_t pcf2129Init(void)
{
  uint8_t result;

  /* Initialise I2C */
  i2cInit(I2CMASTER);

  /* Ping the I2C device first to see if it exists! */
  ASSERT(i2cCheckAddress(PCF2129_ADDRESS), ERROR_I2C_DEVICENOTFOUND);

  /* Set CONTROL1 register (0x00)
     ====================================================================
     BIT  Symbol    Description                                   Default
     ---  ------    --------------------------------------------- -------
       7  EXT_TEST  0 = Normal mode, 1 = External clock test mode       0
       6  --        RESERVED
       5  STOP      0 = RTC clock runs, 1 = RTC clock stopped           0
       4  TSF1      0 = No timestamp interrupt,                         0
                    1 = Flag set when TS input is driven to an
                        intermediate level between power supply
                        and ground. (Flag must be cleared to
                        clear interrupt.)
       3  POR_OVRD  0 = Power on reset override disabled                0
                    1 = Power on reset override enabled
       2  12_24     0 = 24 hour mode, 1 = 12 hour mode                  0
       1  MI        0 = Minute interrupt disabled, 1 = enabled          0
       0  SI        0 = Second interrupt disabled, 1 = enabled          0 */

  ASSERT_STATUS(pcf2129Write8(PCF2129_REG_CONTROL1, 0x00));

  /* Set CONTROL2 register (0x01)
     ====================================================================
     BIT  Symbol    Description                                   Default
     ---  ------    --------------------------------------------- -------
       7  MSF       0 = No minute or second interrupt generated,        0
                    1 = Flag set when minute or second interrupt
                        generated. (Flag must be cleared to clear
                        interrupt.)
       6  WDTF      0 = No watchdog timer interrupt or reset            0
                    1 = Flag set when watchdog timer interrupt or
                        reset generated. (Flag cannot be cleared
                        by using the interface [read only])
       5  TSF2      0 = No timestamp interrupt generated                0
                    1 = Flag set when TS input is driven to GND.
                        (Flag must be cleared to clear interrupt)
       4  AF        0 = No alarm interrupt generated,
                    1 = Flag set when alarm triggered. (Flag must       0
                        be cleared to clear interrupt)
       3  --        RESERVED
       2  TSIE      0 = No interrupt generated from timestamp           0
                        interrupt
                    1 = Interrupt generated when timestamp flag
                        set
       1  AIE       0 = No interrupt generated from the alarm           0
                        flag
                    1 = Interrupt generated when alarm flag set
       0  --        RESERVED                                              */

  ASSERT_STATUS(pcf2129Write8(PCF2129_REG_CONTROL2, 0x00));

  /* Set CONTROL3 register (0x02)
     ====================================================================
     BIT  Symbol    Description                                   Default
     ---  ------    --------------------------------------------- -------
     7-5  PWRMNG    Control of the battery switch-over, battery       000
                    low detection, and extra power fail detection
                    function:
                    000 Battery switch-over function is enabled
                        in standard mode; battery low detection
                        function is enabled
                    001 Battery switch-over function is enabled
                        in standard mode; battery low detection
                        function is disabled
                    010 Battery switch-over function is enabled
                        in standard mode; battery low detection
                        function is disabled
                    011 Battery switch-over function is enabled
                        in direct switching mode; battery low
                        detection function is enabled
                    100 Battery switch-over function is enabled
                        in direct switching mode; battery low
                        detection function is disabled
                    101 Battery switch-over function is enabled
                        in direct switching mode; battery low
                        detection function is disabled
                    111 Battery switch-over function is disabled,
                        only one power supply (VDD); battery low
                        detection function is disabled
       4  BTSE      0 = No timestamp when battery switch-over           0
                        occurs
                    1 = Time-stamped when battery switch-over
                        occurs
       3  BF        0 = No battery switch-over interrupt                0
                        generated
                    1 = Flag set when battery switch-over occurs
                        (Flag must be cleared to clear interrupt)
       2  BLF       0 = Battery status OK (no battery low               0
                        interrupt generated).
                    1 = Battery status low (flag cannot be
                        cleared using the interface)
       1  BIE       0 = No interrupt generated from the battery         0
                        flag (BF)
                    1 = Interrupt generated when BF is set
       0  BLIE      0 = No interrupt generated from the battery         0
                        low flag (BLF)
                    1 = Interrupt generated when BLF is set               */
  ASSERT_STATUS(pcf2129Write8(PCF2129_REG_CONTROL2, 0x00));

  /* Setup the IRQ pin */
  #if PCF2129_INT_ENABLED
    /* Setup the second and minute interrupts to pulse (avoids having to clear them) */
    /* TI_TP (bit 5) = 1 for pulse, TF (bits 1:0) = 11 for 1/60Hz (default) */
    ASSERT_STATUS(pcf2129Write8(PCF2129_REG_WATCHDOG_TIM_CTRL, 0x23));

    /* INT is open drain so we need to enable the pullup on the pin */
    PCF2129_INT_SETPULLUP;

    /* Set interrupt/gpio pin to input */
    LPC_GPIO->DIR[PCF2129_INT_PORT] &= ~(1 << PCF2129_INT_PIN);

    /* Set RTC INT pin as edge sensitive, and active on the falling edge */
    GPIOSetPinInterrupt( PCF2129_INT_FLEXIRQNUM, PCF2129_INT_PORT, PCF2129_INT_PIN, 0, 0 );

    /* Allow the interrupt to wake the device up from power down */
    // LPC_SYSCON->STARTERP0 |= (0x1 << PCF2129_INT_FLEXIRQNUM);

    /* Adjust the interrupt priority if necessary */
    // NVIC_SetPriority(FLEX_INT1_IRQn, 2);

    /* Enable interrupt on falling edge */
    GPIOPinIntEnable( PCF2129_INT_FLEXIRQNUM, 0 );

    /* Clear any possible interrupts */
    ASSERT_STATUS(pcf2129Read8(PCF2129_REG_CONTROL2, &result));
    ASSERT_STATUS(pcf2129Write8(PCF2129_REG_CONTROL2, 0x00));
  #endif

  _pcf2129Initialised = true;

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Registers the optional callback function that will be called
            in the interrupt handler

    @section EXAMPLE

    @code

    uint32_t _rtcCounter;
    void rtcCounter(void)
    {
      _rtcCounter++;
    }

    // ...

    // Try to initialise the PCF2129 RTC
    if (pcf2129Init())
    {
      printf("PCF2129 failed to initialise");
    }
    else
    {
      // Setup an INT callback to rtcCounter
      pcf2129SetCallback (&rtcCounter);

      // Setup the INT line to generate an interrupt every second and on ALARM
      pcf2129SetInterrupt(PCF2129_INTEVENT_SECONDTIMER | PCF2129_INTEVENT_ALARM);
    }

    @endcode
*/
/**************************************************************************/
void pcf2129SetCallback (void (*pFunc)(void))
{
  _pcf2129Callback = pFunc;
}

/**************************************************************************/
/*!
    @brief  Reads the current time from the RTC

    @section EXAMPLE

    @code

    // Try to initialise the PCF2129 RTC
    if (pcf2129Init())
    {
      printf("PCF2129 failed to initialise");
    }
    else
    {
      // Constantly read the time
      rtcTime_t time;
      while(1)
      {
        pcf2129ReadTime(&time);
        printf("%02d:%02d:%02d - %04d/%02d/%02d%s", time.hours, time.minutes, time.seconds, time.years+1900, time.months+1, time.days, CFG_PRINTF_NEWLINE);
        printf("Epoch Time: %u%s", rtcToEpochTime(&time), CFG_PRINTF_NEWLINE);
      }
    }

    @endcode
*/
/**************************************************************************/
err_t pcf2129ReadTime(rtcTime_t *time)
{
  if (!_pcf2129Initialised )
  {
    ASSERT_STATUS(pcf2129Init());
  }

  /* The PCF2129 will auto-increment the register after a read or
     write so we only need to pass the first register and set the
     read length to get the full RTC time. */
  ASSERT_STATUS(pcf2129ReadBytes(PCF2129_REG_SECONDS, (uint8_t *)time, sizeof(rtcTime_t)));

  /* Convert time values to decimal from BCD */
  time->seconds  = rtcBCDToDec(time->seconds & 0x7F);
  time->minutes  = rtcBCDToDec(time->minutes);
  time->hours    = rtcBCDToDec(time->hours);
  time->days     = rtcBCDToDec(time->days);
  time->weekdays = rtcBCDToDec(time->weekdays);
  time->months   = rtcBCDToDec(time->months);
  time->years    = rtcBCDToDec(time->years) + 100; /* PCF year is BCD so 0..99, rtcTime_t is years since 1900 */

  return ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Sets the time on the RTC to the supplied value

    @section EXAMPLE

    @code

    // Try to initialise the PCF2129 RTC
    if (pcf2129Init())
    {
      printf("PCF2129 failed to initialise");
    }
    else
    {
      // Set time to 10:04:00, 4 September 2012 (24-hour time)
      rtcTime_t time;
      rtcCreateTime(2012, RTC_MONTHS_SEPTEMBER, 4, 10, 4, 0, 0, &time);
      pcf2129SetTime(time);
    }

    @endcode
*/
/**************************************************************************/
err_t pcf2129SetTime(rtcTime_t time)
{
  if (!_pcf2129Initialised )
  {
    ASSERT_STATUS(pcf2129Init());
  }

  /* ToDo: Disable RTC while the time is updated? */

  /* Convert time values to BCD format used by the RTC */
  time.seconds  = rtcDecToBCD(time.seconds);
  time.minutes  = rtcDecToBCD(time.minutes);
  time.hours    = rtcDecToBCD(time.hours);
  time.days     = rtcDecToBCD(time.days);
  time.weekdays = rtcDecToBCD(time.weekdays);
  time.months   = rtcDecToBCD(time.months);
  time.years    = rtcDecToBCD(time.years - 100);

  /* The PCF2129 will auto-increment the register after a read or
     write so we only need to pass the first register and set the
     write length to set the full RTC time. */
  return pcf2129WriteBytes(PCF2129_REG_SECONDS, (uint8_t *)&time, sizeof(rtcTime_t));
}

/**************************************************************************/
/*!
    @brief  Sets up the RTC interrupt for the specified event

    @section EXAMPLE

    @code

    if (pcf2129Init())
    {
      printf("PCF2129 failed to initialise");
    }
    else
    {
      // Setup the INT line to generate an interrupt every second and on ALARM
      pcf2129SetInterrupt(PCF2129_INTEVENT_SECONDTIMER | PCF2129_INTEVENT_ALARM);
    }

    @endcode
*/
/**************************************************************************/
err_t pcf2129SetInterrupt(pcf2129_INTEvent_t eventFlags)
{
  uint8_t config[3];

  if (!_pcf2129Initialised )
  {
    ASSERT_STATUS(pcf2129Init());
  }

  /* Read the current config bytes */
  ASSERT_STATUS(pcf2129ReadBytes(PCF2129_REG_CONTROL1, config, 3));

  /* Set SI on REG_CONTROL1 (bit 0) */
  if (eventFlags & PCF2129_INTEVENT_SECONDTIMER)
    config[0] |= (1<<0);
  else
    config[0] &= ~(1<<0);

  /* Set MI on REG_CONTROL1 (bit 1) */
  if (eventFlags & PCF2129_INTEVENT_MINUTETIMER)
    config[0] |= (1<<1);
  else
    config[0] &= ~(1<<1);

  /* Set AIE on REG_CONTROL2 (bit 1) */
  if (eventFlags & PCF2129_INTEVENT_ALARM)
    config[1] |= (1<<1);
  else
    config[1] &= ~(1<<1);

  /* Set TSIE on REG_CONTROL2 (bit 2) */
  if (eventFlags & PCF2129_INTEVENT_TIMESTAMP)
    config[1] |= (1<<2);
  else
    config[1] &= ~(1<<2);

  /* Set BIE on REG_CONTROL3 (bit 1) */
  if (eventFlags & PCF2129_INTEVENT_BATTERYSWITCH)
    config[2] |= (1<<1);
  else
    config[2] &= ~(1<<1);

  /* Set BLIE on REG_CONTROL3 (bit 0) */
  if (eventFlags & PCF2129_INTEVENT_BATTERYLOW)
    config[2] |= (1<<0);
  else
    config[2] &= ~(1<<0);

  /* Write the updated config bits back to the RTC */
  return pcf2129WriteBytes(PCF2129_REG_CONTROL1, config, 3);
}

/**************************************************************************/
/*!
    PCF2129 INT IRQ Handler
*/
/**************************************************************************/
#if defined CFG_MCU_FAMILY_LPC11UXX
void FLEX_INT1_IRQHandler(void)
#elif defined CFG_MCU_FAMILY_LPC13UXX
void PIN_INT1_IRQHandler(void)
#else
  #error "pcf2129.c: No MCU defined"
#endif
{
  /* Make sure the right flag is set in the int status register */
  if ( LPC_GPIO_PIN_INT->IST & (0x1<<1) )
  {
    /* Falling Edge */
    if ( ( LPC_GPIO_PIN_INT->FALL & (0x1<<1) ) && ( LPC_GPIO_PIN_INT->IENF & (0x1<<1) ) )
    {
      /* Call the callback function if present */
      if (NULL != _pcf2129Callback )
      {
        _pcf2129Callback();
      }
      LPC_GPIO_PIN_INT->FALL = 0x1<<1;
    }
    /* Clear the FLEX/PIN interrupt */
    LPC_GPIO_PIN_INT->IST = 0x1<<1;
  }
  return;
}

#endif
