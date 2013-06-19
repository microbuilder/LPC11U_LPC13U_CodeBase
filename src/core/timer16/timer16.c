/**************************************************************************/
/*!
    @file     timer16.c
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
#include "timer16.h"

volatile uint32_t timer16_0_counter[4] = {0,0,0,0};
volatile uint32_t timer16_1_counter[4] = {0,0,0,0};
volatile uint32_t timer16_0_capture[4] = {0,0,0,0};
volatile uint32_t timer16_1_capture[4] = {0,0,0,0};

// NOTE: 16-bit Timer 0 is used by src/core/delay/delay.c
// NOTE: 16-bit Timer 1 is used by src/drivers/sensors/sensorpoll.c

///**************************************************************************/
///*!
//    @brief Interrupt handler for 16-bit timer 0
//*/
///**************************************************************************/
//#if defined CFG_MCU_FAMILY_LPC11UXX
//void TIMER16_0_IRQHandler(void)
//#elif defined CFG_MCU_FAMILY_LPC13UXX
//void CT16B0_IRQHandler(void)
//#else
//  #error "timer16.c: No MCU defined"
//#endif
//{
//  /* Handle match events */
//  if (LPC_CT16B0->IR & (0x01 << 0))
//  {
//    LPC_CT16B0->IR = 0x1 << 0;
//    timer16_0_counter[0]++;
//  }
//  if (LPC_CT16B0->IR & (0x01 << 1))
//  {
//    LPC_CT16B0->IR = 0x1 << 1;
//    timer16_0_counter[1]++;
//  }
//  if (LPC_CT16B0->IR & (0x01 << 2))
//  {
//    LPC_CT16B0->IR = 0x1 << 2;
//    timer16_0_counter[2]++;
//  }
//  if (LPC_CT16B0->IR & (0x01 << 3))
//  {
//    LPC_CT16B0->IR = 0x1 << 3;
//    timer16_0_counter[3]++;
//  }
//
//  /* Handle capture events */
//  if (LPC_CT16B0->IR & (0x1 << 4))
//  {
//    LPC_CT16B0->IR = 0x1 << 4;
//    timer16_0_capture[0]++;
//  }
//  if (LPC_CT16B0->IR & (0x1 << 5))
//  {
//    LPC_CT16B0->IR = 0x1 << 5;
//    timer16_0_capture[1]++;
//  }
//  if (LPC_CT16B0->IR & (0x1 << 6))
//  {
//    LPC_CT16B0->IR = 0x1 << 6;
//    timer16_0_capture[2]++;
//  }
//  if (LPC_CT16B0->IR & (0x1 << 7))
//  {
//    LPC_CT16B0->IR = 0x1 << 7;
//    timer16_0_capture[3]++;
//  }
//
//  return;
//}

///**************************************************************************/
///*!
//    @brief Interrupt handler for 16-bit timer 1
//*/
///**************************************************************************/
//#if defined CFG_MCU_FAMILY_LPC11UXX
//void TIMER16_1_IRQHandler(void)
//#elif defined CFG_MCU_FAMILY_LPC13UXX
//void CT16B1_IRQHandler(void)
//#else
//  #error "timer16.c: No MCU defined"
//#endif
//{
//  /* Handle match events */
//  if (LPC_CT16B1->IR & (0x01 << 0))
//  {
//    LPC_CT16B1->IR = 0x1 << 0;
//    timer16_1_counter[0]++;
//  }
//  if (LPC_CT16B1->IR & (0x01 << 1))
//  {
//    LPC_CT16B1->IR = 0x1 << 1;
//    timer16_1_counter[1]++;
//  }
//  if (LPC_CT16B1->IR & (0x01 << 2))
//  {
//    LPC_CT16B1->IR = 0x1 << 2;
//    timer16_1_counter[2]++;
//  }
//  if (LPC_CT16B1->IR & (0x01 << 3))
//  {
//    LPC_CT16B1->IR = 0x1 << 3;
//    timer16_1_counter[3]++;
//  }
//
//  /* Handle capture events */
//  if (LPC_CT16B1->IR & (0x1 << 4))
//  {
//    LPC_CT16B1->IR = 0x1 << 4;
//    timer16_1_capture[0]++;
//  }
//  if (LPC_CT16B1->IR & (0x1 << 5))
//  {
//    LPC_CT16B1->IR = 0x1 << 5;
//    timer16_1_capture[1]++;
//  }
//  if (LPC_CT16B1->IR & (0x1 << 6))
//  {
//    LPC_CT16B1->IR = 0x1 << 6;
//    timer16_1_capture[2]++;
//  }
//  if (LPC_CT16B1->IR & (0x1 << 7))
//  {
//    LPC_CT16B1->IR = 0x1 << 7;
//    timer16_1_capture[3]++;
//  }
//
//  return;
//}

/**************************************************************************/
/*!
    @brief Initialises the specified 16-bit timer

    @param[in]  timer
                The 16-bit timer to use (must be 0 or 1)
*/
/**************************************************************************/
void timer16Init(uint8_t timer)
{
  uint32_t i;

  if ( timer == 0 )
  {
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 7);
    for ( i = 0; i < 4; i++ )
    {
      timer16_0_counter[i] = 0;
      timer16_0_capture[i] = 0;
    }
    #if defined CFG_MCU_FAMILY_LPC11UXX
    NVIC_EnableIRQ(TIMER_16_0_IRQn);
    #elif defined CFG_MCU_FAMILY_LPC13UXX
    NVIC_EnableIRQ(CT16B0_IRQn);
    #endif
  }
  else if ( timer == 1 )
  {
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 8);
    for ( i = 0; i < 4; i++ )
    {
      timer16_1_counter[i] = 0;
      timer16_1_capture[i] = 0;
    }
    #if defined CFG_MCU_FAMILY_LPC11UXX
    NVIC_EnableIRQ(TIMER_16_1_IRQn);
    #elif defined CFG_MCU_FAMILY_LPC13UXX
    NVIC_EnableIRQ(CT16B1_IRQn);
    #endif
  }

  return;
}

/**************************************************************************/
/*!
    @brief Enables the specified 16-bit timer

    @param[in]  timer
                The 16-bit timer to use (must be 0 or 1)
*/
/**************************************************************************/
void timer16Enable(uint8_t timer)
{
  if ( timer == 0 )
  {
    LPC_CT16B0->TCR = 1;
  }
  else if (timer == 1)
  {
    LPC_CT16B1->TCR = 1;
  }
  return;
}

/**************************************************************************/
/*!
    @brief Disables the specified 16-bit timer

    @param[in]  timer
                The 16-bit timer to use (must be 0 or 1)
*/
/**************************************************************************/
void timer16Disable(uint8_t timer)
{
  if ( timer == 0 )
  {
    LPC_CT16B0->TCR = 0;
  }
  else if (timer == 1)
  {
    LPC_CT16B1->TCR = 0;
  }
  return;
}

/**************************************************************************/
/*!
    @brief Resets the specified 16-bit timer

    @param[in]  timer
                The 16-bit timer to use (must be 0 or 1)
*/
/**************************************************************************/
void timer16Reset(uint8_t timer)
{
  uint32_t regVal;

  if ( timer == 0 )
  {
    regVal = LPC_CT16B0->TCR;
    regVal |= 0x02;
    LPC_CT16B0->TCR = regVal;
  }
  else if (timer == 1)
  {
    regVal = LPC_CT16B1->TCR;
    regVal |= 0x02;
    LPC_CT16B1->TCR = regVal;
  }
  return;
}

/**************************************************************************/
/*!
    @brief Causes a blocking delay on the specified 16-bit timer

    @param[in]  timer
                The 16-bit timer to use (must be 0 or 1)
    @param[in]  delayInTicks
                The number of clock cycles ('ticks') to wait
*/
/**************************************************************************/
void timer16DelayTicks(uint8_t timer, uint16_t delayInTicks)
{
  if (timer == 0)
  {
    LPC_CT16B0->TCR  = 0x02;            /* Reset the timer */
    LPC_CT16B0->PR   = 0x00;            /* Set prescaler to zero */
    LPC_CT16B0->PWMC = 0x00;            /* Disable PWM mode */
    LPC_CT16B0->MR0  = delayInTicks;
    LPC_CT16B0->IR   = 0xff;            /* Reset all interrrupts */
    LPC_CT16B0->MCR  = 0x04;            /* Stop the timer on match */
    LPC_CT16B0->TCR  = 0x01;            /* Start timer */

    /* Wait until delay time has elapsed */
    while (LPC_CT16B0->TCR & 0x01);
  }
  else if (timer == 1)
  {
    LPC_CT16B1->TCR  = 0x02;            /* Reset the timer */
    LPC_CT16B1->PR   = 0x00;            /* Set prescaler to zero */
    LPC_CT16B1->PWMC = 0x00;            /* Disable PWM mode */
    LPC_CT16B1->MR0  = delayInTicks;
    LPC_CT16B1->IR   = 0xff;            /* Reset all interrrupts */
    LPC_CT16B1->MCR  = 0x04;            /* Stop the timer on match */
    LPC_CT16B1->TCR  = 0x01;            /* Start timer */

    /* Wait until delay time has elapsed */
    while (LPC_CT16B1->TCR & 0x01);
  }
  return;
}

/**************************************************************************/
/*!
    @brief Configures the match register for the specified 16-bit timer

    @param[in]  timer
                The 16-bit timer to use (must be 0 or 1)
    @param[in]  matchNum
                The match register to set (must be 0..3)
    @param[in]  value
                The value to assign to the specified match register

    @code
    // Set MAT1 on timer 0 to 12000 ticks
    timer16SetMatch(0, 1, 12000);
    @endcode
*/
/**************************************************************************/
void timer16SetMatch(uint8_t timer, uint8_t matchNum, uint16_t value)
{
  if (timer)
  {
    switch (matchNum)
    {
      case 0:
        LPC_CT16B1->MR0 = value;
        break;
      case 1:
        LPC_CT16B1->MR1 = value;
        break;
      case 2:
        LPC_CT16B1->MR2 = value;
        break;
      case 3:
        LPC_CT16B1->MR3 = value;
        break;
      default:
        break;
    }
  }
  else
  {
    switch (matchNum)
    {
      case 0:
        LPC_CT16B0->MR0 = value;
        break;
      case 1:
        LPC_CT16B0->MR1 = value;
        break;
      case 2:
        LPC_CT16B0->MR2 = value;
        break;
      case 3:
        LPC_CT16B0->MR3 = value;
        break;
      default:
        break;
    }
  }
}

/**************************************************************************/
/*!
    @brief  Configures 16-bit Timer 1 for PWM output using MR0 to control
            the period, and MR1, MR2 and MR3 for duty cycle.  Duty cycle
            can be adjusted via timer16SetMatch.

    @param[in]  period
                The period in timer clock ticks

    @code
    // Setup PWM with 12,000 cycle period (MAT0 used for period)
    timer16SetPWM1(12000);

    // PWM Match Settings
    timer16SetMatch(1, 1, 12000); // MAT1 will go high on last cycle
    timer16SetMatch(1, 2, 8000);  // MAT2 will go high for final 1/3
    timer16SetMatch(1, 3, 4000);  // MAT3 will go high for final 2/3

    // Enable to PWM output
    timer16Enable(1);
    @endcode
*/
/**************************************************************************/
void timer16SetPWM1(uint16_t period)
{
  timer16Disable(1);

  /* Make sure 16-bit timer 1 is enabled */
  LPC_SYSCON->SYSAHBCLKCTRL |= (1<<8);

  /* Setup the external match register (clear on match) */
  LPC_CT16B1->EMR =  (1<<10) | (1<<8) | (1<<6) | (1<<4) |
                     (1<<0)  | (1<<1) | (1<<2) | (1<<3);

  /* Set MAT0..3 to PWM mode via the PWM Control register */
  LPC_CT16B1->PWMC = (1<<0)  | (1<<1) | (1<<2) | (1<<3);

  /* MAT0 controls period, set MAT1..3 to 50% duty cycle to start */
  timer16SetMatch(1, 0, period);
  timer16SetMatch(1, 1, period / 2);
  timer16SetMatch(1, 2, period / 2);
  timer16SetMatch(1, 3, period / 2);

  /* Reset on MR0 */
  LPC_CT16B1->MCR = 1 << 1;
}
