/**************************************************************************/
/*!
    @file     timer32.c
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

#ifdef CFG_ENABLE_TIMER32

#include "timer32.h"

/**************************************************************************/
/*!
    @brief Interrupt handler for 32-bit timer 0
*/
/**************************************************************************/
#if defined CFG_MCU_FAMILY_LPC11UXX
void TIMER32_0_IRQHandler(void)
#elif defined CFG_MCU_FAMILY_LPC13UXX
void CT32B0_IRQHandler(void)
#else
  #error "timer32.c: No MCU defined"
#endif
{
  /* Handle match events */
  if (LPC_CT32B0->IR & (0x01 << 0))
  {
    LPC_CT32B0->IR = 0x1 << 0;
  }
  if (LPC_CT32B0->IR & (0x01 << 1))
  {
    LPC_CT32B0->IR = 0x1 << 1;
  }
  if (LPC_CT32B0->IR & (0x01 << 2))
  {
    LPC_CT32B0->IR = 0x1 << 2;
  }
  if (LPC_CT32B0->IR & (0x01 << 3))
  {
    LPC_CT32B0->IR = 0x1 << 3;
  }

  /* Handle capture events */
  if (LPC_CT32B0->IR & (0x1 << 4))
  {
    LPC_CT32B0->IR = 0x1 << 4;
  }
  if (LPC_CT32B0->IR & (0x1 << 5))
  {
    LPC_CT32B0->IR = 0x1 << 5;
  }
  if (LPC_CT32B0->IR & (0x1 << 6))
  {
    LPC_CT32B0->IR = 0x1 << 6;
  }
  if (LPC_CT32B0->IR & (0x1 << 7))
  {
    LPC_CT32B0->IR = 0x1 << 7;
  }

  return;
}

/**************************************************************************/
/*!
    @brief Interrupt handler for 32-bit timer 1
*/
/**************************************************************************/
#if defined CFG_MCU_FAMILY_LPC11UXX
void TIMER32_1_IRQHandler(void)
#elif defined CFG_MCU_FAMILY_LPC13UXX
void CT32B1_IRQHandler(void)
#else
  #error "timer32.c: No MCU defined"
#endif
{
  /* Handle match events */
  if (LPC_CT32B1->IR & (0x01 << 0))
  {
    LPC_CT32B1->IR = 0x1 << 0;
  }
  if (LPC_CT32B1->IR & (0x01 << 1))
  {
    LPC_CT32B1->IR = 0x1 << 1;
  }
  if (LPC_CT32B1->IR & (0x01 << 2))
  {
    LPC_CT32B1->IR = 0x1 << 2;
  }
  if (LPC_CT32B1->IR & (0x01 << 3))
  {
    LPC_CT32B1->IR = 0x1 << 3;
  }

  /* Handle capture events */
  if (LPC_CT32B1->IR & (0x1 << 4))
  {
    LPC_CT32B1->IR = 0x1 << 4;
  }
  if (LPC_CT32B1->IR & (0x1 << 5))
  {
    LPC_CT32B1->IR = 0x1 << 5;
  }
  if (LPC_CT32B1->IR & (0x1 << 6))
  {
    LPC_CT32B1->IR = 0x1 << 6;
  }
  if (LPC_CT32B1->IR & (0x1 << 7))
  {
    LPC_CT32B1->IR = 0x1 << 7;
  }

  return;
}

/**************************************************************************/
/*!
    @brief Initialises the specified 32-bit timer

    @param[in]  timer
                The 32-bit timer to use (must be 0 or 1)
*/
/**************************************************************************/
void timer32Init(uint8_t timer)
{
  if ( timer == 0 )
  {
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<9);
    #if defined CFG_MCU_FAMILY_LPC11UXX
    NVIC_EnableIRQ(TIMER_32_0_IRQn);
    #elif defined CFG_MCU_FAMILY_LPC13UXX
    NVIC_EnableIRQ(CT32B0_IRQn);
    #endif
  }
  else if ( timer == 1 )
  {
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<10);
    #if defined CFG_MCU_FAMILY_LPC11UXX
    NVIC_EnableIRQ(TIMER_32_1_IRQn);
    #elif defined CFG_MCU_FAMILY_LPC13UXX
    NVIC_EnableIRQ(CT32B1_IRQn);
    #endif
  }

  return;
}

/**************************************************************************/
/*!
    @brief Enables the specified 32-bit timer

    @param[in]  timer
                The 32-bit timer to use (must be 0 or 1)
*/
/**************************************************************************/
void timer32Enable(uint8_t timer)
{
  if ( timer == 0 )
  {
    LPC_CT32B0->TCR = 1;
  }
  else if (timer == 1)
  {
    LPC_CT32B1->TCR = 1;
  }
  return;
}

/**************************************************************************/
/*!
    @brief Disables the specified 32-bit timer

    @param[in]  timer
                The 32-bit timer to use (must be 0 or 1)
*/
/**************************************************************************/
void timer32Disable(uint8_t timer)
{
  if ( timer == 0 )
  {
    LPC_CT32B0->TCR = 0;
  }
  else if (timer == 1)
  {
    LPC_CT32B1->TCR = 0;
  }
  return;
}

/**************************************************************************/
/*!
    @brief Resets the specified 32-bit timer

    @param[in]  timer
                The 32-bit timer to use (must be 0 or 1)
*/
/**************************************************************************/
void timer32Reset(uint8_t timer)
{
  uint32_t regVal;

  if ( timer == 0 )
  {
    regVal = LPC_CT32B0->TCR;
    regVal |= 0x02;
    LPC_CT32B0->TCR = regVal;
  }
  else if (timer == 1)
  {
    regVal = LPC_CT32B1->TCR;
    regVal |= 0x02;
    LPC_CT32B1->TCR = regVal;
  }
  return;
}

/**************************************************************************/
/*!
    @brief Causes a blocking delay on the specified 32-bit timer

    @param[in]  timer
                The 32-bit timer to use (must be 0 or 1)
    @param[in]  delayInMs
                The number of milliseconds to wait
*/
/**************************************************************************/
void timer32DelayMs(uint8_t timer, uint32_t delayInMs)
{
  if (timer == 0)
  {
    LPC_CT32B0->TCR  = 0x02;            /* Reset the timer */
    LPC_CT32B0->PR   = 0x00;            /* Set prescaler to zero */
    LPC_CT32B0->PWMC = 0x00;            /* Disable PWM mode */
    LPC_CT32B0->MR0  = delayInMs * (SystemCoreClock / 1000);
    LPC_CT32B0->IR   = 0xff;            /* Reset all interrrupts */
    LPC_CT32B0->MCR  = 0x04;            /* Stop the timer on match */
    LPC_CT32B0->TCR  = 0x01;            /* Start timer */

    /* Wait until delay time has elapsed */
    while (LPC_CT32B0->TCR & 0x01);
  }
  else if (timer == 1)
  {
    LPC_CT32B1->TCR  = 0x02;            /* Reset the timer */
    LPC_CT32B1->PR   = 0x00;            /* Set prescaler to zero */
    LPC_CT32B1->PWMC = 0x00;            /* Disable PWM mode */
    LPC_CT32B1->MR0  = delayInMs * (SystemCoreClock / 1000);
    LPC_CT32B1->IR   = 0xff;            /* Reset all interrrupts */
    LPC_CT32B1->MCR  = 0x04;            /* Stop the timer on match */
    LPC_CT32B1->TCR  = 0x01;            /* Start timer */

    /* Wait until delay time has elapsed */
    while (LPC_CT32B1->TCR & 0x01);
  }
  return;
}

/**************************************************************************/
/*!
    @brief Causes a blocking delay on the specified 32-bit timer

    @param[in]  timer
                The 32-bit timer to use (must be 0 or 1)
    @param[in]  delayInTicks
                The number of clock cycles ('ticks') to wait
*/
/**************************************************************************/
void timer32DelayTicks(uint8_t timer, uint32_t delayInTicks)
{
  if (timer == 0)
  {
    LPC_CT32B0->TCR  = 0x02;            /* Reset the timer */
    LPC_CT32B0->PR   = 0x00;            /* Set prescaler to zero */
    LPC_CT32B0->PWMC = 0x00;            /* Disable PWM mode */
    LPC_CT32B0->MR0  = delayInTicks;
    LPC_CT32B0->IR   = 0xff;            /* Reset all interrrupts */
    LPC_CT32B0->MCR  = 0x04;            /* Stop the timer on match */
    LPC_CT32B0->TCR  = 0x01;            /* Start timer */

    /* Wait until delay time has elapsed */
    while (LPC_CT32B0->TCR & 0x01);
  }
  else if (timer == 1)
  {
    LPC_CT32B1->TCR  = 0x02;            /* Reset the timer */
    LPC_CT32B1->PR   = 0x00;            /* Set prescaler to zero */
    LPC_CT32B1->PWMC = 0x00;            /* Disable PWM mode */
    LPC_CT32B1->MR0  = delayInTicks;
    LPC_CT32B1->IR   = 0xff;            /* Reset all interrrupts */
    LPC_CT32B1->MCR  = 0x04;            /* Stop the timer on match */
    LPC_CT32B1->TCR  = 0x01;            /* Start timer */

    /* Wait until delay time has elapsed */
    while (LPC_CT32B1->TCR & 0x01);
  }
  return;
}

/**************************************************************************/
/*!
    @brief Configures the match register for the specified 32-bit timer

    @param[in]  timer
                The 32-bit timer to use (must be 0 or 1)
    @param[in]  matchNum
                The match register to set (must be 0..3)
    @param[in]  value
                The value to assign to the specified match register

    @code
    // Set MAT1 on timer 0 to 12000 ticks
    timer32SetMatch(0, 1, 12000);
    @endcode
*/
/**************************************************************************/
void timer32SetMatch(uint8_t timer, uint8_t matchNum, uint32_t value)
{
  if (timer)
  {
    switch (matchNum)
    {
      case 0:
        LPC_CT32B1->MR0 = value;
        break;
      case 1:
        LPC_CT32B1->MR1 = value;
        break;
      case 2:
        LPC_CT32B1->MR2 = value;
        break;
      case 3:
        LPC_CT32B1->MR3 = value;
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
        LPC_CT32B0->MR0 = value;
        break;
      case 1:
        LPC_CT32B0->MR1 = value;
        break;
      case 2:
        LPC_CT32B0->MR2 = value;
        break;
      case 3:
        LPC_CT32B0->MR3 = value;
        break;
      default:
        break;
    }
  }
}

/**************************************************************************/
/*!
    @brief  Configures 32-bit Timer 0 for PWM output using MR0 to control
            the period, and MR1, MR2 and MR3 for duty cycle.  Duty cycle
            can be adjusted via timer32SetMatch.

    @param[in]  period
                The period in timer clock ticks

    @code
    // Set 1.25 to T320_MAT1
    LPC_IOCON->PIO1_25 &= ~0x07;
    LPC_IOCON->PIO1_25 |= 0x01;

    // Set 1.26 to T320_MAT2
    LPC_IOCON->PIO1_26 &= ~0x07;
    LPC_IOCON->PIO1_26 |= 0x01;

    // Set 1.27 to T320_MAT3
    LPC_IOCON->PIO1_27 &= ~0x07;
    LPC_IOCON->PIO1_27 |= 0x01;

    // Setup PWM with 12,000 cycle period (MAT0 used for period)
    timer32SetPWM0(12000);

    // PWM Match Settings
    timer32SetMatch(0, 1, 12000); // MAT1 will go high on last cycle
    timer32SetMatch(0, 2, 8000);  // MAT2 will go high for final 1/3
    timer32SetMatch(0, 3, 4000);  // MAT3 will go high for final 2/3

    // Enable to PWM output
    timer32Enable(0);
    @endcode
*/
/**************************************************************************/
void timer32SetPWM0(uint32_t period)
{
  timer32Disable(0);

  /* Make sure 32-bit timer 0 is enabled */
  LPC_SYSCON->SYSAHBCLKCTRL |= (1<<9);

  /* Setup the external match register (clear on match) */
  LPC_CT32B0->EMR =  (1<<10) | (1<<8) | (1<<6) | (1<<4) |
                     (1<<0)  | (1<<1) | (1<<2) | (1<<3);

  /* Set MAT0..3 to PWM mode via the PWM Control register */
  LPC_CT32B0->PWMC = (1<<0)  | (1<<1) | (1<<2) | (1<<3);

  /* MAT0 controls period, set MAT1..3 to 50% duty cycle to start */
  timer32SetMatch(0, 0, period);
  timer32SetMatch(0, 1, period / 2);
  timer32SetMatch(0, 2, period / 2);
  timer32SetMatch(0, 3, period / 2);

  /* Reset on MR0 */
  LPC_CT32B0->MCR = 1 << 1;
}

#endif
