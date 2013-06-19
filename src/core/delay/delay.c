/**************************************************************************/
/*!
    @file     delay.c
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

#include "delay.h"

/* RTX claims systick so we need to use TIMER16[0] instead */
#define DELAY_USE_TIMER16_0      (1)

#ifdef CFG_SDCARD
#include "drivers/storage/fatfs/diskio.h"
volatile uint32_t fatTicks = 0;
#endif

volatile uint32_t delayTicks = 0;
volatile uint32_t delayRollovers = 0;

#if DELAY_USE_TIMER16_0
/**************************************************************************/
/*!
    @brief Interrupt handler for 16-bit timer 0
*/
/**************************************************************************/
#if defined CFG_MCU_FAMILY_LPC11UXX
void TIMER16_0_IRQHandler(void)
#elif defined CFG_MCU_FAMILY_LPC13UXX
void CT16B0_IRQHandler(void)
#else
  #error "timer16.c: No MCU defined"
#endif
{
  LPC_CT16B0->IR = 0x1 << 0;  /* Clear MAT0 */

  delayTicks++;
  if (delayTicks == 0) delayRollovers++;

  #ifdef CFG_SDCARD
  fatTicks++;
  if (fatTicks == 10)
  {
    fatTicks = 0;
    disk_timerproc();
  }
  #endif

  return;
}
#else
/**************************************************************************/
/*!
    @brief Systick interrupt handler
*/
/**************************************************************************/
void SysTick_Handler (void)
{
  delayTicks++;
  if (delayTicks == 0) delayRollovers++;

  #ifdef CFG_SDCARD
  fatTicks++;
  if (fatTicks == 10)
  {
    fatTicks = 0;
    disk_timerproc();
  }
  #endif

  return;
}
#endif

/**************************************************************************/
/*!
    @brief      Initialises the delay timer
*/
/**************************************************************************/
void delayInit (void)
{
  delayTicks = 0;
  delayRollovers = 0;

  #if DELAY_USE_TIMER16_0
    /* Power up the timer */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 7);

    LPC_CT16B0->TCR  = 0x02;            /* Reset the timer                */
    LPC_CT16B0->PR   = 0x03;            /* Set prescaler to four          */
    LPC_CT16B0->IR   = 0xff;            /* Reset all interrrupts          */
    LPC_CT16B0->PWMC = 0x00;            /* Disable PWM mode               */
    LPC_CT16B0->MR0  = (SystemCoreClock / 1000) >> 2; /* 1 ms w/prescalar */
    LPC_CT16B0->MCR  = (0x3<<0);        /* Interrupt and Reset on MR0     */

    /* Make sure the delay timer IRQ has one higher priority than the     *
     * lowest level to allow for some IRQs to still use delay if          *
     * absolutely necessary, even if it's not ideal. To allow this, the   *
     * IRQs calling delay must have the lowest possible priority, which   *
     * will allow the delay timer to still fire even if we're in the      *
     * lower priority interrupt handler.                                  */
    #if defined CFG_MCU_FAMILY_LPC11UXX
      NVIC_SetPriority(TIMER_16_0_IRQn, (1<<__NVIC_PRIO_BITS) - 2);
    #elif defined CFG_MCU_FAMILY_LPC13UXX
      NVIC_SetPriority(CT16B0_IRQn, (1<<__NVIC_PRIO_BITS) - 2);
    #endif
    
    /* Enable the interrupt */
    #if defined CFG_MCU_FAMILY_LPC11UXX
    NVIC_EnableIRQ(TIMER_16_0_IRQn);
    #elif defined CFG_MCU_FAMILY_LPC13UXX
    NVIC_EnableIRQ(CT16B0_IRQn);
    #endif

    LPC_CT16B0->TCR  = 0x01;            /* Start timer */
  #else
    uint32_t ticks = (SystemCoreClock / 1000);

    /* Reset counter */
    delayTicks = 0;

    /* Set reload register */
    SysTick->LOAD  = (ticks & SYSTICK_STRELOAD_MASK) - 1;

    /* Load the systick counter value */
    SysTick->VAL = 0;

    /* Make sure the systick IRQ has one higher priority than the         *
     * lowest level (to allow for some IRQs to still use delay if         *
     * absolutely necessary, even if it's not ideal). By default, the     *
     * CMSIS SysTick_Config function sets the systick IRQ to the lowest   *
     * priority at startup (see core_cm0/3.h).                            */
    #if defined CFG_MCU_FAMILY_LPC11UXX
      NVIC_SetPriority(SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 2);
    #elif defined CFG_MCU_FAMILY_LPC13UXX
      NVIC_SetPriority(SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 2);
    #endif
    
    /* Enable systick IRQ and timer */
    SysTick->CTRL = SYSTICK_STCTRL_CLKSOURCE |
                    SYSTICK_STCTRL_TICKINT |
                    SYSTICK_STCTRL_ENABLE;
  #endif
}

/**************************************************************************/
/*!
    @brief      Causes a blocking delay for 'ticks' on the timer.  For
                example: delay(100) would cause a blocking delay for 100
                ticks of the timer.

    @param[in]  ticks
                The number of ticks to cause a blocking delay for, with
                the exact duration of one tick set via delayInit()

    @note       This function takes into account the fact that the tick
                counter may eventually roll over to 0 once it reaches
                0xFFFFFFFF.
*/
/**************************************************************************/
void delay (uint32_t ticks)
{
  uint32_t curTicks;
  curTicks = delayTicks;

  // Make sure delay is at least 1 tick in case of division, etc.
  if (ticks == 0) return;

  if (curTicks > 0xFFFFFFFF - ticks)
  {
    // Rollover will occur during delay
    while (delayTicks >= curTicks)
    {
      while (delayTicks < (ticks - (0xFFFFFFFF - curTicks)));
    }
  }
  else
  {
    while ((delayTicks - curTicks) < ticks);
  }
}

/**************************************************************************/
/*!
    @brief      Returns the current value of the timer counter.
                This value is incremented by one every time an interrupt
                fires for the timer.
*/
/**************************************************************************/
uint32_t delayGetTicks(void)
{
  return delayTicks;
}

/**************************************************************************/
/*!
    @brief      Returns the current value of the timer rollover
                counter. This value is incremented by one every time the
                tick counter rolls over from 0xFFFFFFFF to 0.
*/
/**************************************************************************/
uint32_t delayGetRollovers(void)
{
  return delayRollovers;
}

/**************************************************************************/
/*!
    @brief      Returns the approximate number of seconds that the
                timer has been running.
*/
/**************************************************************************/
uint32_t delayGetSecondsActive(void)
{
  uint32_t currentTick = delayTicks;
  uint32_t rollovers = delayRollovers;
  uint32_t secsActive = currentTick / 1000;
  secsActive += rollovers * (0xFFFFFFFF / 1000);

  return secsActive;
}

