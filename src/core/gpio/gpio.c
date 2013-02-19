/****************************************************************************
 *   $Id:: gpio.c 6874 2011-03-22 01:58:31Z usb00423                        $
 *   Project: NXP LPC13Uxx GPIO example
 *
 *   Description:
 *     This file contains GPIO code example which include GPIO
 *     initialization, GPIO interrupt handler, and related APIs for
 *     GPIO access.
 *
 ****************************************************************************
* Software that is described herein is for illustrative purposes only
* which provides customers with programming information regarding the
* products. This software is supplied "AS IS" without any warranties.
* NXP Semiconductors assumes no responsibility or liability for the
* use of the software, conveys no license or title under any patent,
* copyright, or mask work right to the product. NXP Semiconductors
* reserves the right to make changes in the software without
* notification. NXP Semiconductors also make no representation or
* warranty that such application will be suitable for the specified
* use without further testing or modification.
* Permission to use, copy, modify, and distribute this software and its
* documentation is hereby granted, under NXP Semiconductors'
* relevant copyright in the software, without fee, provided that it
* is used in conjunction with NXP Semiconductors microcontrollers.  This
* copyright, permission, and disclaimer notice must appear in all copies of
* this code.
****************************************************************************/
#include "projectconfig.h"                        /* LPC13Uxx Peripheral Registers */
#include "gpio.h"

/*****************************************************************************
** Function name:                PIN_INT0_IRQHandler
**
** Descriptions:                Use one GPIO pin as interrupt source
**
** parameters:                        None
**
** Returned value:                None
**
*****************************************************************************/
#if defined CFG_GPIO_ENABLE_PINIRQ0
#if defined CFG_MCU_FAMILY_LPC11UXX
void FLEX_INT0_IRQHandler(void)
#elif defined CFG_MCU_FAMILY_LPC13UXX
void PIN_INT0_IRQHandler(void)
#else
  #error "gpio.c: No MCU defined"
#endif
{
  if ( LPC_GPIO_PIN_INT->IST & (0x1<<0) )
  {
        if ( LPC_GPIO_PIN_INT->ISEL & (0x1<<0) )
        {
          // Level Counter
        }
        else
        {
          if ( ( LPC_GPIO_PIN_INT->RISE & (0x1<<0) ) && ( LPC_GPIO_PIN_INT->IENR & (0x1<<0) ) )
          {
                // Rising Edge
                LPC_GPIO_PIN_INT->RISE = 0x1<<0;
          }
          if ( ( LPC_GPIO_PIN_INT->FALL & (0x1<<0) ) && ( LPC_GPIO_PIN_INT->IENF & (0x1<<0) ) )
          {
                // Falling Edge
                LPC_GPIO_PIN_INT->FALL = 0x1<<0;
          }
          LPC_GPIO_PIN_INT->IST = 0x1<<0;
        }
  }
  return;
}
#endif

/*****************************************************************************
** Function name:                PIN_INT1_IRQHandler
**
** Descriptions:                Use one GPIO pin as interrupt source
**
** parameters:                        None
**
** Returned value:                None
**
*****************************************************************************/
#if defined CFG_GPIO_ENABLE_PINIRQ1
#if defined CFG_MCU_FAMILY_LPC11UXX
void FLEX_INT1_IRQHandler(void)
#elif defined CFG_MCU_FAMILY_LPC13UXX
void PIN_INT1_IRQHandler(void)
#else
  #error "gpio.c: No MCU defined"
#endif
{
  if ( LPC_GPIO_PIN_INT->IST & (0x1<<1) )
  {
        if ( LPC_GPIO_PIN_INT->ISEL & (0x1<<1) )
        {
          // Level Counter
        }
        else
        {
          if ( ( LPC_GPIO_PIN_INT->RISE & (0x1<<1) ) && ( LPC_GPIO_PIN_INT->IENR & (0x1<<1) ) )
          {
                // Rising Edge
                LPC_GPIO_PIN_INT->RISE = 0x1<<1;
          }
          if ( ( LPC_GPIO_PIN_INT->FALL & (0x1<<1) ) && ( LPC_GPIO_PIN_INT->IENF & (0x1<<1) ) )
          {
                // Falling Edge
                LPC_GPIO_PIN_INT->FALL = 0x1<<1;
          }
          LPC_GPIO_PIN_INT->IST = 0x1<<1;
        }
  }
  return;
}
#endif

/*****************************************************************************
** Function name:                PIN_INT2_IRQHandler
**
** Descriptions:                Use one GPIO pin as interrupt source
**
** parameters:                        None
**
** Returned value:                None
**
*****************************************************************************/
#if defined CFG_GPIO_ENABLE_PINIRQ2
#if defined CFG_MCU_FAMILY_LPC11UXX
void FLEX_INT2_IRQHandler(void)
#elif defined CFG_MCU_FAMILY_LPC13UXX
void PIN_INT2_IRQHandler(void)
#else
  #error "gpio.c: No MCU defined"
#endif
{
  if ( LPC_GPIO_PIN_INT->IST & (0x1<<2) )
  {
        if ( LPC_GPIO_PIN_INT->ISEL & (0x1<<2) )
        {
          // Level Counter
        }
        else
        {
          if ( ( LPC_GPIO_PIN_INT->RISE & (0x1<<2) ) && ( LPC_GPIO_PIN_INT->IENR & (0x1<<2) ) )
          {
                // Rising Edge
                LPC_GPIO_PIN_INT->RISE = 0x1<<2;
          }
          if ( ( LPC_GPIO_PIN_INT->FALL & (0x1<<2) ) && ( LPC_GPIO_PIN_INT->IENF & (0x1<<2) ) )
          {
                // Falling Edge
                LPC_GPIO_PIN_INT->FALL = 0x1<<2;
          }
          LPC_GPIO_PIN_INT->IST = 0x1<<2;
        }
  }
  return;
}
#endif

/*****************************************************************************
** Function name:                PIN_INT3_IRQHandler
**
** Descriptions:                Use one GPIO pin as interrupt source
**
** parameters:                        None
**
** Returned value:                None
**
*****************************************************************************/
#if defined CFG_GPIO_ENABLE_PINIRQ3
#if defined CFG_MCU_FAMILY_LPC11UXX
void FLEX_INT3_IRQHandler(void)
#elif defined CFG_MCU_FAMILY_LPC13UXX
void PIN_INT3_IRQHandler(void)
#else
  #error "gpio.c: No MCU defined"
#endif
{
  if ( LPC_GPIO_PIN_INT->IST & (0x1<<3) )
  {
        if ( LPC_GPIO_PIN_INT->ISEL & (0x1<<3) )
        {
          // Level Counter
        }
        else
        {
          if ( ( LPC_GPIO_PIN_INT->RISE & (0x1<<3) ) && ( LPC_GPIO_PIN_INT->IENR & (0x1<<3) ) )
          {
                // Rising Edge
                LPC_GPIO_PIN_INT->RISE = 0x1<<3;
          }
          if ( ( LPC_GPIO_PIN_INT->FALL & (0x1<<3) ) && ( LPC_GPIO_PIN_INT->IENF & (0x1<<3) ) )
          {
                // Falling Edge
                LPC_GPIO_PIN_INT->FALL = 0x1<<3;
          }
          LPC_GPIO_PIN_INT->IST = 0x1<<3;
        }
  }
  return;
}
#endif

/*****************************************************************************
** Function name:                PIN_INT4_IRQHandler
**
** Descriptions:                Use one GPIO pin as interrupt source
**
** parameters:                        None
**
** Returned value:                None
**
*****************************************************************************/
#if defined CFG_GPIO_ENABLE_PINIRQ4
#if defined CFG_MCU_FAMILY_LPC11UXX
void FLEX_INT4_IRQHandler(void)
#elif defined CFG_MCU_FAMILY_LPC13UXX
void PIN_INT4_IRQHandler(void)
#else
  #error "gpio.c: No MCU defined"
#endif
{
  if ( LPC_GPIO_PIN_INT->IST & (0x1<<4) )
  {
        if ( LPC_GPIO_PIN_INT->ISEL & (0x1<<4) )
        {
          // Level Counter
        }
        else
        {
          if ( ( LPC_GPIO_PIN_INT->RISE & (0x1<<4) ) && ( LPC_GPIO_PIN_INT->IENR & (0x1<<4) ) )
          {
                // Rising Edge
                LPC_GPIO_PIN_INT->RISE = 0x1<<4;
          }
          if ( ( LPC_GPIO_PIN_INT->FALL & (0x1<<4) ) && ( LPC_GPIO_PIN_INT->IENF & (0x1<<4) ) )
          {
                // Falling Edge
                LPC_GPIO_PIN_INT->FALL = 0x1<<4;
          }
          LPC_GPIO_PIN_INT->IST = 0x1<<4;
        }
  }
  return;
}
#endif

/*****************************************************************************
** Function name:                PIN_INT5_IRQHandler
**
** Descriptions:                Use one GPIO pin as interrupt source
**
** parameters:                        None
**
** Returned value:                None
**
*****************************************************************************/
#if defined CFG_GPIO_ENABLE_PINIRQ5
#if defined CFG_MCU_FAMILY_LPC11UXX
void FLEX_INT5_IRQHandler(void)
#elif defined CFG_MCU_FAMILY_LPC13UXX
void PIN_INT5_IRQHandler(void)
#else
  #error "gpio.c: No MCU defined"
#endif
{
  if ( LPC_GPIO_PIN_INT->IST & (0x1<<5) )
  {
        if ( LPC_GPIO_PIN_INT->ISEL & (0x1<<5) )
        {
          // Level Counter
        }
        else
        {
          if ( ( LPC_GPIO_PIN_INT->RISE & (0x1<<5) ) && ( LPC_GPIO_PIN_INT->IENR & (0x1<<5) ) )
          {
                // Rising Edge
                LPC_GPIO_PIN_INT->RISE = 0x1<<5;
          }
          if ( ( LPC_GPIO_PIN_INT->FALL & (0x1<<5) ) && ( LPC_GPIO_PIN_INT->IENF & (0x1<<5) ) )
          {
                // Falling Edge
                LPC_GPIO_PIN_INT->FALL = 0x1<<5;
          }
          LPC_GPIO_PIN_INT->IST = 0x1<<5;
        }
  }
  return;
}
#endif

/*****************************************************************************
** Function name:                PIN_INT6_IRQHandler
**
** Descriptions:                Use one GPIO pin as interrupt source
**
** parameters:                        None
**
** Returned value:                None
**
*****************************************************************************/
#if defined CFG_GPIO_ENABLE_PINIRQ6
#if defined CFG_MCU_FAMILY_LPC11UXX
void FLEX_INT6_IRQHandler(void)
#elif defined CFG_MCU_FAMILY_LPC13UXX
void PIN_INT6_IRQHandler(void)
#else
  #error "gpio.c: No MCU defined"
#endif
{
  if ( LPC_GPIO_PIN_INT->IST & (0x1<<6) )
  {
        if ( LPC_GPIO_PIN_INT->ISEL & (0x1<<6) )
        {
          // Level
        }
        else
        {
          if ( ( LPC_GPIO_PIN_INT->RISE & (0x1<<6) ) && ( LPC_GPIO_PIN_INT->IENR & (0x1<<6) ) )
          {
                // Rising Edge
                LPC_GPIO_PIN_INT->RISE = 0x1<<6;
          }
          if ( ( LPC_GPIO_PIN_INT->FALL & (0x1<<6) ) && ( LPC_GPIO_PIN_INT->IENF & (0x1<<6) ) )
          {
                // Falling Edge
                LPC_GPIO_PIN_INT->FALL = 0x1<<6;
          }
          LPC_GPIO_PIN_INT->IST = 0x1<<6;
        }
  }
  return;
}
#endif

/*****************************************************************************
** Function name:                PIN_INT7_IRQHandler
**
** Descriptions:                Use one GPIO pin as interrupt source
**
** parameters:                        None
**
** Returned value:                None
**
*****************************************************************************/
#if defined CFG_GPIO_ENABLE_PINIRQ7
#if defined CFG_MCU_FAMILY_LPC11UXX
void FLEX_INT7_IRQHandler(void)
#elif defined CFG_MCU_FAMILY_LPC13UXX
void PIN_INT7_IRQHandler(void)
#else
  #error "gpio.c: No MCU defined"
#endif
{
  if ( LPC_GPIO_PIN_INT->IST & (0x1<<7) )
  {
        if ( LPC_GPIO_PIN_INT->ISEL & (0x1<<7) )
        {
          // Level
        }
        else
        {
          if ( ( LPC_GPIO_PIN_INT->RISE & (0x1<<7) ) && ( LPC_GPIO_PIN_INT->IENR & (0x1<<7) ) )
          {
                // Rising Edge
                LPC_GPIO_PIN_INT->RISE = 0x1<<7;
          }
          if ( ( LPC_GPIO_PIN_INT->FALL & (0x1<<7) ) && ( LPC_GPIO_PIN_INT->IENF & (0x1<<7) ) )
          {
                // Falling Edge
                LPC_GPIO_PIN_INT->FALL = 0x1<<7;
          }
          LPC_GPIO_PIN_INT->IST = 0x1<<7;
        }
  }
  return;
}
#endif

/*****************************************************************************
** Function name:                GINT0_IRQHandler
**
** Descriptions:                Use one GPIO pin as interrupt source
**
** parameters:                        None
**
** Returned value:                None
**
*****************************************************************************/
#ifdef CFG_GPIO_ENABLE_IRQ0
void GINT0_IRQHandler(void)
{
  if ( LPC_GPIO_GROUP_INT0->CTRL & (0x1<<0) )
  {
        if ( LPC_GPIO_GROUP_INT0->CTRL & (0x1<<2) )
        {
          // Level Interrupt
        }
        else
        {
          // Edge Interrupt
        }
        LPC_GPIO_GROUP_INT0->CTRL |= (0x1<<0);
  }
  return;
}
#endif

/*****************************************************************************
** Function name:                GINT1_IRQHandler
**
** Descriptions:                Use one GPIO pin as interrupt source
**
** parameters:                        None
**
** Returned value:                None
**
*****************************************************************************/
#ifdef CFG_GPIO_ENABLE_IRQ1
void GINT1_IRQHandler(void)
{
  if ( LPC_GPIO_GROUP_INT1->CTRL & (0x1<<0) )
  {
        if ( LPC_GPIO_GROUP_INT1->CTRL & (0x1<<2) )
        {
          // Level Interrupt
        }
        else
        {
          // Edge Interrupt
        }
        LPC_GPIO_GROUP_INT1->CTRL |= (0x1<<0);
  }
  return;
}
#endif

/*****************************************************************************
** Function name:                GPIOInit
**
** Descriptions:                Initialize GPIO, install the
**                                                GPIO interrupt handler
**
** parameters:                        None
**
** Returned value:                true or false, return false if the VIC table
**                                                is full and GPIO interrupt handler can be
**                                                installed.
**
*****************************************************************************/
void GPIOInit( void )
{
  /* Enable AHB clock to the GPIO domain. */
  LPC_SYSCON->SYSAHBCLKCTRL |= (1<<6);

  /* Enable AHB clock to the PinInt, GroupedInt domain. */
  LPC_SYSCON->SYSAHBCLKCTRL |= ((1<<19) | (1<<23) | (1<<24));

  return;
}

/*****************************************************************************
** Function name:                GPIOSetPinInterrupt
**
** Descriptions:                Set interrupt sense, event, etc.
**                                                sense: edge or level, 0 is edge, 1 is level
**                                                event/polarity: 0 is active low/falling, 1 is high/rising.
**
** parameters:                        channel #, port #, bit position, sense, event(polarity)
**
** Returned value:                None
**
*****************************************************************************/
void GPIOSetPinInterrupt( uint32_t channelNum, uint32_t portNum, uint32_t bitPosi,
                uint32_t sense, uint32_t event )
{
  switch ( channelNum )
  {
        case CHANNEL0:
          if ( portNum )
          {
            #if defined CFG_MCU_FAMILY_LPC11UXX
                LPC_SYSCON->PINTSEL[0] = bitPosi + 24;
            #elif defined CFG_MCU_FAMILY_LPC13UXX
                LPC_SYSCON->PINSEL[0] = bitPosi + 24;
            #endif
          }
          else
          {
            #if defined CFG_MCU_FAMILY_LPC11UXX
                LPC_SYSCON->PINTSEL[0] = bitPosi;
            #elif defined CFG_MCU_FAMILY_LPC13UXX
                LPC_SYSCON->PINSEL[0] = bitPosi;
            #endif
          }
          #if defined CFG_MCU_FAMILY_LPC11UXX
            NVIC_EnableIRQ(FLEX_INT0_IRQn);
          #elif defined CFG_MCU_FAMILY_LPC13UXX
            NVIC_EnableIRQ(PIN_INT0_IRQn);
          #endif
        break;
        case CHANNEL1:
          if ( portNum )
          {
            #if defined CFG_MCU_FAMILY_LPC11UXX
                LPC_SYSCON->PINTSEL[1] = bitPosi + 24;
            #elif defined CFG_MCU_FAMILY_LPC13UXX
                LPC_SYSCON->PINSEL[1] = bitPosi + 24;
            #endif
          }
          else
          {
            #if defined CFG_MCU_FAMILY_LPC11UXX
                LPC_SYSCON->PINTSEL[1] = bitPosi;
            #elif defined CFG_MCU_FAMILY_LPC13UXX
                LPC_SYSCON->PINSEL[1] = bitPosi;
            #endif
          }
          #if defined CFG_MCU_FAMILY_LPC11UXX
            NVIC_EnableIRQ(FLEX_INT1_IRQn);
          #elif defined CFG_MCU_FAMILY_LPC13UXX
            NVIC_EnableIRQ(PIN_INT1_IRQn);
          #endif
        break;
        case CHANNEL2:
          if ( portNum )
          {
            #if defined CFG_MCU_FAMILY_LPC11UXX
                LPC_SYSCON->PINTSEL[2] = bitPosi + 24;
            #elif defined CFG_MCU_FAMILY_LPC13UXX
                LPC_SYSCON->PINSEL[2] = bitPosi + 24;
            #endif
          }
          else
          {
            #if defined CFG_MCU_FAMILY_LPC11UXX
                LPC_SYSCON->PINTSEL[2] = bitPosi;
            #elif defined CFG_MCU_FAMILY_LPC13UXX
                LPC_SYSCON->PINSEL[2] = bitPosi;
            #endif
          }
          #if defined CFG_MCU_FAMILY_LPC11UXX
            NVIC_EnableIRQ(FLEX_INT2_IRQn);
          #elif defined CFG_MCU_FAMILY_LPC13UXX
            NVIC_EnableIRQ(PIN_INT2_IRQn);
          #endif
        break;
        case CHANNEL3:
          if ( portNum )
          {
            #if defined CFG_MCU_FAMILY_LPC11UXX
                LPC_SYSCON->PINTSEL[3] = bitPosi + 24;
            #elif defined CFG_MCU_FAMILY_LPC13UXX
                LPC_SYSCON->PINSEL[3] = bitPosi + 24;
            #endif
          }
          else
          {
            #if defined CFG_MCU_FAMILY_LPC11UXX
                LPC_SYSCON->PINTSEL[3] = bitPosi;
            #elif defined CFG_MCU_FAMILY_LPC13UXX
                LPC_SYSCON->PINSEL[3] = bitPosi;
            #endif
          }
          #if defined CFG_MCU_FAMILY_LPC11UXX
            NVIC_EnableIRQ(FLEX_INT3_IRQn);
          #elif defined CFG_MCU_FAMILY_LPC13UXX
            NVIC_EnableIRQ(PIN_INT3_IRQn);
          #endif
        break;
        case CHANNEL4:
          if ( portNum )
          {
            #if defined CFG_MCU_FAMILY_LPC11UXX
                LPC_SYSCON->PINTSEL[4] = bitPosi + 24;
            #elif defined CFG_MCU_FAMILY_LPC13UXX
                LPC_SYSCON->PINSEL[4] = bitPosi + 24;
            #endif
          }
          else
          {
            #if defined CFG_MCU_FAMILY_LPC11UXX
                LPC_SYSCON->PINTSEL[4] = bitPosi;
            #elif defined CFG_MCU_FAMILY_LPC13UXX
                LPC_SYSCON->PINSEL[4] = bitPosi;
            #endif
          }
          #if defined CFG_MCU_FAMILY_LPC11UXX
            NVIC_EnableIRQ(FLEX_INT4_IRQn);
          #elif defined CFG_MCU_FAMILY_LPC13UXX
            NVIC_EnableIRQ(PIN_INT4_IRQn);
          #endif
        break;
        case CHANNEL5:
          if ( portNum )
          {
            #if defined CFG_MCU_FAMILY_LPC11UXX
                LPC_SYSCON->PINTSEL[5] = bitPosi + 24;
            #elif defined CFG_MCU_FAMILY_LPC13UXX
                LPC_SYSCON->PINSEL[5] = bitPosi + 24;
            #endif
          }
          else
          {
            #if defined CFG_MCU_FAMILY_LPC11UXX
                LPC_SYSCON->PINTSEL[5] = bitPosi;
            #elif defined CFG_MCU_FAMILY_LPC13UXX
                LPC_SYSCON->PINSEL[5] = bitPosi;
            #endif
          }
          #if defined CFG_MCU_FAMILY_LPC11UXX
            NVIC_EnableIRQ(FLEX_INT5_IRQn);
          #elif defined CFG_MCU_FAMILY_LPC13UXX
            NVIC_EnableIRQ(PIN_INT5_IRQn);
          #endif
        break;
        case CHANNEL6:
          if ( portNum )
          {
            #if defined CFG_MCU_FAMILY_LPC11UXX
                LPC_SYSCON->PINTSEL[6] = bitPosi + 24;
            #elif defined CFG_MCU_FAMILY_LPC13UXX
                LPC_SYSCON->PINSEL[6] = bitPosi + 24;
            #endif
          }
          else
          {
            #if defined CFG_MCU_FAMILY_LPC11UXX
                LPC_SYSCON->PINTSEL[6] = bitPosi;
            #elif defined CFG_MCU_FAMILY_LPC13UXX
                LPC_SYSCON->PINSEL[6] = bitPosi;
            #endif
          }
          #if defined CFG_MCU_FAMILY_LPC11UXX
            NVIC_EnableIRQ(FLEX_INT6_IRQn);
          #elif defined CFG_MCU_FAMILY_LPC13UXX
            NVIC_EnableIRQ(PIN_INT6_IRQn);
          #endif
        break;
        case CHANNEL7:
          if ( portNum )
          {
            #if defined CFG_MCU_FAMILY_LPC11UXX
                LPC_SYSCON->PINTSEL[7] = bitPosi + 24;
            #elif defined CFG_MCU_FAMILY_LPC13UXX
                LPC_SYSCON->PINSEL[7] = bitPosi + 24;
            #endif
          }
          else
          {
            #if defined CFG_MCU_FAMILY_LPC11UXX
                LPC_SYSCON->PINTSEL[7] = bitPosi;
            #elif defined CFG_MCU_FAMILY_LPC13UXX
                LPC_SYSCON->PINSEL[7] = bitPosi;
            #endif
          }
          #if defined CFG_MCU_FAMILY_LPC11UXX
            NVIC_EnableIRQ(FLEX_INT7_IRQn);
          #elif defined CFG_MCU_FAMILY_LPC13UXX
            NVIC_EnableIRQ(PIN_INT7_IRQn);
          #endif
        break;
        default:
          break;
  }
  if ( sense == 0 )
  {
        LPC_GPIO_PIN_INT->ISEL &= ~(0x1<<channelNum);       /* Edge trigger */
        if ( event == 0 )
        {
          LPC_GPIO_PIN_INT->IENF |= (0x1<<channelNum);      /* Falling edge */
        }
        else
        {
          LPC_GPIO_PIN_INT->IENR |= (0x1<<channelNum);      /* Rising edge */
        }
  }
  else
  {
        LPC_GPIO_PIN_INT->ISEL |= (0x1<<channelNum);        /* Level trigger. */
        LPC_GPIO_PIN_INT->IENR |= (0x1<<channelNum);        /* Level enable */
        if ( event == 0 )
        {
          LPC_GPIO_PIN_INT->IENF &= ~(0x1<<channelNum);      /* active-low */
        }
        else
        {
          LPC_GPIO_PIN_INT->IENF |= (0x1<<channelNum);      /* active-high */
        }
  }
  return;
}

/*****************************************************************************
** Function name:                GPIOPinIntEnable
**
** Descriptions:                Enable Interrupt
**
** parameters:                        channel num, event(0 is falling edge, 1 is rising edge)
** Returned value:                None
**
*****************************************************************************/
void GPIOPinIntEnable( uint32_t channelNum, uint32_t event )
{
  if ( !( LPC_GPIO_PIN_INT->ISEL & (0x1<<channelNum) ) )
  {
        if ( event == 0 )
        {
          LPC_GPIO_PIN_INT->SIENF |= (0x1<<channelNum);      /* Falling edge */
        }
        else
        {
          LPC_GPIO_PIN_INT->SIENR |= (0x1<<channelNum);      /* Rising edge */
        }
  }
  else
  {
        LPC_GPIO_PIN_INT->SIENR |= (0x1<<channelNum);        /* Level */
  }
  return;
}

/*****************************************************************************
** Function name:                GPIOPinIntDisable
**
** Descriptions:                Disable Interrupt
**
** parameters:                        channel num, event(0 is falling edge, 1 is rising edge)
**
** Returned value:                None
**
*****************************************************************************/
void GPIOPinIntDisable( uint32_t channelNum, uint32_t event )
{
  if ( !( LPC_GPIO_PIN_INT->ISEL & (0x1<<channelNum) ) )
  {
        if ( event == 0 )
        {
          LPC_GPIO_PIN_INT->CIENF |= (0x1<<channelNum);      /* Falling edge */
        }
        else
        {
          LPC_GPIO_PIN_INT->CIENR |= (0x1<<channelNum);      /* Rising edge */
        }
  }
  else
  {
        LPC_GPIO_PIN_INT->CIENR |= (0x1<<channelNum);        /* Level */
  }
  return;
}

/*****************************************************************************
** Function name:                GPIOPinIntStatus
**
** Descriptions:                Get Interrupt status
**
** parameters:                        channel num
**
** Returned value:                None
**
*****************************************************************************/
uint32_t GPIOPinIntStatus( uint32_t channelNum )
{
  if ( LPC_GPIO_PIN_INT->IST & (0x1<<channelNum) )
  {
        return( 1 );
  }
  else
  {
        return( 0 );
  }
}

/*****************************************************************************
** Function name:                GPIOPinIntClear
**
** Descriptions:                Clear Interrupt
**
** parameters:                        channel num
**
** Returned value:                None
**
*****************************************************************************/
void GPIOPinIntClear( uint32_t channelNum )
{
  if ( !( LPC_GPIO_PIN_INT->ISEL & (0x1<<channelNum) ) )
  {
        LPC_GPIO_PIN_INT->IST = (1<<channelNum);
  }
  return;
}

/*****************************************************************************
** Function name:                GPIOSetGroupedInterrupt
**
** Descriptions:                Set interrupt logic, sense, eventPattern, etc.
**                                                logic: AND or OR, 0 is OR, 1 is AND
**                                                sensePattern: edge or level, 0 is edge, 1 is level
**                                                event/polarity: 0 is active low/falling, 1 is high/rising.
**
** parameters:                        group #, bit pattern, logic, sense, event(polarity) pattern
**
** Returned value:                None
**
*****************************************************************************/
void GPIOSetGroupedInterrupt( uint32_t groupNum, uint32_t *bitPattern, uint32_t logic,
                uint32_t sense, uint32_t *eventPattern )
{
  switch ( groupNum )
  {
        case GROUP0:
          if ( sense == 0 )
          {
                LPC_GPIO_GROUP_INT0->CTRL &= ~(0x1<<2);        /* Edge trigger */
          }
          else
          {
                LPC_GPIO_GROUP_INT0->CTRL |= (0x1<<2);        /* Level trigger. */
          }
          if ( logic == 0 )
          {
                LPC_GPIO_GROUP_INT0->CTRL &= ~(0x1<<1);        /* OR */
          }
          else
          {
                LPC_GPIO_GROUP_INT0->CTRL |= (0x1<<1);        /* AND */
          }
          LPC_GPIO_GROUP_INT0->PORT_POL[0] = *((uint32_t *)(eventPattern + 0));
          LPC_GPIO_GROUP_INT0->PORT_POL[1] = *((uint32_t *)(eventPattern + 1));
          LPC_GPIO_GROUP_INT0->PORT_ENA[0] = *((uint32_t *)(bitPattern + 0));
          LPC_GPIO_GROUP_INT0->PORT_ENA[1] = *((uint32_t *)(bitPattern + 1));
      /* as soon as enabled, an edge may be generated       */
          /* clear interrupt flag and NVIC pending interrupt to */
          /* workaround the potential edge generated as enabled */
          LPC_GPIO_GROUP_INT0->CTRL |= (1<<0);
          NVIC_ClearPendingIRQ(GINT0_IRQn);
          NVIC_EnableIRQ(GINT0_IRQn);
        break;
        case GROUP1:
          if ( sense == 0 )
          {
                LPC_GPIO_GROUP_INT1->CTRL &= ~(0x1<<2);        /* Edge trigger */
          }
          else
          {
                LPC_GPIO_GROUP_INT1->CTRL |= (0x1<<2);        /* Level trigger. */
          }
          if ( logic == 0 )
          {
                LPC_GPIO_GROUP_INT1->CTRL &= ~(0x1<<1);        /* OR */
          }
          else
          {
                LPC_GPIO_GROUP_INT1->CTRL |= (0x1<<1);        /* AND */
          }
          LPC_GPIO_GROUP_INT1->PORT_POL[0] = *((uint32_t *)(eventPattern + 0));
          LPC_GPIO_GROUP_INT1->PORT_POL[1] = *((uint32_t *)(eventPattern + 1));
          LPC_GPIO_GROUP_INT1->PORT_ENA[0] = *((uint32_t *)(bitPattern + 0));
          LPC_GPIO_GROUP_INT1->PORT_ENA[1] = *((uint32_t *)(bitPattern + 1));
      /* as soon as enabled, an edge may be generated       */
          /* clear interrupt flag and NVIC pending interrupt to */
          /* workaround the potential edge generated as enabled */
          LPC_GPIO_GROUP_INT1->CTRL |= (1<<0);
          NVIC_ClearPendingIRQ(GINT1_IRQn);
          NVIC_EnableIRQ(GINT1_IRQn);
        break;
        default:
          break;
  }

  return;
}

/*****************************************************************************
** Function name:                GPIOGetPinValue
**
** Descriptions:                Read Current state of port pin, PIN register value
**
** parameters:                        port num, bit position
** Returned value:                None
**
*****************************************************************************/
uint32_t GPIOGetPinValue( uint32_t portNum, uint32_t bitPosi )
{
  uint32_t regVal = 0;

  if( bitPosi < 0x20 )
  {
        if ( LPC_GPIO->PIN[portNum] & (0x1<<bitPosi) )
        {
          regVal = 1;
        }
  }
  else if( bitPosi == 0xFF )
  {
        regVal = LPC_GPIO->PIN[portNum];
  }
  return ( regVal );
}

/*****************************************************************************
** Function name:                GPIOSetBitValue
**
** Descriptions:                Set/clear a bit in a specific position
**
** parameters:                        port num, bit position, bit value
**
** Returned value:                None
**
*****************************************************************************/
void GPIOSetBitValue( uint32_t portNum, uint32_t bitPosi, uint32_t bitVal )
{
  if ( bitVal )
  {
        LPC_GPIO->SET[portNum] = 1<<bitPosi;
  }
  else
  {
        LPC_GPIO->CLR[portNum] = 1<<bitPosi;
  }
  return;
}

/*****************************************************************************
** Function name:                GPIOSetDir
**
** Descriptions:                Set the direction in GPIO port
**
** parameters:                        portNum, bit position, direction (1 out, 0 input)
**
** Returned value:                None
**
*****************************************************************************/
void GPIOSetDir( uint32_t portNum, uint32_t bitPosi, uint32_t dir )
{
  if( dir )
  {
        LPC_GPIO->DIR[portNum] |= (1<<bitPosi);
  }
  else
  {
        LPC_GPIO->DIR[portNum] &= ~(1<<bitPosi);
  }
  return;
}

/******************************************************************************
**                            End Of File
******************************************************************************/
