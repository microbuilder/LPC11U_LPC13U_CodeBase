/**************************************************************************/
/*!
    @file     gpio.c

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
#include "gpio.h"

/**************************************************************************/
/*!
    @brief Initialises the GPIO block
*/
/**************************************************************************/
void GPIOInit( void )
{
  /* Enable AHB clock to the GPIO domain. */
  LPC_SYSCON->SYSAHBCLKCTRL |= (1<<6);

  /* Enable AHB clock to the PinInt, GroupedInt domain. */
  LPC_SYSCON->SYSAHBCLKCTRL |= ((1<<19) | (1<<23) | (1<<24));

  return;
}

/**************************************************************************/
/*!
    @brief Sets up a GPIO interrupt, including sense and polarity (event)

    @param[in]  channelNum
                Interrupt channel number (0..7)
    @param[in]  portNum
                Port number for the pin to setup as the interrupt source
    @param[in]  bitPosi
                Bit position of the pin to setup as the interrupt source
    @param[in]  sense
                Edge or level sensitive (0 = edge, 1 = level)
    @param[in]  event
                Polarity for the interrupt (0 = active low/falling-edge,
                1 = active high/rising edge)
*/
/**************************************************************************/
void GPIOSetPinInterrupt( uint32_t channelNum, uint32_t portNum,
    uint32_t bitPosi, uint32_t sense, uint32_t event )
{
  /* Pair the interrupt channel to the specified pin */
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

  /* Configure for edge (0) or level (1) sensitivity */
  if ( sense == 0 )
  {
    /* Interrupt is edge sensitive */
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
    /* Interrupt is level sensitive */
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

/**************************************************************************/
/*!
    @brief Enables the specified interrupt

    @param[in]  channelNum
                Interrupt channel number (0..7)
    @param[in]  event
                Polarity for the interrupt (0 = active low/falling-edge,
                1 = active high/rising edge)
*/
/**************************************************************************/
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

/**************************************************************************/
/*!
    @brief Disables the specified interrupt

    @param[in]  channelNum
                Interrupt channel number (0..7)
    @param[in]  event
                Polarity for the interrupt (0 = active low/falling-edge,
                1 = active high/rising edge)
*/
/**************************************************************************/
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

/**************************************************************************/
/*!
    @brief Gets the interrupt status

    @param[in]  channelNum
                Interrupt channel number (0..7)
*/
/**************************************************************************/
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

/**************************************************************************/
/*!
    @brief Clears the specified interrupt

    @param[in]  channelNum
                Interrupt channel number (0..7)
*/
/**************************************************************************/
void GPIOPinIntClear( uint32_t channelNum )
{
  if ( !( LPC_GPIO_PIN_INT->ISEL & (0x1<<channelNum) ) )
  {
    LPC_GPIO_PIN_INT->IST = (1<<channelNum);
  }
  return;
}

/**************************************************************************/
/*!
    @brief  Sets up the interrupt logic (sense, eventPattern, etc.) for
            a grouped interrupt

    @param[in]  groupNum
                Grouped interrupt number (0..1)
    @param[in]  bitPattern
                Bit pattern to match
    @param[in]  logic
                AND or OR logic (0 = OR, 1 = AND)
    @param[in]  sense
                Edge or level sensitive (0 = edge, 1 = level)
    @param[in]  eventPattern
                Polarity pattern for the interrupt (0 = low/falling-edge,
                1 = high/rising edge)
*/
/**************************************************************************/
void GPIOSetGroupedInterrupt( uint32_t groupNum, uint32_t *bitPattern,
      uint32_t logic, uint32_t sense, uint32_t *eventPattern )
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
      /* As soon as enabled, an edge may be generated       */
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
      /* As soon as enabled, an edge may be generated       */
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

/**************************************************************************/
/*!
    @brief Reads the current state of the specified pin

    @param[in]  portNum
                Port number for the pin
    @param[in]  bitPosi
                Bit position of the pin
*/
/**************************************************************************/
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

/**************************************************************************/
/*!
    @brief Sets the state of the specified pin

    @param[in]  portNum
                Port number for the pin
    @param[in]  bitPosi
                Bit position of the pin
    @param[in]  bitVal
                1 to set the pin high, 0 to set it low
*/
/**************************************************************************/
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

/**************************************************************************/
/*!
    @brief Sets the pin direction (input or output)

    @param[in]  portNum
                Port number for the pin
    @param[in]  bitPosi
                Bit position of the pin
    @param[in]  dir
                Pin direction (1 = output, 0 = input)
*/
/**************************************************************************/
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
