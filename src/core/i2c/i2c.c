/*****************************************************************************
 *   i2c.c:  I2C C file for NXP LPC11xx/13xx Family Microprocessors
 *
 *   Copyright(C) 2008, NXP Semiconductor
 *   Parts of this code are (C) 2010, MyVoice CAD/CAM Services
 *   All rights reserved.
 *
 *   History
 *   2009.12.07  ver 1.00    Preliminary version, first Release
 *   2010.07.19  ver 1.10    Rob Jansen - MyVoice CAD/CAM Services:
 *                           Major cleaning and a rewrite of some functions
 *                           - adding ACK/NACK handling to the state machine
 *                           - adding a return result to the I2CEngine()
 *
*****************************************************************************/
#include "projectconfig.h"

#ifdef CFG_ENABLE_I2C

#include "i2c.h"

volatile uint32_t I2CMasterState = I2CSTATE_IDLE;
volatile uint32_t I2CSlaveState = I2CSTATE_IDLE;

volatile uint8_t I2CMasterBuffer[I2C_BUFSIZE];    // Master Mode
volatile uint8_t I2CSlaveBuffer[I2C_BUFSIZE];     // Master Mode

volatile uint32_t I2CReadLength;
volatile uint32_t I2CWriteLength;

volatile uint32_t I2CRdIndex;
volatile uint32_t I2CWrIndex;

volatile uint32_t _I2cMode;                       // I2CMASTER or I2CSLAVE (only master is used at present)

/*****************************************************************************
** Function name:                I2C_IRQHandler
**
** Descriptions:                I2C interrupt handler, deal with master mode only.
**
** parameters:                        None
** Returned value:                None
**
*****************************************************************************/
void I2C_IRQHandler(void)
{
        uint8_t StatValue;

        /* this handler deals with master read and master write only */
        StatValue = LPC_I2C->STAT;
        switch ( StatValue )
        {
        case 0x08:
                /*
                 * A START condition has been transmitted.
                 * We now send the slave address and initialize
                 * the write buffer
                 * (we always start with a write after START+SLA)
                 */
                I2CWrIndex = 0;
                LPC_I2C->DAT = I2CMasterBuffer[I2CWrIndex++];
                LPC_I2C->CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);
                I2CMasterState = I2CSTATE_PENDING;
                break;

        case 0x10:
                /*
                 * A repeated START condition has been transmitted.
                 * Now a second, read, transaction follows so we
                 * initialize the read buffer.
                 */
                I2CRdIndex = 0;
                /* Send SLA with R bit set, */
                LPC_I2C->DAT = I2CMasterBuffer[I2CWrIndex++];
                LPC_I2C->CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);
        break;

        case 0x18:
                /*
                 * SLA+W has been transmitted; ACK has been received.
                 * We now start writing bytes.
                 */
                LPC_I2C->DAT = I2CMasterBuffer[I2CWrIndex++];
                LPC_I2C->CONCLR = I2CONCLR_SIC;
                break;

        case 0x20:
                /*
                 * SLA+W has been transmitted; NOT ACK has been received.
                 * Send a stop condition to terminate the transaction
                 * and signal I2CEngine the transaction is aborted.
                 */
                LPC_I2C->CONSET = I2CONSET_STO;
                LPC_I2C->CONCLR = I2CONCLR_SIC;
                I2CMasterState = I2CSTATE_SLA_NACK;
                break;

        case 0x28:
                /*
                 * Data in I2DAT has been transmitted; ACK has been received.
                 * Continue sending more bytes as long as there are bytes to send
                 * and after this check if a read transaction should follow.
                 */
                if ( I2CWrIndex < I2CWriteLength )
                {
                        /* Keep writing as long as bytes avail */
                        LPC_I2C->DAT = I2CMasterBuffer[I2CWrIndex++];
                }
                else
                {
                        if ( I2CReadLength != 0 )
                        {
                                /* Send a Repeated START to initialize a read transaction */
                                /* (handled in state 0x10)                                */
                                LPC_I2C->CONSET = I2CONSET_STA;        /* Set Repeated-start flag */
                        }
                        else
                        {
                                I2CMasterState = I2CSTATE_ACK;
                                LPC_I2C->CONSET = I2CONSET_STO;      /* Set Stop flag */
                        }
                }
                LPC_I2C->CONCLR = I2CONCLR_SIC;
                break;

        case 0x30:
                /*
                 * Data byte in I2DAT has been transmitted; NOT ACK has been received
                 * Send a STOP condition to terminate the transaction and inform the
                 * I2CEngine that the transaction failed.
                 */
                LPC_I2C->CONSET = I2CONSET_STO;
                LPC_I2C->CONCLR = I2CONCLR_SIC;
                I2CMasterState = I2CSTATE_NACK;
                break;

        case 0x38:
                /*
                 * Arbitration loss in SLA+R/W or Data bytes.
                 * This is a fatal condition, the transaction did not complete due
                 * to external reasons (e.g. hardware system failure).
                 * Inform the I2CEngine of this and cancel the transaction
                 * (this is automatically done by the I2C hardware)
                 */
                I2CMasterState = I2CSTATE_ARB_LOSS;
                LPC_I2C->CONCLR = I2CONCLR_SIC;
                break;

        case 0x40:
                /*
                 * SLA+R has been transmitted; ACK has been received.
                 * Initialize a read.
                 * Since a NOT ACK is sent after reading the last byte,
                 * we need to prepare a NOT ACK in case we only read 1 byte.
                 */
                if ( I2CReadLength == 1 )
                {
                        /* last (and only) byte: send a NACK after data is received */
                        LPC_I2C->CONCLR = I2CONCLR_AAC;
                }
                else
                {
                        /* more bytes to follow: send an ACK after data is received */
                        LPC_I2C->CONSET = I2CONSET_AA;
                }
                LPC_I2C->CONCLR = I2CONCLR_SIC;
                break;

        case 0x48:
                /*
                 * SLA+R has been transmitted; NOT ACK has been received.
                 * Send a stop condition to terminate the transaction
                 * and signal I2CEngine the transaction is aborted.
                 */
                LPC_I2C->CONSET = I2CONSET_STO;
                LPC_I2C->CONCLR = I2CONCLR_SIC;
                I2CMasterState = I2CSTATE_SLA_NACK;
                break;

        case 0x50:
                /*
                 * Data byte has been received; ACK has been returned.
                 * Read the byte and check for more bytes to read.
                 * Send a NOT ACK after the last byte is received
                 */
                I2CSlaveBuffer[I2CRdIndex++] = LPC_I2C->DAT;
                if ( I2CRdIndex < (I2CReadLength-1) )
                {
                        /* lmore bytes to follow: send an ACK after data is received */
                        LPC_I2C->CONSET = I2CONSET_AA;
                }
                else
                {
                        /* last byte: send a NACK after data is received */
                        LPC_I2C->CONCLR = I2CONCLR_AAC;
                }
                LPC_I2C->CONCLR = I2CONCLR_SIC;
                break;

        case 0x58:
                /*
                 * Data byte has been received; NOT ACK has been returned.
                 * This is the last byte to read.
                 * Generate a STOP condition and flag the I2CEngine that the
                 * transaction is finished.
                 */
                I2CSlaveBuffer[I2CRdIndex++] = LPC_I2C->DAT;
                I2CMasterState = I2CSTATE_ACK;
                LPC_I2C->CONSET = I2CONSET_STO;        /* Set Stop flag */
                LPC_I2C->CONCLR = I2CONCLR_SIC;        /* Clear SI flag */
                break;

        default:
                LPC_I2C->CONCLR = I2CONCLR_SIC;
                break;
  }
  return;
}

/*****************************************************************************
** Function name:        I2CStart
**
** Descriptions:        Create I2C start condition, a timeout
**                        value is set if the I2C never gets started,
**                        and timed out. It's a fatal error.
**
** parameters:                None
** Returned value:        true or false, return false if timed out
**
*****************************************************************************/
static uint32_t I2CStart( void )
{
  uint32_t timeout = 0;

  /*--- Issue a start condition ---*/
  LPC_I2C->CONSET = I2CONSET_STA;      /* Set Start flag */

  while((I2CMasterState != I2CSTATE_PENDING) && (timeout < MAX_TIMEOUT))
  {
    timeout++;
  }

  return (timeout < MAX_TIMEOUT);
}

/*****************************************************************************
** Function name:        I2CStop
**
** Descriptions:        Set the I2C stop condition
**
** parameters:                None
** Returned value:        true or never return
**
*****************************************************************************/
static uint32_t I2CStop( void )
{
  uint32_t timeout = 0;

  LPC_I2C->CONSET = I2CONSET_STO;  /* Set Stop flag */
  LPC_I2C->CONCLR = I2CONCLR_SIC;  /* Clear SI flag */

  /*--- Wait for STOP detected ---*/
  while((LPC_I2C->CONSET & I2CONSET_STO) && (timeout < MAX_TIMEOUT))
  {
    timeout++;
  }
  return (timeout >= MAX_TIMEOUT);
}

/*****************************************************************************
** Function name:        I2CInit
**
** Descriptions:        Initialize I2C controller
**
** parameters:                I2c mode is either MASTER or SLAVE
** Returned value:        true or false, return false if the I2C
**                        interrupt handler was not installed correctly
**
*****************************************************************************/
uint32_t i2cInit( uint32_t I2cMode )
{
  _I2cMode = I2cMode;

  LPC_SYSCON->PRESETCTRL |= (0x1<<1);

  /* Enable AHB clock to the I2C domain. */
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 5);

  /* Set P0.4 to SCL and I2C mode (open drain) */
  LPC_IOCON->PIO0_4 &= ~0x07 | 0x300; /* Add I2C mode mask */
  LPC_IOCON->PIO0_4 |= 0x01;

  /* Set P0.5 to SDA and I2C mode (open drain) */
  LPC_IOCON->PIO0_5 &= ~0x07 | 0x300; /* Add I2C mode mask */
  LPC_IOCON->PIO0_5 |= 0x01;

  // Clear flags
  LPC_I2C->CONCLR = I2CONCLR_AAC |
                  I2CONCLR_SIC |
                  I2CONCLR_STAC |
                  I2CONCLR_I2ENC;

#if I2C_FAST_MODE_PLUS
  LPC_IOCON->PIO0_4 |= (0x200); /* Fast mode plus */
  LPC_IOCON->PIO0_5 |= (0x200); /* Fast mode plus */
  LPC_I2C->SCLL   = I2C_SCLL_HS_SCLL;
  LPC_I2C->SCLH   = I2C_SCLH_HS_SCLH;
#else
  LPC_I2C->SCLL   = I2SCLL_SCLL;
  LPC_I2C->SCLH   = I2SCLH_SCLH;
#endif

  if ( _I2cMode == I2CSLAVE )
  {
    LPC_I2C->ADR0 = SLAVE_ADDR;
    I2CSlaveState = I2C_IDLE;
  }

  /* Enable the I2C Interrupt */
  NVIC_EnableIRQ(I2C_IRQn);
  if ( _I2cMode == I2CMASTER )
  {
    LPC_I2C->CONSET = I2CONSET_I2EN;
  }

  return( TRUE );
}

/*****************************************************************************
** Function name:        I2CEngine
**
** Descriptions:        The routine to complete a I2C transaction
**                        from start to stop. All the intermitten
**                        steps are handled in the interrupt handler.
**                        Before this routine is called, the read
**                        length, write length and I2C master buffer
**                        need to be filled.
**
** parameters:                None
** Returned value:        Any of the I2CSTATE_... values. See i2c.h
**
*****************************************************************************/
uint32_t i2cEngine( void )
{
  I2CMasterState = I2CSTATE_IDLE;
  I2CRdIndex = 0;
  I2CWrIndex = 0;
  if ( I2CStart() != TRUE )
  {
        // I2C timed out
    I2CStop();
    return ( I2CSTATE_TIMEOUT );
  }

  /* wait until the state is a terminal state */
  while (I2CMasterState < 0x100);

  return ( I2CMasterState );
}

/*****************************************************************************
** Function name:        i2cCheckAddress
**
** Descriptions:        The routine checks if a device is present at
**                        the specified address.
**
** parameters:                8-bit address byte including the read bit
** Returned value:        True if a device ACKed, otherwise False
**
*****************************************************************************/
bool i2cCheckAddress( uint8_t addr )
{
  uint32_t i2cState;

  // Send the addr byte and check for ACK
  I2CWriteLength = 1;
  I2CReadLength = 0;
  I2CMasterBuffer[0] = addr;
  i2cState = i2cEngine();

  // Check if we got an ACK
  if ((i2cState == I2CSTATE_NACK) || (i2cState == I2CSTATE_SLA_NACK))
  {
    // I2C slave didn't acknowledge the master transfer
    // The device probably isn't connected
    return false;
  }
  else if (i2cState == I2CSTATE_TIMEOUT)
  {
    // I2C timed out waiting for start/stop
        return false;
  }
  else
  {
    // I2C device found
    return true;
  }
}

/******************************************************************************
**                            End Of File
******************************************************************************/

#endif
