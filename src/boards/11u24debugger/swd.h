/****************************************************************************
 *   $Id:: swd.c 6085 2011-01-05 23:55:06Z usb00423                   $
 *   Project: LPC11U00 SWD to USB bridge
 *
 *   Description:
 *     Header file for basic SWD bitbang support
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
****************************************************************************/

//#define SWD_DEBUG

/* GPIO pins used for bit-banging SWD */
#define SWD_PORT          (1)
#define SWD_SWCLK_BIT     (27)
#define SWD_SWDIO_BIT     (26)

#define SWD_SWCLK_BIT_MSK (1<<SWD_SWCLK_BIT)
#define SWD_SWDIO_BIT_MSK (1<<SWD_SWDIO_BIT)

/* Bit delay */
#define SWD_BIT_DELAY     (1)

/* SWD status responses. SWD_ACK is good. */
#define SWD_ACK           (0b001)
#define SWD_WAIT          (0b010)
#define SWD_FAULT         (0b100)
#define SWD_PARITY        (0b1000)

void swdEnable(void);
void swdFlush(void);
int  swdWrite(char APnDP, int A, unsigned long data);
int  swdRead(char APnDP, int A, volatile unsigned long *data);

