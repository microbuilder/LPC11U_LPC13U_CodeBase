/****************************************************************************
 *   $Id:: swdbridge.h 6085 2011-01-05 23:55:06Z usb00423                   $
 *   Project: LPC11U00 SWD to USB bridge
 *
 *   Description:
 *     Header file for SWD to USB bridge firmware
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
#ifndef __SWDBRIDGE_H__
#define __SWDBRIDGE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "core/usb/usbd.h"

#define SWDBRIDGE_OR_OPCODE_NOTHING         (0)
#define SWDBRIDGE_OR_OPCODE_CONNECT         (1)
#define SWDBRIDGE_OR_OPCODE_READDP          (2)
#define SWDBRIDGE_OR_OPCODE_WRITEDP         (3)
#define SWDBRIDGE_OR_OPCODE_READAP          (4)
#define SWDBRIDGE_OR_OPCODE_WRITEAP         (5)
#define SWDBRIDGE_OR_OPCODE_FLUSH           (6)
#define SWDBRIDGE_OR_OPCODE_LED             (7)
#define SWDBRIDGE_OR_OPCODE_READMEM         (8)
#define SWDBRIDGE_OR_OPCODE_WRITEMEM        (9)
#define SWDBRIDGE_OR_OPCODE_RUN             (10)
#define SWDBRIDGE_OR_OPCODE_STOP            (11)
#define SWDBRIDGE_OR_OPCODE_RESET           (12)
#define SWDBRIDGE_OR_OPCODE_DISABLEWDT      (13)
#define SWDBRIDGE_OR_OPCODE_READCORE        (14)
#define SWDBRIDGE_OR_OPCODE_WRITECORE       (15)
#define SWDBRIDGE_OR_OPCODE_SETUPINVOKE     (16)
#define SWDBRIDGE_OR_OPCODE_INVOKE          (17)

#define SWD_OPCODE_STR \
    (const char * []){"OPCODE_NOTHING",\
    "OPCODE_CONNECT",\
    "OPCODE_READDP",\
    "OPCODE_WRITEDP",\
    "OPCODE_READAP",\
    "OPCODE_WRITEAP",\
    "OPCODE_FLUSH",\
    "OPCODE_LED",\
    "OPCODE_READMEM",\
    "OPCODE_WRITEMEM",\
    "OPCODE_RUN",\
    "OPCODE_STOP",\
    "OPCODE_RESET",\
    "OPCODE_DISABLEWDT",\
    "OPCODE_READCORE",\
    "OPCODE_WRITECORE",\
    "OPCODE_SETUPINVOKE",\
    "OPCODE_INVOKE"}\

typedef USB_HID_GenericReportOut_t tOutReport;

/* SWD status responses. SWD_ACK is good. */
#define SWDBRIDGE_OR_STATUS_ACK             (0b001)
#define SWDBRIDGE_OR_STATUS_WAIT            (0b010)
#define SWDBRIDGE_OR_STATUS_FAULT           (0b100)
#define SWDBRIDGE_OR_STATUS_PARITY          (0b1000)
#define SWDBRIDGE_OR_STATUS_NOTHING         (0b11111111)


#define REPORT_STATUS_STR(n) \
    ((n) == SWDBRIDGE_OR_STATUS_ACK     ? "STATUS_ACK"     :\
     (n) == SWDBRIDGE_OR_STATUS_WAIT    ? "STATUS_WAIT"    :\
     (n) == SWDBRIDGE_OR_STATUS_FAULT   ? "STATUS_FAULT"   :\
     (n) == SWDBRIDGE_OR_STATUS_PARITY  ? "STATUS_PARITY"  :\
     (n) == SWDBRIDGE_OR_STATUS_NOTHING ? "STATUS_NOTHING" :\
     (n) == 7                           ? "STATUS_DISCONECT" : "UNKNOWN_STATUS")

typedef USB_HID_GenericReportIn_t tInReport;

extern volatile tInReport InReport;
//extern volatile tOutReport OutReport;

#define SWDBRIDGE_CORE_R0                   (0)
#define SWDBRIDGE_CORE_R1                   (1)
#define SWDBRIDGE_CORE_R2                   (2)
#define SWDBRIDGE_CORE_R3                   (3)
#define SWDBRIDGE_CORE_R4                   (4)
#define SWDBRIDGE_CORE_R5                   (5)
#define SWDBRIDGE_CORE_R6                   (6)
#define SWDBRIDGE_CORE_R7                   (7)
#define SWDBRIDGE_CORE_R8                   (8)
#define SWDBRIDGE_CORE_R9                   (9)
#define SWDBRIDGE_CORE_R10                  (10)
#define SWDBRIDGE_CORE_R11                  (11)
#define SWDBRIDGE_CORE_R12                  (12)
#define SWDBRIDGE_CORE_R13                  (13)
#define SWDBRIDGE_CORE_SP                   (13)
#define SWDBRIDGE_CORE_R14                  (14)
#define SWDBRIDGE_CORE_R15                  (15)
#define SWDBRIDGE_CORE_PC                   (15)
#define SWDBRIDGE_CORE_XPSR                 (16)

/* HID Demo Functions */
extern void swdbridgeGetInReport  (void);
extern void swdbridgeSetOutReport (void);
int swdbridgeInit (void);

#ifdef __cplusplus
}
#endif

#endif
