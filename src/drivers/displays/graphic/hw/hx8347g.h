/**************************************************************************/
/*! 
    @file     hx8347g.h
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
#ifndef __HX8347G_H__
#define __HX8347G_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "projectconfig.h"
#include "drivers/displays/graphic/lcd.h"

// Backlight enable and reset pins
#define HX8347G_BL_PORT               (1)
#define HX8347G_BL_PIN                (27)
#define HX8347G_RES_PORT              (1)
#define HX8347G_RES_PIN               (28)

// Control pins (these should be on the same port for best performance)
#define HX8347G_CONTROL_PORT          (1)     
#define HX8347G_CS_PIN                (13)
#define HX8347G_CD_PIN                (14)
#define HX8347G_WR_PIN                (15)
#define HX8347G_RD_PIN                (16)

// Combined pin definitions for optimisation purposes.
#define HX8347G_CS_CD_PINS            ((1<<HX8347G_CS_PIN) + (1<<HX8347G_CD_PIN))
#define HX8347G_RD_WR_PINS            ((1<<HX8347G_RD_PIN) + (1<<HX8347G_WR_PIN))
#define HX8347G_WR_CS_PINS            ((1<<HX8347G_WR_PIN) + (1<<HX8347G_CS_PIN))
#define HX8347G_CD_RD_WR_PINS         ((1<<HX8347G_CD_PIN) + (1<<HX8347G_RD_PIN) + (1<<HX8347G_WR_PIN))
#define HX8347G_CS_CD_RD_WR_PINS      ((1<<HX8347G_CS_PIN) + (1<<HX8347G_CD_PIN) + (1<<HX8347G_RD_PIN) + (1<<HX8347G_WR_PIN))

// Data bus (data pins must be consecutive and on the same port)
#define HX8347G_DATA_PORT             (1)
#define HX8347G_DATA_PIN1             (19)
#define HX8347G_DATA_PIN2             (20)
#define HX8347G_DATA_PIN3             (21)
#define HX8347G_DATA_PIN4             (22)
#define HX8347G_DATA_PIN5             (23)
#define HX8347G_DATA_PIN6             (24)
#define HX8347G_DATA_PIN7             (25)
#define HX8347G_DATA_PIN8             (26)
#define HX8347G_DATA_OFFSET           (HX8347G_DATA_PIN1)
#define HX8347G_DATA_MASK             (0xFF << HX8347G_DATA_OFFSET)

// Macros to set data bus direction to input/output
#define HX8347G_GPIO2DATA_SETINPUT    do { LPC_GPIO->DIR[HX8347G_DATA_PORT] &= ~HX8347G_DATA_MASK; } while(0)
#define HX8347G_GPIO2DATA_SETOUTPUT   do { LPC_GPIO->DIR[HX8347G_DATA_PORT] |= HX8347G_DATA_MASK; } while(0)

// Macros for control line state
// NOPs required since the bit-banding is too fast for some HX8347Gs to handle :(
#define CLR_CD                        do { LPC_GPIO->CLR[HX8347G_CONTROL_PORT] = (1 << HX8347G_CD_PIN); } while(0)
#define SET_CD                        do { LPC_GPIO->SET[HX8347G_CONTROL_PORT] = (1 << HX8347G_CD_PIN); } while(0)
#define CLR_CS                        do { LPC_GPIO->CLR[HX8347G_CONTROL_PORT] = (1 << HX8347G_CS_PIN); } while(0)
#define SET_CS                        do { LPC_GPIO->SET[HX8347G_CONTROL_PORT] = (1 << HX8347G_CS_PIN); } while(0)
#define CLR_WR                        do { LPC_GPIO->CLR[HX8347G_CONTROL_PORT] = (1 << HX8347G_WR_PIN); } while(0)
#define SET_WR                        do { LPC_GPIO->SET[HX8347G_CONTROL_PORT] = (1 << HX8347G_WR_PIN); } while(0)
#define CLR_RD                        do { LPC_GPIO->CLR[HX8347G_CONTROL_PORT] = (1 << HX8347G_RD_PIN); } while(0)
#define SET_RD                        do { LPC_GPIO->SET[HX8347G_CONTROL_PORT] = (1 << HX8347G_RD_PIN); } while(0)
#define CLR_RESET                     do { LPC_GPIO->CLR[HX8347G_RES_PORT] = (1 << HX8347G_RES_PORT); } while(0)
#define SET_RESET                     do { LPC_GPIO->SET[HX8347G_RES_PORT] = (1 << HX8347G_RES_PORT); } while(0)
#define CLR_BL                        do { LPC_GPIO->CLR[HX8347G_BL_PORT] = (1 << HX8347G_BL_PORT); } while(0)
#define SET_BL                        do { LPC_GPIO->SET[HX8347G_BL_PORT] = (1 << HX8347G_BL_PORT); } while(0)

// These 'combined' macros are defined to improve code performance by
// reducing the number of instructions in heavily used functions
#define CLR_CS_CD                     do { LPC_GPIO->CLR[HX8347G_CONTROL_PORT] = (HX8347G_CS_CD_PINS); } while(0)
#define SET_RD_WR                     do { LPC_GPIO->SET[HX8347G_CONTROL_PORT] = (HX8347G_RD_WR_PINS); } while(0)
#define SET_WR_CS                     do { LPC_GPIO->SET[HX8347G_CONTROL_PORT] = (HX8347G_WR_CS_PINS); } while(0)
#define SET_CD_RD_WR                  do { LPC_GPIO->SET[HX8347G_CONTROL_PORT] = (HX8347G_CD_RD_WR_PINS); } while(0)
#define CLR_CS_CD_SET_RD_WR           do { LPC_GPIO->CLR[HX8347G_CONTROL_PORT] = (HX8347G_CS_CD_PINS); LPC_GPIO->SET[HX8347G_CONTROL_PORT] = (HX8347G_RD_WR_PINS); } while(0)
#define CLR_CS_SET_CD_RD_WR           do { LPC_GPIO->CLR[HX8347G_CONTROL_PORT] = (HX8347G_CS_PIN); LPC_GPIO->SET[HX8347G_CONTROL_PORT] = (HX8347G_CD_RD_WR_PINS); } while(0)

// Used to indicate a delay in the init sequence
#define HX8347G_INIT_DELAY                      (0xF8)  // 0xFF is already used

#define HX8347G_CMD_HIMAXID                     (0x00)
#define HX8347G_CMD_DISPLAYMODECONTROL          (0x01)
#define HX8347G_CMD_COLADDRSTART2               (0x02)
#define HX8347G_CMD_COLADDRSTART1               (0x03)
#define HX8347G_CMD_COLADDREND2                 (0x04)
#define HX8347G_CMD_COLADDREND1                 (0x05)
#define HX8347G_CMD_ROWADDRSTART2               (0x06)
#define HX8347G_CMD_ROWADDRSTART1               (0x07)
#define HX8347G_CMD_ROWADDREND2                 (0x08)
#define HX8347G_CMD_ROWADDREND1                 (0x09)
#define HX8347G_CMD_PARTIALAREASTARTROW2        (0x0A)
#define HX8347G_CMD_PARTIALAREASTARTROW1        (0x0B)
#define HX8347G_CMD_PARTIALAREAENDROW2          (0x0C)
#define HX8347G_CMD_PARTIALAREAENDROW1          (0x0D)
#define HX8347G_CMD_VERTICALSCROLLTOPFIXEDAREA2 (0x0E)
#define HX8347G_CMD_VERTICALSCROLLTOPFIXEDAREA1 (0x0F)
#define HX8347G_CMD_VERTICALSCROLLHEIGHTAREA2   (0x10)
#define HX8347G_CMD_VERTICALSCROLLHEIGHTAREA1   (0x11)
#define HX8347G_CMD_VERTICALSCROLLBUTTONAREA2   (0x12)
#define HX8347G_CMD_VERTICALSCROLLBUTTONAREA1   (0x13)
#define HX8347G_CMD_VERTICALSCROLLSTARTADDR2    (0x14)
#define HX8347G_CMD_VERTICALSCROLLSTARTADDR1    (0x15)
#define HX8347G_CMD_MEMORYACCESSCONTROL         (0x16)
#define HX8347G_CMD_COLMOD                      (0x17)
#define HX8347G_CMD_OSCCONTROL2                 (0x18)
#define HX8347G_CMD_OSCCONTROL1                 (0x19)
#define HX8347G_CMD_POWERCONTROL1               (0x1A)
#define HX8347G_CMD_POWERCONTROL2               (0x1B)
#define HX8347G_CMD_POWERCONTROL3               (0x1C)
#define HX8347G_CMD_POWERCONTROL4               (0x1D)
#define HX8347G_CMD_POWERCONTROL5               (0x1E)
#define HX8347G_CMD_POWERCONTROL6               (0x1F)
#define HX8347G_CMD_SRAMWRITECONTROL            (0x22)
#define HX8347G_CMD_VCOMCONTROL1                (0x23)
#define HX8347G_CMD_VCOMCONTROL2                (0x24)
#define HX8347G_CMD_VCOMCONTROL3                (0x25)
#define HX8347G_CMD_DISPLAYCONTROL1             (0x26)
#define HX8347G_CMD_DISPLAYCONTROL2             (0x27)
#define HX8347G_CMD_DISPLAYCONTROL3             (0x28)
#define HX8347G_CMD_FRAMERATECONTROL1           (0x29)
#define HX8347G_CMD_FRAMERATECONTROL2           (0x2A)
#define HX8347G_CMD_FRAMERATECONTROL3           (0x2B)
#define HX8347G_CMD_FRAMERATECONTROL4           (0x2C)
#define HX8347G_CMD_CYCLECONTROL1               (0x2D)
#define HX8347G_CMD_CYCLECONTROL2               (0x2E)
#define HX8347G_CMD_DISPLAYINVERSION            (0x2F)
#define HX8347G_CMD_RGBINTERFACECONTROL1        (0x31)
#define HX8347G_CMD_RGBINTERFACECONTROL2        (0x32)
#define HX8347G_CMD_RGBINTERFACECONTROL3        (0x33)
#define HX8347G_CMD_RGBINTERFACECONTROL4        (0x34)
#define HX8347G_CMD_PANELCHARACTERISTICS        (0x36)
#define HX8347G_CMD_OTPCONTROL1                 (0x38)
#define HX8347G_CMD_OTPCONTROL2                 (0x39)
#define HX8347G_CMD_OTPCONTROL3                 (0x3A)
#define HX8347G_CMD_OTPCONTROL4                 (0x3B)
#define HX8347G_CMD_CABCCONTROL1                (0x3C)
#define HX8347G_CMD_CABCCONTROL2                (0x3D)
#define HX8347G_CMD_CABCCONTROL3                (0x3E)
#define HX8347G_CMD_CABCCONTROL4                (0x3F)
#define HX8347G_CMD_TECONTROL                   (0x60)
#define HX8347G_CMD_ID1                         (0x61)
#define HX8347G_CMD_ID2                         (0x62)
#define HX8347G_CMD_ID3                         (0x63)
#define HX8347G_CMD_TEOUTPUTLINE2               (0x84)
#define HX8347G_CMD_TEOUTPUTLINE1               (0x85)
#define HX8347G_CMD_POWERSAVING1                (0xE4)
#define HX8347G_CMD_POWERSAVING2                (0xE5)
#define HX8347G_CMD_POWERSAVING3                (0xE6)
#define HX8347G_CMD_POWERSAVING4                (0xE7)
#define HX8347G_CMD_SOURCEOP_CONTROLNORMAL      (0xE8)
#define HX8347G_CMD_SOURCEOP_CONTROLIDLE        (0xE9)
#define HX8347G_CMD_PAGESELECT                  (0xFF)

#ifdef __cplusplus
}
#endif 

#endif
