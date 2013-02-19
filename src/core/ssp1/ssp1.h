/**************************************************************************/
/*!
    @file     ssp1.h
    @author   K. Townsend (microBuilder.eu)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2012, K. Townsend (microBuilder.eu)
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
#ifndef __SSP1_H__
#define __SSP1_H__

#ifdef __cplusplus
extern "C" {
#endif

#define SSP1_FIFOSIZE            8       /* SPI read and write buffer size */

/* SSP Clock div (0..255) */
#define SCB_CLKDIV_DISABLE (0x00000000)
#define SCB_CLKDIV_DIV1    (0x00000001)
#define SCB_CLKDIV_DIV2    (0x00000002)
#define SCB_CLKDIV_DIV3    (0x00000003)
#define SCB_CLKDIV_DIV4    (0x00000004)
#define SCB_CLKDIV_DIV6    (0x00000006)
#define SCB_CLKDIV_DIV10   (0x0000000A)
#define SCB_CLKDIV_DIV12   (0x0000000C)
#define SCB_CLKDIV_DIV15   (0x0000000F)
#define SCB_CLKDIV_DIV20   (0x00000014)
#define SCB_CLKDIV_DIV40   (0x00000028)
#define SCB_CLKDIV_MASK    (0x000000FF)

/* SSP Serial Clock Rate Values */
#define SSP1_SCR_1          (0x00000100)
#define SSP1_SCR_2          (0x00000200)
#define SSP1_SCR_3          (0x00000300)
#define SSP1_SCR_4          (0x00000400)
#define SSP1_SCR_5          (0x00000500)
#define SSP1_SCR_6          (0x00000600)
#define SSP1_SCR_7          (0x00000700)
#define SSP1_SCR_8          (0x00000800)
#define SSP1_SCR_9          (0x00000900)
#define SSP1_SCR_10         (0x00000A00)
#define SSP1_SCR_11         (0x00000B00)
#define SSP1_SCR_12         (0x00000C00)
#define SSP1_SCR_13         (0x00000D00)
#define SSP1_SCR_14         (0x00000E00)
#define SSP1_SCR_15         (0x00000F00)
#define SSP1_SCR_16         (0x00001000)

/*  Current status of the SSP controller.  */
#define SSP1_SR_TFE_MASK      (0x00000001) // Transmit FIFO empty
#define SSP1_SR_TFE_EMPTY     (0x00000001)
#define SSP1_SR_TFE_NOTEMPTY  (0x00000000)
#define SSP1_SR_TNF_MASK      (0x00000002) // Transmit FIFO not full
#define SSP1_SR_TNF_NOTFULL   (0x00000002)
#define SSP1_SR_TNF_FULL      (0x00000000)
#define SSP1_SR_RNE_MASK      (0x00000004) // Receive FIFO not empty
#define SSP1_SR_RNE_NOTEMPTY  (0x00000004)
#define SSP1_SR_RNE_EMPTY     (0x00000000)
#define SSP1_SR_RFF_MASK      (0x00000008) // Receive FIFO full
#define SSP1_SR_RFF_FULL      (0x00000008)
#define SSP1_SR_RFF_NOTFULL   (0x00000000)
#define SSP1_SR_BSY_MASK      (0x00000010) // Busy Flag
#define SSP1_SR_BSY_IDLE      (0x00000000)
#define SSP1_SR_BSY_BUSY      (0x00000010)

/* Control Register 1 */
#define SSP1_CR1_LBM_MASK     (0x00000001) // Loop back mode
#define SSP1_CR1_LBM_NORMAL   (0x00000000)
#define SSP1_CR1_LBM_INVERTED (0x00000001) // MISO/MOSI are reversed
#define SSP1_CR1_SSE_MASK     (0x00000002) // SSP enable
#define SSP1_CR1_SSE_DISABLED (0x00000000)
#define SSP1_CR1_SSE_ENABLED  (0x00000002)
#define SSP1_CR1_MS_MASK      (0x00000004) // Master/Slave Mode
#define SSP1_CR1_MS_MASTER    (0x00000000)
#define SSP1_CR1_MS_SLAVE     (0x00000004)
#define SSP1_CR1_SOD_MASK     (0x00000008) // Slave output disable

void ssp1ClockSlow(void);
void ssp1ClockFast(void);
void ssp1Init(void);
void ssp1Send(uint8_t *buf, uint32_t length);
void ssp1Receive(uint8_t *buf, uint32_t length);

#ifdef __cplusplus
}
#endif 

#endif
