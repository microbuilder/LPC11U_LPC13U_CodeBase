;/**************************************************************************//**
; * @file     startup_LPC11Uxx.s
; * @brief    CMSIS Cortex-M0 Core Device Startup File for
; *           NXP LPC11Uxx Device Series
; * @version  V1.12
; * @date     10. October 2012
; *
; * @note
; * Copyright (C) 2012 ARM Limited. All rights reserved.
; *
; * @par
; * ARM Limited (ARM) is supplying this software for use with Cortex-M
; * processor based microcontrollers.  This file can be freely distributed
; * within development tools that are supporting such ARM based processors.
; *
; * @par
; * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
; * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
; * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
; * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
; * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
; *
; ******************************************************************************/

; *------- <<< Use Configuration Wizard in Context Menu >>> ------------------

; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000200

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000000

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts
                DCD     FLEX_INT0_IRQHandler      ; 16+ 0: All GPIO pin can be routed to FLEX_INTx
                DCD     FLEX_INT1_IRQHandler      ; 16+ 1: Pin interrupt
                DCD     FLEX_INT2_IRQHandler      ; 16+ 2: Pin interrupt
                DCD     FLEX_INT3_IRQHandler      ; 16+ 3: Pin interrupt
                DCD     FLEX_INT4_IRQHandler      ; 16+ 4: Pin interrupt
                DCD     FLEX_INT5_IRQHandler      ; 16+ 5: Pin interrupt
                DCD     FLEX_INT6_IRQHandler      ; 16+ 6: Pin interrupt
                DCD     FLEX_INT7_IRQHandler      ; 16+ 7: Pin interrupt
                DCD     GINT0_IRQHandler          ; 16+ 8: Port interrupt
                DCD     GINT1_IRQHandler          ; 16+ 9: Port interrupt
                DCD     Reserved_IRQHandler		  ; 16+10: Reserved
                DCD     Reserved_IRQHandler		  ; 16+11: Reserved
                DCD     Reserved_IRQHandler       ; 16+12: Reserved
                DCD     Reserved_IRQHandler       ; 16+13: Reserved
                DCD     SSP1_IRQHandler           ; 16+14: SSP1
                DCD     I2C_IRQHandler            ; 16+15: I2C
                DCD     TIMER16_0_IRQHandler      ; 16+16: 16-bit Timer0
                DCD     TIMER16_1_IRQHandler      ; 16+17: 16-bit Timer1
                DCD     TIMER32_0_IRQHandler      ; 16+18: 32-bit Timer0
                DCD     TIMER32_1_IRQHandler      ; 16+19: 32-bit Timer1
                DCD     SSP0_IRQHandler           ; 16+20: SSP0
                DCD     UART_IRQHandler           ; 16+21: UART
                DCD     USB_IRQHandler            ; 16+22: USB IRQ
                DCD     USB_FIQHandler            ; 16+23: USB FIQ
                DCD     ADC_IRQHandler            ; 16+24: A/D Converter
                DCD     WDT_IRQHandler            ; 16+25: Watchdog Timer
                DCD     BOD_IRQHandler            ; 16+26: Brown Out Detect
                DCD     FMC_IRQHandler            ; 16+27: IP2111 Flash Memory Controller
                DCD     Reserved_IRQHandler       ; 16+28: Reserved
                DCD     Reserved_IRQHandler       ; 16+29: Reserved
                DCD     USBWakeup_IRQHandler      ; 16+30: USB wake up
                DCD     Reserved_IRQHandler       ; 16+31: Reserved

; <h> Code Read Protection
;   <o> Code Read Protection  <0xFFFFFFFF=>CRP Disabled
;                             <0x12345678=>CRP Level 1
;                             <0x87654321=>CRP Level 2
;                             <0x43218765=>CRP Level 3 (ARE YOU SURE?)
;                             <0x4E697370=>NO ISP (ARE YOU SURE?)
; </h>
                IF      :LNOT::DEF:NO_CRP
                AREA    |.ARM.__at_0x02FC|, CODE, READONLY
                DCD     0xFFFFFFFF
                ENDIF

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main
                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP
Reserved_IRQHandler PROC
                EXPORT  Reserved_IRQHandler       [WEAK]
                B       .
                ENDP

Default_Handler PROC
                EXPORT  FLEX_INT0_IRQHandler      [WEAK]
                EXPORT  FLEX_INT1_IRQHandler      [WEAK]
                EXPORT  FLEX_INT2_IRQHandler      [WEAK]
                EXPORT  FLEX_INT3_IRQHandler      [WEAK]
                EXPORT  FLEX_INT4_IRQHandler      [WEAK]
                EXPORT  FLEX_INT5_IRQHandler      [WEAK]
                EXPORT  FLEX_INT6_IRQHandler      [WEAK]
                EXPORT  FLEX_INT7_IRQHandler      [WEAK]
                EXPORT  GINT0_IRQHandler          [WEAK]
                EXPORT  GINT1_IRQHandler          [WEAK]
                EXPORT  SSP1_IRQHandler           [WEAK]
                EXPORT  I2C_IRQHandler            [WEAK]
                EXPORT  TIMER16_0_IRQHandler      [WEAK]
                EXPORT  TIMER16_1_IRQHandler      [WEAK]
                EXPORT  TIMER32_0_IRQHandler      [WEAK]
                EXPORT  TIMER32_1_IRQHandler      [WEAK]
                EXPORT  SSP0_IRQHandler           [WEAK]
                EXPORT  UART_IRQHandler           [WEAK]
                EXPORT  USB_IRQHandler            [WEAK]
                EXPORT  USB_FIQHandler            [WEAK]
                EXPORT  ADC_IRQHandler            [WEAK]
                EXPORT  WDT_IRQHandler            [WEAK]
                EXPORT  BOD_IRQHandler            [WEAK]
                EXPORT  FMC_IRQHandler            [WEAK]
                EXPORT	USBWakeup_IRQHandler      [WEAK]

FLEX_INT0_IRQHandler
FLEX_INT1_IRQHandler
FLEX_INT2_IRQHandler
FLEX_INT3_IRQHandler
FLEX_INT4_IRQHandler
FLEX_INT5_IRQHandler
FLEX_INT6_IRQHandler
FLEX_INT7_IRQHandler
GINT0_IRQHandler
GINT1_IRQHandler
SSP1_IRQHandler
I2C_IRQHandler
TIMER16_0_IRQHandler
TIMER16_1_IRQHandler
TIMER32_0_IRQHandler
TIMER32_1_IRQHandler
SSP0_IRQHandler
UART_IRQHandler
USB_IRQHandler
USB_FIQHandler
ADC_IRQHandler
WDT_IRQHandler
BOD_IRQHandler
FMC_IRQHandler
USBWakeup_IRQHandler

                B       .

                ENDP


                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap
__user_initial_stackheap

                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR

                ALIGN

                ENDIF


                END
