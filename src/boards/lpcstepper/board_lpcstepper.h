/**************************************************************************/
/*!
    @file     board_lpcstepper.h
    @author   K. Townsend (microBuilder.eu)

    @brief    Board file for the LPC1347 stepper driver board (DRV8833)
    @ingroup  Boards

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013 K. Townsend
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
#ifndef __BOARD_LPCSTEPPER_H__
#define __BOARD_LPCSTEPPER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "sysdefs.h"

/*=========================================================================
    MCU SELECTION

    Include one of the following definitions depending on if you are
    using the LPC11U37 or LPC1347.  They're generally interchangeable, but
    the LPC11Uxx and LPC13xx CMSIS implementations have some differences
    in naming convention, and occasionally a feature is only available on
    the M3 (DWT for example).  Selecting the appropriate MCU allows
    the right code to be included when differences between the CMSIS
    implementations are present.

    -----------------------------------------------------------------------*/
    // #define CFG_MCU_LPC11U24FBD48_401
    // #define CFG_MCU_LPC11U37FBD48_401
    // #define CFG_MCU_LPC1347FBD48
    #define CFG_MCU_LPC1347FHN33

    /* Basic error checking */
    #if !defined CFG_MCU_LPC1347FBD48 && \
        !defined CFG_MCU_LPC11U37FBD48_401 && \
        !defined CFG_MCU_LPC11U24FBD48_401 && \
        !defined CFG_MCU_LPC1347FHN33
      #error "An MCU must be selected in projectconfig.h (Ex. CFG_MCU_LPC11U37FBD48_401, CFG_MCU_LPC1347FBD48, etc.)"
    #endif

    /* Set flag to indicate which CMSIS library to use */
    #if (defined CFG_MCU_LPC1347FBD48 || defined CFG_MCU_LPC1347FHN33)
      #define CFG_MCU_FAMILY_LPC13UXX
    #elif (defined CFG_MCU_LPC11U24FBD48_401 || defined CFG_MCU_LPC11U37FBD48_401)
      #define CFG_MCU_FAMILY_LPC11UXX
    #endif

    /* Include the correct MCU header file */
    #if defined CFG_MCU_FAMILY_LPC13UXX
      #include "LPC13Uxx.h"
    #endif
    #if defined CFG_MCU_FAMILY_LPC11UXX
      #include "LPC11Uxx.h"
    #endif
/*=========================================================================*/


/*=========================================================================
    FIRMWARE VERSION SETTINGS
    -----------------------------------------------------------------------*/
    #define CFG_FIRMWARE_VERSION_MAJOR      (0)
    #define CFG_FIRMWARE_VERSION_MINOR      (0)
    #define CFG_FIRMWARE_VERSION_REVISION   (0)
/*=========================================================================*/


/*=========================================================================
    GPIO INTERRUPTS
    -----------------------------------------------------------------------
    This table shows where GPIO interrupts are mapped in this project
    (Note that the LPC11U and LPC13U use different names for the
    IRQ Handlers in the standard headers)

    Interrupt                                     Location
    ------------------------------------------    -------------------------
    PIN_INT0_IRQHandler - FLEX_INT0_IRQHandler    chb_drvr.c
    PIN_INT1_IRQHandler - FLEX_INT1_IRQHandler    pcf2129.c
    PIN_INT2_IRQHandler - FLEX_INT2_IRQHandler
    PIN_INT3_IRQHandler - FLEX_INT3_IRQHandler
    PIN_INT4_IRQHandler - FLEX_INT4_IRQHandler
    PIN_INT5_IRQHandler - FLEX_INT5_IRQHandler
    PIN_INT6_IRQHandler - FLEX_INT6_IRQHandler
    PIN_INT7_IRQHandler - FLEX_INT7_IRQHandler
    GINT0_IRQHandler
    GINT0_IRQHandler
    -----------------------------------------------------------------------*/
/*=========================================================================*/


/*=========================================================================
    SUPPORTED PERIPHERALS
    -----------------------------------------------------------------------
    Because all ISRs are referenced in the startup code, GCC typically
    won't optimise out the ISR functions during compilation even if the
    ISRs will never be entered, resulting in larger binaries than required
    (for example if no I2C sensors are used, be sure to disable I2C support
    since the I2C ISR is quite large).

    Use the defines below to include or exclude support for specific
    peripherals.

    NOTE: GPIO ISRs are handled separately in GPIO INTERRUPTS below
    -----------------------------------------------------------------------*/
    #define CFG_ENABLE_I2C
    #define CFG_ENABLE_UART
    #define CFG_ENABLE_USB
    #define CFG_ENABLE_TIMER32
/*=========================================================================*/


/*=========================================================================
    EEPROM
    -----------------------------------------------------------------------
    EEPROM is used to persist certain user modifiable values to make
    sure that these changes remain in effect after a reset or hard
    power-down.  The addresses in EEPROM for these various system
    settings/values are defined below.  The first 256 bytes of EEPROM
    are reserved for this (0x0000..0x00FF).

    CFG_EEPROM_SIZE           The number of bytes available on the EEPROM
    CFG_EEPROM_RESERVED       The last byte of reserved EEPROM memory

          EEPROM Address (0x0000..0x00FF)
          ===============================
          0 1 2 3 4 5 6 7 8 9 A B C D E F
    000x  x x . . x x x x x x x x . . . .   Chibi
    001x  . . . . . . . . . . . . . . . .
    002x  . . . . . . . . . . . . . . . .
    003x  . . . . . . . . . . . . . . . .
    004x  . . . . . . . . . . . . . . . .
    005x  . . . . . . . . . . . . . . . .
    006x  . . . . . . . . . . . . . . . .
    007x  . . . . . . . . . . . . . . . .
    008x  . . . . . . . . . . . . . . . .
    009x  . . . . . . . . . . . . . . . .
    00Ax  . . . . . . . . . . . . . . . .
    00Bx  . . . . . . . . . . . . . . . .
    00Cx  . . . . . . . . . . . . . . . .
    00Dx  . . . . . . . . . . . . . . . .
    00Ex  . . . . . . . . . . . . . . . .
    00Fx  . . . . . . . . . . . . . . . .

    -----------------------------------------------------------------------*/
    #define CFG_EEPROM_SIZE                   (4032)
    #define CFG_EEPROM_RESERVED               (0x00FF)              // Protect first 256 bytes of memory
    #define CFG_EEPROM_CHIBI_NODEADDR         (uint16_t)(0x0000)    // 2
    #define CFG_EEPROM_CHIBI_IEEEADDR         (uint16_t)(0x0004)    // 8
/*=========================================================================*/


/*=========================================================================
    ON-BOARD LED
    -----------------------------------------------------------------------

    CFG_LED_PORT              The port for the on board LED
    CFG_LED_PIN               The pin for the on board LED
    CFG_LED_ON                The pin state to turn the LED on (0 = low, 1 = high)
    CFG_LED_OFF               The pin state to turn the LED off (0 = low, 1 = high)

    -----------------------------------------------------------------------*/
    #define CFG_LED_PORT                  (0)
    #define CFG_LED_PIN                   (23)
    #define CFG_LED_ON                    (0)
    #define CFG_LED_OFF                   (1)
/*=========================================================================*/


/*=========================================================================
    ADC
    -----------------------------------------------------------------------

    CFG_ADC_MODE_LOWPOWER     If set to 1, this will configure the ADC
                              for low-power operation (LPC1347 only)
    CFG_ADC_MODE_10BIT        If set to 1, this will configure the ADC
                              for 10-bit mode (LPC1347 only)

    -----------------------------------------------------------------------*/
    #define CFG_ADC_MODE_LOWPOWER       (0)
    #define CFG_ADC_MODE_10BIT          (0)
/*=========================================================================*/


/*=========================================================================
    UART
    -----------------------------------------------------------------------

    CFG_UART_BAUDRATE         The default UART speed.  This value is used
                              when initialising UART, and should be a
                              standard value like 57600, 9600, etc.
                              NOTE: This value may be overridden if
                              another value is stored in EEPROM!
    CFG_UART_BUFSIZE          The length in bytes of the UART RX FIFO. This
                              will determine the maximum number of received
                              characters to store in memory.

    -----------------------------------------------------------------------*/
    #define CFG_UART_BAUDRATE           (115200)
    #define CFG_UART_BUFSIZE            (256)
/*=========================================================================*/


/*=========================================================================
    SPI
    -----------------------------------------------------------------------

    CFG_SSP_SCK0_LOCATION     The location of the SCK pin for SSP0
    CFG_SSP_MISO1_LOCATION    The location of the MISO1 pin for SSP1
    CFG_SSP_MOSI1_LOCATION    The location of the MOSI1 pin for SSP1
    CFG_SSP_SCK1_LOCATION     The location of the SCK pin for SSP1

    -----------------------------------------------------------------------*/
    #define CFG_SSP_SCK0_0_6            (6)     // Used by USBConnect
    #define CFG_SSP_SCK0_0_10           (10)    // Used by SWD
    #define CFG_SSP_SCK0_1_29           (29)

    #define CFG_SSP_MISO1_0_22          (22)
    #define CFG_SSP_MISO1_1_21          (21)
    #define CFG_SSP_MOSI1_0_21          (21)
    #define CFG_SSP_MOSI1_1_22          (22)
    #define CFG_SSP_SCK1_1_15           (15)
    #define CFG_SSP_SCK1_1_20           (20)

    // Select the appropriate pin locations here
    #define CFG_SSP_SCK0_LOCATION       (CFG_SSP_SCK0_1_29)
    #define CFG_SSP_MISO1_LOCATION      (CFG_SSP_MISO1_1_21)
    #define CFG_SSP_MOSI1_LOCATION      (CFG_SSP_MOSI1_1_22)
    #define CFG_SSP_SCK1_LOCATION       (CFG_SSP_SCK1_1_20)

    // Set the phase and polarity for SSP0 and SSP1
    #define CFG_SSP_CPOL0               (0)
    #define CFG_SSP_CPHA0               (0)
    #define CFG_SSP_CPOL1               (0)
    #define CFG_SSP_CPHA1               (0)
/*=========================================================================*/


/*=========================================================================
    PRINTF REDIRECTION
    -----------------------------------------------------------------------

    CFG_PRINTF_MAXSTRINGSIZE  Maximum size of string buffer for printf
    CFG_PRINTF_UART           Will cause all printf statements to be
                              redirected to UART
    CFG_PRINTF_USBCDC         Will cause all printf statements to be
                              redirect to USB Serial
    CFG_PRINTF_NEWLINE        This is typically "\r\n" for Windows or
                              "\n" for *nix

    Note: If no printf redirection definitions are present, all printf
    output will be ignored.
    -----------------------------------------------------------------------*/
    #define CFG_PRINTF_MAXSTRINGSIZE    (255)
    // #define CFG_PRINTF_UART
    #define CFG_PRINTF_USBCDC
    // #define CFG_PRINTF_DEBUG

    #ifdef CFG_PRINTF_DEBUG
      #define CFG_PRINTF_NEWLINE          "\n"
    #else
      #define CFG_PRINTF_NEWLINE          "\r\n"
    #endif
/*=========================================================================*/


/*=========================================================================
    COMMAND LINE INTERFACE
    -----------------------------------------------------------------------

    CFG_INTERFACE             If this field is defined the UART or USBCDC
                              based command-line interface will be included
    CFG_INTERFACE_MAXMSGSIZE  The maximum number of bytes to accept for an
                              incoming command
    CFG_INTERFACE_PROMPT      The command prompt to display at the start
                              of every new data entry line
    CFG_INTERFACE_SILENTMODE  If this is set to 1 only text generated in
                              response to commands will be send to the
                              output buffer.  The command prompt will not
                              be displayed and incoming text will not be
                              echoed back to the output buffer (allowing
                              you to see the text you have input).  This
                              is normally only desirable in a situation
                              where another MCU is communicating with
                              the LPC1343.
    CFG_INTERFACE_DROPCR      If this is set to 1 all incoming \r
                              characters will be dropped
    CFG_INTERFACE_ENABLEIRQ   If this is set to 1 the IRQ pin will be
                              set high when a command starts executing
                              and will go low when the command has
                              finished executing or the LCD is not busy.
                              This allows another device to know when a
                              new command can safely be sent.
    CFG_INTERFACE_IRQPORT     The gpio port for the IRQ/busy pin
    CFG_INTERFACE_IRQPIN      The gpio pin number for the IRQ/busy pin
    CFG_INTERFACE_SHORTERRORS If this is enabled only short 1 character
                              error messages will be returned (followed
                              by CFG_PRINTF_NEWLINE), rather than more
                              verbose error messages.  The specific
                              characters used are defined below.
    CFG_INTERFACE_CONFIRMREADY  If this is set to 1 a text confirmation
                              will be sent when the command prompt is
                              ready for a new command.  This is in
                              addition to CFG_INTERFACE_ENABLEIRQ if
                              this is also enabled.  The character used
                              is defined below.
    CFG_INTERFACE_LONGSYSINFO If this is set to 1 extra information will
                              be included in the Sys Info ('V') command
                              on the CLI. This can be useful when trying
                              to debug problems on remote HW, or with
                              unknown firmware.  It will also use about
                              0.5KB flash, though, so only enable it is
                              necessary.

    NOTE:                     The command-line interface will use either
                              USB-CDC or UART depending on whether
                              CFG_PRINTF_UART or CFG_PRINTF_USBCDC are
                              selected.
    -----------------------------------------------------------------------*/
    #define CFG_INTERFACE
    #define CFG_INTERFACE_MAXMSGSIZE    (256)
    #define CFG_INTERFACE_PROMPT        "LPCSTEPPER >> "
    #define CFG_INTERFACE_SILENTMODE    (0)
    #define CFG_INTERFACE_DROPCR        (0)
    #define CFG_INTERFACE_ENABLEIRQ     (0)
    #define CFG_INTERFACE_IRQPORT       (0)
    #define CFG_INTERFACE_IRQPIN        (7)
    #define CFG_INTERFACE_SHORTERRORS   (0)
    #define CFG_INTERFACE_CONFIRMREADY  (0)
    #define CFG_INTERFACE_LONGSYSINFO   (1)
/*=========================================================================*/


/*=========================================================================
    SIMPLE BINARY PROTOCOL
    -----------------------------------------------------------------------

    CFG_PROTOCOL             If this field is defined the binary command
                              parser will be included
    -----------------------------------------------------------------------*/
    // #define CFG_PROTOCOL

    // #define CFG_PROTOCOL_VIA_HID
    #define CFG_PROTOCOL_VIA_BULK

    #if defined(CFG_PROTOCOL) && !defined(CFG_PROTOCOL_VIA_HID) && !defined(CFG_PROTOCOL_VIA_BULK)
        #error CFG_PROTOCOL must be enabled with either CFG_PROTOCOL_VIA_HID or CFG_PROTOCOL_VIA_BULK
    #endif
/*=========================================================================*/


/*=========================================================================
    TFT LCD
    -----------------------------------------------------------------------

    CFG_TFTLCD                  If defined, this will cause drivers for
                                a pre-determined LCD screen to be included
                                during build.  Only one LCD driver can be
                                included during the build process (for ex.
                                'drivers/displays/hw/ILI9325.c')
    CFG_TFTLCD_INCLUDESMALLFONTS If set to 1, smallfont support will be
                                included for 3x6, 5x8, 7x8 and 8x8 fonts.
                                This should only be enabled if these small
                                fonts are required since there is already
                                support for larger fonts generated with
                                Dot Factory
                                http://www.pavius.net/downloads/tools/53-the-dot-factory
    CFG_TFTLCD_USEAAFONTS       If set to a non-zero value, anti-aliased
                                fonts will be used instead of regular 1-bit
                                font.  These result in much higher-
                                quality text, but the fonts are 2 or 4
                                times larger than plain bitmap fonts and
                                take a bit more rendering time to display.
    CFG_TFTLCD_TS_DEFAULTTHRESHOLD  Default minimum threshold to trigger a
                                touch event with the touch screen (and exit
                                from 'tsWaitForEvent' in touchscreen.c).
                                Should be an 8-bit value somewhere between
                                8 and 75 in normal circumstances.  This is
                                the default value and may be overriden by
                                a value stored in EEPROM.
    CFG_TFTLCD_TS_KEYPADDELAY   The delay in milliseconds between key
                                presses in dialogue boxes
    ----------------------------------------------------------------------*/
    // #define CFG_TFTLCD
    #define CFG_TFTLCD_INCLUDESMALLFONTS   (1)
    #define CFG_TFTLCD_USEAAFONTS          (0)
    #define CFG_TFTLCD_TS_DEFAULTTHRESHOLD (50)
    #define CFG_TFTLCD_TS_KEYPADDELAY      (100)
/*=========================================================================*/


/*=========================================================================
    MICRO-SD CARD
    -----------------------------------------------------------------------

    CFG_SDCARD                If this field is defined SD Card and FAT32
                              file system support will be included
    CFG_SDCARD_SPIPORT        SSP Port used for the SD card (0 or 1)
    CFG_SDCARD_READONLY       If this is set to 1, all commands to
                              write to the SD card will be removed
                              saving some flash space.
    CFG_SDCARD_CDPORT         The card detect port number
    CFG_SDCARD_CDPIN          The card detect pin number

    NOTE:                     All config settings for FAT32 are defined
                              in ffconf.h
    -----------------------------------------------------------------------*/
    // #define CFG_SDCARD
    #define CFG_SDCARD_READONLY         (1)   // Must be 0 or 1
    #define CFG_SDCARD_SPIPORT          (0)
    #define CFG_SDCARD_SSELPORT         (0)
    #define CFG_SDCARD_SSELPIN          (0)
    #define CFG_SDCARD_CDPORT           (0)
    #define CFG_SDCARD_CDPIN            (0)
    #define CFG_SDCARD_ENBLPORT         (0)
    #define CFG_SDCARD_ENBLPIN          (0)

    #ifdef CFG_SDCARD
      #if !((CFG_SDCARD_READONLY == 0) || (CFG_SDCARD_READONLY == 1))
        #error "Invalid value for CFG_SDCARD_READONLY"
      #endif
      #if !((CFG_SDCARD_SPIPORT == 0) || (CFG_SDCARD_SPIPORT == 1))
        #error "Invalid SPI port for CFG_SDCARD_SPIPORT"
      #endif
    #endif
/*=========================================================================*/


/*=========================================================================
    CHIBI WIRELESS STACK
    -----------------------------------------------------------------------

    CFG_CHIBI                   If defined, the CHIBI wireless stack will be
                                included during build.  Requires external HW.
    CFG_CHIBI_MODE              The mode to use when receiving and transmitting
                                wireless data.  See chb_drvr.h for possible values
    CFG_CHIBI_POWER             The power level to use when transmitting.  See
                                chb_drvr.h for possible values
    CFG_CHIBI_CHANNEL           802.15.4 Channel (0 = 868MHz, 1-10 = 915MHz)
    CFG_CHIBI_PANID             16-bit PAN Identifier (ex.0x1234)
    CFG_CHIBI_PROMISCUOUS       Set to 1 to enabled promiscuous mode or
                                0 to disable it.  If promiscuous mode is
                                enabled be sure to set CFG_CHIBI_BUFFERSIZE
                                to an appropriately large value (ex. 1024)
    CFG_CHIBI_BUFFERSIZE        The size of the message buffer in bytes
    -----------------------------------------------------------------------*/
    // #define CFG_CHIBI
    #define CFG_CHIBI_MODE              (0)                 // OQPSK_868MHZ
    #define CFG_CHIBI_POWER             (0xE9)              // CHB_PWR_EU2_3DBM
    #define CFG_CHIBI_CHANNEL           (0)                 // 868-868.6 MHz
    #define CFG_CHIBI_PANID             (0x1234)
    #define CFG_CHIBI_PROMISCUOUS       (0)
    #define CFG_CHIBI_BUFFERSIZE        (256)

    // Pin config settings
    #define CFG_CHIBI_SPIPORT           (0)  // Must be 0 or 1
    #define CFG_CHIBI_SSPORT            (0)
    #define CFG_CHIBI_SSPIN             (0)
    #define CFG_CHIBI_EINTPORT          (0)
    #define CFG_CHIBI_EINTPIN           (0)
    #define CFG_CHIBI_RSTPORT           (0)
    #define CFG_CHIBI_RSTPIN            (0)
    #define CFG_CHIBI_SLPTRPORT         (0)
    #define CFG_CHIBI_SLPTRPIN          (0)
    #define CFG_CHIBI_CC1190_HGM_PORT   (0)   // Not used
    #define CFG_CHIBI_CC1190_HGM_PIN    (0)   // Not used

    #ifdef CFG_CHIBI
      #if !((CFG_CHIBI_SPIPORT == 0) || (CFG_CHIBI_SPIPORT == 1))
        #error "Invalid SPI port for CFG_CHIBI_SPIPORT"
      #endif
    #endif
/*=========================================================================*/


/*=========================================================================
    PN532/NFC STACK
    -----------------------------------------------------------------------

    CFG_PN532                      If defined, the PN532/NFC stack will be
                                   included during build.  Requires
                                   external HW
    CFG_PN532_MEM_POOL_SIZE_BYTES  Size of the dynamic memory pool in bytes
                                   (used by pn532/mem_allocator/ when
                                   working with NDEF messages)
    -----------------------------------------------------------------------*/
    // #define CFG_PN532
    #define CFG_PN532_RSTPD_PORT                      (0)
    #define CFG_PN532_RSTPD_PIN                       (16)
    #define CFG_PN532_I2C_IRQPORT                     (0)
    #define CFG_PN532_I2C_IRQPIN                      (17)
    #define CFG_PN532_MEM_POOL_SIZE_BYTES             (512)
/*=========================================================================*/


/*=========================================================================
    RTC SUPPORT
    -----------------------------------------------------------------------

    CFG_RTC                     If defined, RTC support will be included.
                                Requires external HW.
    -----------------------------------------------------------------------*/
    // #define CFG_RTC

    #if defined(CFG_RTC) && !defined(CFG_ENABLE_I2C)
      #error "CFG_ENABLE_I2C must be defined with CFG_RTC"
    #endif
/*=========================================================================*/


/*=========================================================================
    STEPPER MOTOR SUPPORT
    -----------------------------------------------------------------------

    CFG_STEPPER                 If defined, basic stepper motor support
                                will be included.  Requires external HW.
    -----------------------------------------------------------------------*/
    #define CFG_STEPPER
    #define CFG_STEPPER_TIMER32                       (0)
    #define CFG_STEPPER_IN1_PORT                      (0)
    #define CFG_STEPPER_IN1_PIN                       (8)
    #define CFG_STEPPER_IN2_PORT                      (0)
    #define CFG_STEPPER_IN2_PIN                       (9)
    #define CFG_STEPPER_IN3_PORT                      (0)
    #define CFG_STEPPER_IN3_PIN                       (14)
    #define CFG_STEPPER_IN4_PORT                      (0)
    #define CFG_STEPPER_IN4_PIN                       (13)
/*=========================================================================*/


/*=========================================================================
    USB

    CFG_USB_STRING_MANUFACTURER Manufacturer name that will appear in the
                                device descriptor during USB enumeration
    CFG_USB_STRING_PRODUCT      Product name that will appear in the
                                device descriptor during USB enumeration
    CFG_USB_VENDORID            16-bit USB vendor ID
    USB_PRODUCT_ID              Define this to set a custom product ID
                                if you do not wish to use the 'auto'
                                product ID feature
    CFG_CDC                     Enable USB CDC support
    CFG_USB_HID_KEYBOARD        Enable USB HID keyboard emulation
    CFG_USB_HID_MOUSE           Enable USB HID mouse emulation for a five
                                button 'Windows' mouse with scroll wheels
    CFG_USB_HID_GENERIC         Enable USB HID Generic support for custom
                                in and out reports, with report size set
                                via CFG_USB_HID_GENERIC_REPORT_SIZE
    CFG_USB_MSC                 Enable USB Mass Storage support, pointing
                                to the SD card reader (requires mmc.c from
                                the FATFS drivers, but doesn't use FATFS)


    You can combine more than one USB class below and they will be
    automatically combined in a USB composite device within the limit of
    available USB endpoints.  The USB Product ID is calculated automatically
    based on the combination of classes defined below.

    NOTE: Windows requires the .inf file in '/core/usb' for CDC support
    -----------------------------------------------------------------------*/
    #ifdef CFG_ENABLE_USB
      #define CFG_USB_STRING_MANUFACTURER       "microBuilder.eu"
      #define CFG_USB_STRING_PRODUCT            "LPCStepper"
      #define CFG_USB_VENDORID                  (0x1FC9)

      #define CFG_USB_CDC

      // #define CFG_USB_HID_KEYBOARD
      // #define CFG_USB_HID_MOUSE
      #define CFG_USB_HID_GENERIC
      #define CFG_USB_HID_GENERIC_REPORT_SIZE (64)

      // #define CFG_USB_MSC

      // #define CFG_USB_CUSTOM_CLASS

      #if (defined(CFG_USB_CDC)       || defined(CFG_USB_HID_KEYBOARD) || \
           defined(CFG_USB_HID_MOUSE) || defined(CFG_USB_HID_GENERIC)  || \
           defined(CFG_USB_MSC)       || defined(CFG_USB_CUSTOM_CLASS))
        #define CFG_USB
        #if defined(CFG_USB_HID_KEYBOARD) || defined(CFG_USB_HID_MOUSE) || defined(CFG_USB_HID_GENERIC)
          #define CFG_USB_HID
          #if defined(CFG_USB_HID_GENERIC) && (CFG_USB_HID_GENERIC_REPORT_SIZE > 64)
            #error "CFG_USB_HID_GENERIC_REPORT_SIZE exceeds the maximum value of 64 bytes (based on USB specs 2.0 for 'Full Speed Interrupt Endpoint Size')"
          #endif
        #endif
      #endif
    #endif
/*=========================================================================*/


/*=========================================================================
    CONFIG FILE VALIDATION

    Basic error checking to make sure that incompatible defines are not
    enabled at the same time, etc.

    -----------------------------------------------------------------------*/
    #if defined(CFG_INTERFACE) && !( defined CFG_PRINTF_UART || defined CFG_PRINTF_USBCDC || defined CFG_PRINTF_DEBUG)
      #error "At least one CFG_PRINTF target must be defined with CFG_INTERFACE"
    #endif

    #if defined(CFG_PRINTF_UART) && !defined(CFG_ENABLE_UART)
      #error "CFG_ENABLE_UART must be enabled with CFG_PRINTF_UART"
    #endif

    #if defined(CFG_PRINTF_USBCDC) && !defined(CFG_USB_CDC)
      #error "CFG_USB_CDC must be defined with CFG_PRINTF_USBCDC"
    #endif

    #if defined(CFG_USB_MSC) && !defined(CFG_SDCARD)
      #error "CFG_USB_MSC must be defined with CFG_SDCARD"
    #endif

    #if defined(CFG_PROTOCOL)
      #if defined(CFG_PROTOCOL_VIA_HID) && !defined(CFG_USB_HID_GENERIC)
        #error "CFG_PROTOCOL_VIA_HID requires CFG_USB_HID_GENERIC"
      #endif

      #if defined(CFG_PROTOCOL_VIA_BULK) && !defined(CFG_USB_CUSTOM_CLASS)
        #error "CFG_PROTOCOL_VIA_BULK requires CFG_USB_CUSTOM_CLASS to be defined"
      #endif
    #endif
/*=========================================================================*/

#ifdef __cplusplus
}
#endif

#endif
