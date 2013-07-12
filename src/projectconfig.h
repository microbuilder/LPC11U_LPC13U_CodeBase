/**************************************************************************/
/*!
    @file     projectconfig.h
    @author   K. Townsend (microBuilder.eu)

    @brief    Indicates which board should be used during the build process
    @ingroup  Board/HW Abstration Layer

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2012, 2013 K. Townsend
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
#ifndef _PROJECTCONFIG_H_
#define _PROJECTCONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
    ABOUT THIS FILE
    -----------------------------------------------------------------------
    This file contains global config settings that apply to any board
    or project.

    Any board-specific config settings should be placed in individual
    config files in the 'boards/' sub-directory, and the settings will be
    includes in any file via 'projectconfig.h'.
   ========================================================================*/

#include "sysdefs.h"
#include "errors.h"
#include "asserts.h"
#include "binary.h"
#include "localisation/localisation.h"

/*=========================================================================
    CODE BASE VERSION SETTINGS

    Please do not modify this version number.  To set a version number
    for your project or firmware, change the values in your 'boards/'
    config file.
    -----------------------------------------------------------------------*/
    #define CFG_CODEBASE_VERSION_MAJOR      (0)
    #define CFG_CODEBASE_VERSION_MINOR      (9)
    #define CFG_CODEBASE_VERSION_REVISION   (0)
/*=========================================================================*/


/*=========================================================================
    BOARD SELECTION

    Because several boards use this code library with sometimes slightly
    different pin configuration, you will need to specify which board you
    are using by enabling one of the following definitions. The code base
    will then try to configure itself accordingly for that board.

    GNU Toolchain
    -------------
    If you are using make, the target board is specified in the makefile

    Crossworks for ARM
    ------------------
    The board is selected as an additional defined in the solution
    properties window, in the 'Common' group under 'Preprocessor Options >
    Preprocessor Definitions'

    LPCXpresso/Red Suite
    --------------------
    Each board has it's own 'Build Configuration', which can be accessed
    by right-clicking on your project, and selecting the 'Build
    Configurations > Set Active ... > BUILD_CONFIG' menu

    Keil uVision
    ------------
    Select a board via the project 'Options' dialogue box, in the C/C++
    tab, entering an appropriate value in the 'Define' textbox

    -----------------------------------------------------------------------*/
    #if defined(CFG_BRD_LPCXPRESSO_LPC1347)
      #include "boards/lpcxpresso1347/board_lpcxpresso1347.h"
    #elif defined(CFG_BRD_RF1GHZNODE)
       #include "boards/rf1ghznode/board_rf1ghznode.h"
    #elif defined(CFG_BRD_RF1GHZUSB)
      #include "boards/rf1ghzusb/board_rf1ghzusb.h"
    #elif defined (CFG_BRD_LPCNFC)
      #include "boards/lpcnfc/board_lpcnfc.h"
    #elif defined (CFG_BRD_LPCSTEPPER)
      #include "boards/lpcstepper/board_lpcstepper.h"
    #elif defined (CFG_BRD_SIMULATOR)
      #include "boards/simulator/board_simulator.h"
    #else
      #error "No CFG_BRD_* has been defined"
    #endif
/*=========================================================================*/

#ifdef __cplusplus
}
#endif

#endif
