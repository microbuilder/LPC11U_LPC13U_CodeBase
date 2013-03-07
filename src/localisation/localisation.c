/**************************************************************************/
/*!
    @file     localisation.c
    @author   K. Townsend (microBuilder.eu)

    @section DESCRIPTION

    Common, board-specific files for the AT86RF2xx wireless boards

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
#include "projectconfig.h"
#include "localisation.h"

#define LOCALE_EN(key, en) [CULTURE_EN][key]=en
#define LOCALE_FR(key, fr) [CULTURE_FR][key]=fr

/* Set the default culture/language */
static volatile culture_t _localisation_currentCulture = CULTURE_EN;

/* Make sure the language data files are in same order as culture_t! */
const char* localised_strings[CULTURE_COUNT][LOCALISATION_FINAL+1] =
{
  #include "localisation/locale_en.dat"
  #include "localisation/locale_fr.dat"
};

/**************************************************************************/
/*!
    @brief Returns the localised string corresponding to the specified
           'key' value.
*/
/**************************************************************************/
char* localisation_GetString(localisedTextKeys_t key)
{
  return (char*)localised_strings[_localisation_currentCulture][key];
}

/**************************************************************************/
/*!
    @brief Sets the current language, which will determine which lookup
           table is used when retrieving text, etc.
*/
/**************************************************************************/
void localisation_SetCulture(culture_t culture)
{
  _localisation_currentCulture = culture;
}
