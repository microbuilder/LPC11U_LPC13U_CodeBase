/**************************************************************************/
/*!
    @file     asserts.h
    @author   K. Townsend (microBuilder.eu)

    @brief    Various static and dynamic ASSERT macros to simplify error
              checking, and centralise error logging when debugging.
    @ingroup  Errors

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2012, K. Townsend
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
#ifndef _ASSERTS_H_
#define _ASSERTS_H_

#ifdef __cplusplus
extern "C" {
#endif

#if defined DEBUG
  #define _PRINTF(...)      printf(__VA_ARGS__)
#else
  #define _PRINTF(...)
#endif

/*! Compiler specific macro returning a string containing the current line number */
#define ASSERT_LINE __LINE__
/*! Compiler specific macro returning a integer containing the current file name */
#define ASSERT_FILE __FILE__
/*! Compiler specific macro returning a string containing the current function */
#define ASSERT_FUNC __func__

#define STRING_CONCAT(a, b) a##b                  // TODO: Move to another place
#define XSTRING_CONCAT(a, b) STRING_CONCAT(a, b)  // Expand then concat ... TODO: Move to another place

/**************************************************************************/
/*!
    @brief  This macro will assert the test condition and return the
            specified returnValue, as well as the specified 'message'
            string in the \ref ASSERT output

    @code
    // Make sure 'addr' is within range
    ASSERT(addr <= MAX_ADDR, ERROR_ADDRESSOUTOFRANGE, "Invalid address");
    @endcode
*/
/**************************************************************************/
#define ASSERT_MESSAGE(condition, returnValue, message) \
        do{\
          if (!(condition)) {\
            _PRINTF("Assert: %s at line %d: %s%s", ASSERT_FUNC, ASSERT_LINE, message, CFG_PRINTF_NEWLINE);\
            return (returnValue);\
          }\
        }while(0)

/**************************************************************************/
/*!
    @brief Checks the condition, and if the assert fails the supplied
           returnValue will be returned in the calling function.

    @code
    // Make sure 'addr' is within range
    ASSERT(addr <= MAX_ADDR, ERROR_ADDRESSOUTOFRANGE);
    @endcode
*/
/**************************************************************************/
#define ASSERT(condition, returnValue)  ASSERT_MESSAGE(condition, returnValue, NULL)

/**************************************************************************/
/*!
    @brief  Checks the supplied \ref err_t value (sts), and if it is
            not equal to \ref ERROR_NONE the sts value will be returned
            and the supplied error message will be sent via _PRINTF.

    @details

    This macro is useful to check if a function returned an error without
    bloating your own code with endless "if (error) {...}".

    @code
    // If anything other than ERROR_NONE is returned by tsl2561Init()
    // this macro will log the error as well as the optional message,
    // and exit the function returning the err_t value.
    ASSERT_STATUS(tsl2561Init(), "Initialisation failed!");
    @endcode
*/
/**************************************************************************/
#define ASSERT_STATUS_MESSAGE(sts, message) \
        do{\
          err_t status = (sts);\
          if (ERROR_NONE != status) {\
            _PRINTF("Assert: %s at line %d: 0x%X %s%s", ASSERT_FUNC, ASSERT_LINE, (uint32_t) status, message, CFG_PRINTF_NEWLINE);\
            return status;\
          }\
        } while(0)

/**************************************************************************/
/*!
    @brief  Checks the supplied \ref err_t value (sts), and if it is
            not equal to \ref ERROR_NONE the sts value will be returned.

    @details
    This macro is useful to check if a function returned an error without
    bloating your own code with endless "if (error) {...}".

    @code
    // If anything other than ERROR_NONE is returned by tsl2561Enable()
    // this macro will log the error and exit the function returning the
    // err_t value.
    ASSERT_STATUS(tsl2561Enable());
    @endcode
*/
/**************************************************************************/
#define ASSERT_STATUS(sts)                ASSERT_STATUS_MESSAGE(sts, NULL)

/**************************************************************************/
/*!
    @brief  Checks the supplied condition using a static assert

    @details
    The ASSERT_xxx macros are for dynamic assertions that can provide
    protection against unexpected conditions encountered at runtime. An
    even stronger check can be provided by static assertions that can be
    evaluated by the compiler at the time code is compiled. A static
    assertion can be defined like the ASSERT_xxx macros, but can be used
    standalone (i.e., not in a conditional), for instance as follows:

    @code
    // This assertion will trigger an error when the code is compiled
    // on 32-bit machines.
    STATIC_ASSERT(sizeof(void *) != 4);

    // To check the opposite requirement, i.e., to make sure that we are
    // executing on a 32-bit machine only, the following static assertion
    // can be used, which will trigger an error when the code is compiled
    // on machines that do not have a 32-bit wordsize:
    STATIC_ASSERT(sizeof(void *) == 4);
    @endcode
*/
/**************************************************************************/
#define STATIC_ASSERT(const_expr) enum { XSTRING_CONCAT(static_assert_, __LINE__) = 1/(!!(const_expr)) }

#ifdef __cplusplus
}
#endif

#endif
