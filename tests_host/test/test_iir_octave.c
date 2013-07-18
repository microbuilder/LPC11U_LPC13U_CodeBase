/**************************************************************************/
/*!
    @file     test_iir_octave.c
    @ingroup  Unit Tests

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
#include "unity.h"
#include "iir.h"

float32_t i, filtered;

void setUp(void)
{
  i = filtered = 0.0F;
}

void tearDown(void)
{
}

void test_iir_1p_init(void)
{
  /* Configure the IIR filter */
  iir_filt_1p_instance iir =  { .a1=0.1F,
                                .b0=0.9F, .b1=0.9F };

  /* Calculate comparison data in Octave:
   * a = [1.0 0.1]
   * b = [0.9 0.9]
   * data = [12.345 12.345 12.345 12.345 12.345]
   * results = filter(b,a,data)
   * => 11.111   21.110   20.110   20.210   20.200
   */

  /* Push some data into the filter */
  filtered = iir_filt_1p(&iir, 12.345F);
  TEST_ASSERT_EQUAL_FLOAT(11.1105F, filtered);
  filtered = iir_filt_1p(&iir, 12.345F);
  TEST_ASSERT_EQUAL_FLOAT(21.11F, filtered);
  filtered = iir_filt_1p(&iir, 12.345F);
  TEST_ASSERT_EQUAL_FLOAT(20.11F, filtered);
  filtered = iir_filt_1p(&iir, 12.345F);
  TEST_ASSERT_EQUAL_FLOAT(20.21F, filtered);
  filtered = iir_filt_1p(&iir, 12.345F);
  TEST_ASSERT_EQUAL_FLOAT(20.2F, filtered);
}
