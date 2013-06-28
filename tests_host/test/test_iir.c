/**************************************************************************/
/*!
    @file     test_iir.c
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
#include <string.h>
#include "unity.h"
#include "iir_f.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_iir_f_init(void)
{
  iir_f_t iir;

  /* Initialise IIR filter with an alpha of -0.01 */
  /* Alpha should be adjust to 0 */
  iir_f_init(&iir, -0.01F);
  TEST_ASSERT_EQUAL_FLOAT(0.0F, iir.alpha);

  /* Initialise IIR filter with an alpha of 1.1 */
  /* Alpha should be adjust to 1.0F */
  iir_f_init(&iir, 1.1F);
  TEST_ASSERT_EQUAL_FLOAT(1.0F, iir.alpha);

  /* Initialise IIR filter with an alpha of 0.01 */
  iir_f_init(&iir, 0.01);
  TEST_ASSERT_EQUAL_FLOAT(0.01F, iir.alpha);
  TEST_ASSERT_EQUAL_FLOAT(0.0F, iir.avg);
  TEST_ASSERT_EQUAL(0, iir.k);
}

void test_iir_f_add(void)
{
  iir_f_t iir;

  /* Initialise IIR filter with an alpha of 0.01 */
  iir_f_init(&iir, 0.01);

  /* Add a sample */
  iir_f_add(&iir, 10);
  TEST_ASSERT_EQUAL(1, iir.k);
  TEST_ASSERT_EQUAL_FLOAT(10.0F, iir.avg);

  /* Add three more samples */
  iir_f_add(&iir, 11);
  iir_f_add(&iir, 12);
  iir_f_add(&iir, 13);
  TEST_ASSERT_EQUAL(4, iir.k);
}

void test_iir_f_avg(void)
{
  iir_f_t iir;

  /* Initialise IIR filter with an alpha of 0.01 */
  iir_f_init(&iir, 0.01);

  /* Add some samples */
  iir_f_add(&iir, 10);
  iir_f_add(&iir, 20);
  iir_f_add(&iir, 30);
  iir_f_add(&iir, 35);

  /* With an alpha of 0.01 avg should be 10.546 */
  TEST_ASSERT_EQUAL_FLOAT(10.546F, iir.avg);
}
