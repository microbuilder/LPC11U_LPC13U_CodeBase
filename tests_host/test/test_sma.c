/**************************************************************************/
/*!
    @file     test_sma.c
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
#include "sma_f.h"

/* Declare a data buffer */
float sma_buffer[8];

void setUp(void)
{
}

void tearDown(void)
{
}

void test_sma_f_init(void)
{
  /* Declare the filter */
  sma_f_t sma = { .k = 0,
                  .size = 8,
                  .avg = 0,
                  .buffer = sma_buffer };

  /* Initialise the moving average filter */
  TEST_ASSERT_FALSE(sma_f_init(&sma))
  TEST_ASSERT_EQUAL_UINT32(8, sma.size);
  TEST_ASSERT_EQUAL_UINT32(0, sma.k);
}

void test_sma_f_add(void)
{
  /* Declare the filter */
  sma_f_t sma = { .k = 0,
                  .size = 8,
                  .avg = 0,
                  .buffer = sma_buffer };

  /* Initialise the moving average filter */
  sma_f_init(&sma);

  /* Add some values */
  sma_f_add(&sma, 1.0);
  TEST_ASSERT_EQUAL_FLOAT(0.0F, sma.avg);
  TEST_ASSERT_EQUAL_UINT32(1, sma.k);
  sma_f_add(&sma, 2.1);
  TEST_ASSERT_EQUAL_FLOAT(0.0F, sma.avg);
  TEST_ASSERT_EQUAL_UINT32(2, sma.k);
  sma_f_add(&sma, -30.2);
  TEST_ASSERT_EQUAL_FLOAT(0.0F, sma.avg);
  TEST_ASSERT_EQUAL_UINT32(3, sma.k);
  sma_f_add(&sma, -35.3);
  TEST_ASSERT_EQUAL_FLOAT(0.0F, sma.avg);
  TEST_ASSERT_EQUAL_UINT32(4, sma.k);
  sma_f_add(&sma, 11.4);
  TEST_ASSERT_EQUAL_FLOAT(0.0F, sma.avg);
  TEST_ASSERT_EQUAL_UINT32(5, sma.k);
  sma_f_add(&sma, 35.5);
  TEST_ASSERT_EQUAL_FLOAT(0.0F, sma.avg);
  TEST_ASSERT_EQUAL_UINT32(6, sma.k);
  sma_f_add(&sma, 30.6);
  TEST_ASSERT_EQUAL_FLOAT(0.0F, sma.avg);
  TEST_ASSERT_EQUAL_UINT32(7, sma.k);
  /* We should have an avg value starting here */
  sma_f_add(&sma, 20.7);
  TEST_ASSERT_EQUAL_FLOAT(4.475F, sma.avg);
  TEST_ASSERT_EQUAL_UINT32(8, sma.k);
  sma_f_add(&sma, 3.8);
  TEST_ASSERT_EQUAL_FLOAT(4.825F, sma.avg);
  TEST_ASSERT_EQUAL_UINT32(9, sma.k);
}
