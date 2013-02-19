/**************************************************************************/
/*!
    @file     test_fifo.c
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
#include "unity_fixture.h"
#include "core/fifo/fifo.h"

//--------------------------------------------------------------------+
// GROUP fifo, start with Group Declare
//--------------------------------------------------------------------+
#define FIFO_SIZE 10
static fifo_t ff;
static uint8_t buffer[FIFO_SIZE];

TEST_GROUP(fifo); // Group declaration

TEST_SETUP(fifo) // called before each test in the group, to set up group environment
{
  fifo_init(&ff, buffer, FIFO_SIZE, 0, 0);
}
TEST_TEAR_DOWN(fifo) // called after a test in the group has been invoked to clean up
{
  memset(&ff, 0, sizeof(fifo_t));
}

TEST(fifo, create_null)
{
  memset(&ff, 0, sizeof(fifo_t)); // clear fifo to test null created
  TEST_ASSERT_FALSE( fifo_init(&ff, buffer, 0, 0, 0) );
  TEST_ASSERT_TRUE( fifo_init(&ff, buffer, 1, 0, 0) );
}

TEST(fifo, normal)
{
  uint8_t i;

  for(i=0; i < FIFO_SIZE; i++)
  {
    fifo_write(&ff, i);
  }

  for(i=0; i < FIFO_SIZE; i++)
  {
    uint8_t c;
    fifo_read(&ff, &c);
    TEST_ASSERT_EQUAL(i, c);
  }
}

TEST(fifo, is_empty)
{
  TEST_ASSERT_TRUE(fifo_isEmpty(&ff));
  fifo_write(&ff, 1);
  TEST_ASSERT_FALSE(fifo_isEmpty(&ff));
}

TEST(fifo, is_full)
{
  uint8_t i;

  TEST_ASSERT_FALSE(fifo_isFull(&ff));

  for(i=0; i < FIFO_SIZE; i++)
  {
    fifo_write(&ff, i);
  }

  TEST_ASSERT_TRUE(fifo_isFull(&ff));
}


//----- Group Runner required hand installed, all tests should be above -----
TEST_GROUP_RUNNER(fifo)
{
  RUN_TEST_CASE(fifo, create_null);
  RUN_TEST_CASE(fifo, normal);
  RUN_TEST_CASE(fifo, is_empty);
  RUN_TEST_CASE(fifo, is_full)
}
