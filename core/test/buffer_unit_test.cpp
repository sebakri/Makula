// Copyright (c) 2013 Sebastian Krieger

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "base/buffer.h"
#include <gtest/gtest.h>
#include <future>

using namespace makula::core;

class BufferUnitTest : public ::testing::Test {
protected:

     Buffer<int>::Ptr b;

     BufferUnitTest()
     {}

     ~BufferUnitTest()
     {}

     virtual void SetUp() {
          b = std::unique_ptr<Buffer<int> > ( new Buffer<int> ( 10 ) );
     }

     virtual void TearUp() {
     }
};

/// \brief Test the instantiation of a buffer.
TEST_F ( BufferUnitTest, CreateNewBuffer )
{
     // check if the created buffer has a capacity of 10...
     EXPECT_EQ ( b->capacity(), 10 );

     // ...and check if it's empty.
     EXPECT_EQ ( b->empty(), true );
}

/// \brief Test if buffer is thread-safe with elements count = capacity.
TEST_F ( BufferUnitTest, testThreadSafetyWithMaxCapacity )
{
     // start a thread, which is reading from buffer.
     auto f2 = std::async ( [this] {
          for ( int i = 0; i < b->capacity(); i++ ) {
               int v = b->pop();
          }
          return 0;
     } );

     // start a thread, which is writing to buffer.
     auto f1 = std::async ( [this] {
          for ( int i = 0; i < b->capacity(); i++ ) {
               b->push ( i );
          }
          return 0;
     } );

     // Check if both threads are finished correctly.
     EXPECT_EQ ( f1.get(), 0 );
     EXPECT_EQ ( f2.get(), 0 );

     // Check if buffer is empty.
     EXPECT_EQ ( b->empty(), true );
}


/// \brief Test if buffer is thread-safe with elements count = 2 * capacity.
TEST_F ( BufferUnitTest, testThreadSafetyWithTwentyElements )
{
     // start thread for reading the buffer.
     auto f2 = std::async ( std::launch::async, [this] {
          for ( int i = 0; i < 2 * b->capacity(); i++ ) {
               int v = b->pop();
          }
          return 0;
     } );

     // start thread for writing the buffer.
     auto f1 = std::async ( std::launch::async, [this] {
          for ( int i = 0; i < 2 * b->capacity(); i++ ) {
               b->push ( i );
          }
          return 0;
     } );

     // Check if threads are finished and everything is ok.
     EXPECT_EQ ( f1.get(), 0 );
     EXPECT_EQ ( f2.get(), 0 );

     // Check if buffer is empty.
     EXPECT_EQ ( b->empty(), true );
}