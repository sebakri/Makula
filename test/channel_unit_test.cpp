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

#include "makula/base/channel.h"
#include <gtest/gtest.h>
#include <future>

using namespace makula::base;

// Channel
class ChannelUnitTest : public ::testing::Test {
protected:
     Channel<int>::Ptr c;

     ChannelUnitTest()
     {}

     ~ChannelUnitTest()
     {}

     virtual void SetUp() {
          c = Channel<int>::Ptr ( new Channel<int> ( 100 ) );
     }

     virtual void TearDown()
     {}
};

TEST_F ( ChannelUnitTest, CreateNewChannel )
{
     EXPECT_EQ ( c->size(), 0 );
}

TEST_F ( ChannelUnitTest, SendAndRead )
{
     c->send ( 5 );
     int i = c->read();
     EXPECT_EQ ( i,5 );
}

TEST_F ( ChannelUnitTest, SendWhenBufferIsFull )
{
     // send until buffer's capacity is reached
     for ( int i = 0; i < c->capacity(); i++ ) {
          c->send ( i );
     }

     // another thread is sending data...
     auto f = std::async ( std::launch::async, [this] {
          c->send ( 11 );
     } );

     //  ...but capacity is reached...so the thread should wait.
     EXPECT_EQ ( c->size(), c->capacity() );

     // reading data from channel, so the waiting thread can continue.
     c->read();

     // wait for the the sending thread is finished.
     f.wait();

     // the capacity should be reached again.
     EXPECT_EQ ( c->size(), c->capacity() );
}

TEST_F ( ChannelUnitTest, ReadWhenBufferIsEmpty )
{
     auto f = std::async ( std::launch::async, [this] {
          return c->read();
     } );

     c->send ( 42 );

     EXPECT_EQ ( f.get(), 42 );
}

TEST_F ( ChannelUnitTest, IsItThreadSafe )
{
     auto f1 = std::async ( std::launch::async, [this]() {
          for ( int i = 0; i < c->capacity(); i++ ) {
               c->send ( i );
          }

          return 0;
     } );

     auto f2 =  std::async ( std::launch::async, [this]() {
          for ( int i = 0; i < c->capacity(); i++ ) {
               int v = c->read();
          }
          return 0;
     } );

     EXPECT_EQ ( f1.get(), 0 );
     EXPECT_EQ ( f2.get(), 0 );

     EXPECT_EQ ( c->size(), 0 );
}
