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

#include "makula/base/producer.h"
#include "makula/base/consumer.h"
#include "makula/base/process.h"
#include <gtest/gtest.h>

using namespace makula::base;

class ProducerImpl : public Process, public Producer<int> {
public:
     ProducerImpl() {};
     ~ProducerImpl() {};

protected:
     bool quit;

     virtual int main() override {
          for ( int i = 0; i < 10; i++ ) {
               deliver ( i );
          }
          return 0;
     }
};

class Consumer1Impl : public Process, public Consumer<int> {
public:
     Consumer1Impl() {};
     ~Consumer1Impl() {};

protected:
     bool quit;

     virtual int main() override {
          for ( int i = 0; i < 10; i++ ) {
               int v; takeOn(v);
          }
          return 0;
     }
};

class Consumer2Impl : public Process, public Consumer<int> {
public:
     Consumer2Impl() {};
     ~Consumer2Impl() {};

protected:
     bool quit;

     virtual int main() override {
          for ( int i = 0; i < 10; i++ ) {
               int v; takeOn(v);
               if ( v != i + 10 )
                    return 1;
          }
          return 0;
     }
};
class PipeImpl : public Process, public Producer<int>, public Consumer<int>{
public:
     PipeImpl() {}
     ~PipeImpl() {};

     virtual int main() override {
          for ( int i = 0; i < 10; i++ ) {
               int v; takeOn(v);
               v += 10;
               deliver(v);
          }

          return 0;
     }
};

template<typename T>
static void connect ( Producer<T>& p, Consumer<T>& c )
{
     auto ch = typename Channel<T>::SharedPtr ( new Channel<T> ( 10 ) );
     p.setOutputChannel ( ch );
     c.setInputChannel ( ch );
}

class ProcessUnitTest : public ::testing::Test {
protected:
     ProcessUnitTest() {}
     ~ProcessUnitTest() {};

     virtual void SetUp() {

     }

     virtual void TearDown() {

     }
};

TEST_F ( ProcessUnitTest, testProducerConsumer )
{
     Consumer1Impl c;
     ProducerImpl p;

     connect ( p, c );

     p.execute();
     c.execute();

     EXPECT_EQ ( p.wait_until_finished(), 0 );
     EXPECT_EQ ( c.wait_until_finished(), 0 );
}

TEST_F ( ProcessUnitTest, testProducerConsumerOverPipe )
{
     Consumer2Impl c;
     ProducerImpl p;
     PipeImpl pipe;

     connect ( p, pipe );
     connect ( pipe, c );

     p.execute();
     c.execute();
     pipe.execute();

     EXPECT_EQ ( p.wait_until_finished(), 0 );
     EXPECT_EQ ( c.wait_until_finished(), 0 );
     EXPECT_EQ ( pipe.wait_until_finished(), 0 );
}
