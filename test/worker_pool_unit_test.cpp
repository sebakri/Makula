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

#include <gtest/gtest.h>
#include "makula/base/worker_pool.h"

using namespace makula::base;

// class WorkerPoolImpl : public WorkerPool<const int&> {
// public:
//      WorkerPoolImpl() {}
//      ~WorkerPoolImpl() {}
// 
// protected:
//      virtual int workerFunc ( const int& i ) {
//           int j = i;
//           j++;
// 
//           return 0;
//      }
// };

class WorkerPoolUnitTest : public ::testing::Test {
protected:
     WorkerPoolUnitTest() {}
     ~WorkerPoolUnitTest() {}

     WorkerPool<const int&>::Ptr w;

     void response(const int& i)
     {
          int j = i;
          j++;
     }
     
     virtual void SetUp() {
          w = WorkerPool<const int&>::Ptr ( 
          new WorkerPool<const int&>([this](const int& i)
               {
                    response(i);
               })
          );
     }

     virtual void TearDown() {
          w.release();
     }
};

TEST_F ( WorkerPoolUnitTest, createWorkerPool )
{
     WorkerPool<const int&> w([this](const int& i){ response(i);});
}

TEST_F ( WorkerPoolUnitTest, startSomeWorkers )
{
     for ( int i = 0; i < 200; i++ ) {
          w->startWorker ( i );
     }
     
     w->wait();

     EXPECT_EQ ( w->currentWorkerCount(), 0 );
}

