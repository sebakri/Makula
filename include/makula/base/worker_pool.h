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


#ifndef WORKERPOOL_H
#define WORKERPOOL_H

#include <atomic>
#include <future>
#include <vector>
#include <functional>
#include "channel.h"

namespace makula {
namespace base {

/**
 * \brief WorkerPool can run a specific task asynchronously in seperate workers.
 * \param Args Argmuments for the task to run.
 */
template<typename... Args>
class WorkerPool {
public:
     /**
      * \brief pointer type
      */
     using Ptr = std::unique_ptr<WorkerPool<Args...> >;

     /**
      * \brief shared pointer type
      */
     using SharedPtr = std::shared_ptr<WorkerPool<Args...> >;

     /**
      * \brief default constructor
      */
     WorkerPool ( const std::function<void ( Args... ) >& f, uint maxWorkerCount = std::thread::hardware_concurrency() ) :
          workerFunction ( std::move ( f ) ),
          workerCounter ( 0 ),
          maxWorkerCount ( maxWorkerCount ) {
     }

     /**
      * \brief move constructor
      */
     WorkerPool ( const WorkerPool& w ) :
          workerFunction ( std::move ( w.workerFunction ) ),
          workerCounter ( std::move ( w.workerCounter.load() ) ),
          maxWorkerCount ( std::move ( w.maxWorkerCount.load() ) ) {

     }

     /**
      * \brief destructor
      */
     virtual ~WorkerPool() {};

     /**
      * \brief get the current worker count
      * \return number of current workers.
      */
     uint currentWorkerCount() {
          return workerCounter;
     }

     /**
      * \brief start a new worker with specific arguments
      */
     void startWorker ( Args... args ) {
          wait_for_free_slots();

          workerCounter++;
          auto f = std::async ( std::launch::async, [this] ( Args... args1 ) {

               workerFunction ( args1... );

               std::lock_guard<std::mutex> lock ( mtx );
               workerCounter--;
               maxWorkersReached = false;
               cv.notify_one();
          }, args... );

          futures.push_back ( std::move ( f ) );

          if ( workerCounter == maxWorkerCount )
               maxWorkersReached = true;
     }

     /**
      * wait until all workers finished
      */
     void wait() {
          for ( auto& f : futures ) {
               f.wait();
          }
     }


private:
     std::function<void ( Args... ) > workerFunction;
     std::vector<std::future<void> > futures;
     std::atomic_uint workerCounter;
     std::atomic_uint maxWorkerCount;
     bool maxWorkersReached = false;
     std::mutex mtx;
     std::condition_variable_any cv;


     void wait_for_free_slots() {
          while ( maxWorkersReached ) {
               std::unique_lock<std::mutex> lock ( mtx );
               cv.wait ( lock );
          }
     }
};

}
}

#endif // WORKERPOOL_H
