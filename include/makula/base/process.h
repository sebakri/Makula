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
/**
 * \example example_process.cpp
 */

#ifndef PROCESS_H
#define PROCESS_H

#include "makula/base/channel.h"
#include <future>


namespace makula {
namespace base {

/**
 * \brief Abstract class to define a Process. Runs it's main-method asynchron.
 */
class Process {
public:
     virtual ~Process() {}

     /**
      * \brief start runing the main-method.
      */
     void execute() {
          fut = std::async ( std::launch::async, [this] {
               int retValue = std::move ( main() );

               return std::move ( retValue );
          } );
     }

     /**
      * \brief is waiting until main-method is finished.
      * \return return-value of the main-mehtod.
      */
     int wait_until_finished() {
          fut.wait();
          return fut.get();
     }

private:
     std::future<int> fut;
protected:
     /**
      * \brief main-method to implement. Will executed asynchonous.
      */
     virtual int main() = 0;
};

}
}

#endif //PROCESS_H
