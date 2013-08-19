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
 * \example example_buffer.cpp
 */

#ifndef BUFFER_H
#define BUFFER_H

/**
 * Necesary for using abort()
 */
#define TBB_USE_EXCEPTIONS 1

#include <tbb/concurrent_queue.h>
#include <memory>
#include <atomic>

namespace makula {
namespace base {

/**
 * \brief A concurrent Buffer based on TBBs concurrent_bounded_queue.
 *
 * \param T type to store in Buffer.
 */
template<typename T>
class Buffer {
public:
     /**
      * \brief pointer type
      */
     using Ptr = std::unique_ptr<Buffer<T> >;

     /**
      * \brief shared pointer type
      */
     using SharedPtr = std::shared_ptr<Buffer<T> >;

     /**
      * \brief  default constructor
      */
     Buffer ( uint capacity = 1 ) {
          queue.set_capacity ( capacity );
     }

     /**
      * \brief destructor
      */
     ~Buffer() {
          try {
               queue.abort();
          } catch ( tbb::user_abort &e ) {

          }
     }

     /**
      * \brief copy constructor
      */
     Buffer ( const Buffer<T>& b ) :
          queue ( b.queue ) {
     }

     /**
      * \brief move constructor
      */
     Buffer ( const Buffer<T>&& b ) :
          queue ( std::move ( b.queue ) ) {
     }

     /**
      * \brief push an element to the buffer.
      * \param element element to push.
      */
     const bool push ( const T& element ) {
          try {
               queue.push ( element );
               return true;
          } catch ( tbb::user_abort &e ) {
               return false;
          }
     }

     /**
      * \brief pop the next element from buffer.
      * \param[in,out] element of type T.
      */
     const bool pop ( T& element ) {
          try {
               queue.pop ( element );
               return true;
          } catch ( tbb::user_abort &e ) {
               return false;
          }
     }

     /**
      * \brief get the max count of elements.
      * \return max count of elements.
      */
     const uint capacity() {
          return queue.capacity();
     }

     /**
      * \brief get the current element count.
      * \return current element count.
     */
     const uint size() {
          return queue.size();
     }

     /**
      * \brief check if buffer is empty
      * \return true if empty.
      */
     const bool empty() const {
          return queue.empty();
     }

     /**
      * \brief Forget all the waiting threads.
      */
     void abort() {
          queue.abort();
     }

     /**
      * \brief set the capacity of the buffer
      * \param[in] capacity the new capacity
      */
     void setCapacity ( const uint& capacity ) {
          queue.set_capacity ( capacity );
     }

protected:
     /**
      * \brief a concurrent bounded queue.
      */
     tbb::concurrent_bounded_queue<T> queue;
};

}
}

#endif // PROCESS_H
