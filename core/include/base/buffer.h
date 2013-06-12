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


#ifndef BUFFER_H
#define BUFFER_H

#include <tbb/concurrent_queue.h>
#include <memory>
#include <atomic>

namespace makula {
namespace core {

/**
*	\brief A thread-safe buffer.
*	\param T type to store in Buffer.
*	\param N max size for Buffer.
*/
template<typename T>
class Buffer {
public:
     /// \brief pointer type
     using Ptr = std::unique_ptr<Buffer<T> >;

     /// \brief shared pointer type
     using SharedPtr = std::shared_ptr<Buffer<T> >;

     /// \brief  default constructor
     Buffer ( uint capacity = 1 ) :
          maxElementCount ( capacity ) {
          queue.set_capacity ( capacity );
     }

     /// \brief destructor
     ~Buffer() {}

     /// \brief copy constructor
     Buffer ( const Buffer<T>& b ) :
          queue ( b.queue ),
          maxElementCount ( b.maxElementCount ) {

     }

     /// \brief move constructor
     Buffer ( const Buffer<T>&& b ) :
          queue ( std::move ( b.queue ) ),
          maxElementCount ( std::move ( b.maxElementCount ) ) {
     }

     /**
     *	\brief push an element to the buffer.
     *	\param element element to push.
     */
     void push ( const T& element ) {
          queue.push ( element );
     }

     /**
     *	\brief pop the next element from buffer.
     *	\return element of type T.
     */
     T pop() {
          T element;
          queue.pop ( element );
          return std::move ( element );
     }

     /**
     *	\brief get the max count of elements.
     *	\return max count of elements.
     */
     uint capacity() {
          return queue.capacity();
     }

     /**
     *	\brief get the current element count.
     *	\return current element count.
     */
     uint size() {
          return queue.size();
     }

     /**
     *	\brief check if buffer is empty
     *  \return true if empty.
     */
     bool empty() const {
          return queue.empty();
     }

protected:
     tbb::concurrent_bounded_queue<T> queue;
     std::atomic_uint maxElementCount;
};

}
}

#endif // PROCESS_H
