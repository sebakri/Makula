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
 * \example example_channel.cpp
 */

#ifndef CHANNEL_H
#define CHANNEL_H

#include <makula/base/buffer.h>

namespace makula {
namespace base {

/**
 * \brief A concurrent, buffered Channel. Easily exchange data between threads.
 *
 * \param MessageType Type to store in Channel.
 */
template<typename MessageType>
class Channel {
public:
     /**
      * \brief pointer type
      */
     using Ptr = std::unique_ptr<Channel<MessageType> >;
     /**
      * \brief shared point type
      */
     using SharedPtr = std::shared_ptr<Channel<MessageType> >;

     /**
      * \brief default constructor
      */
     Channel ( uint maxElementCount = 1 ) :
          buffer ( maxElementCount ) ,
          maxElementCount ( maxElementCount ) {

     }

     /**
      * \brief destructor
      */
     ~Channel() {
          stop();
     }

     /**
      * \brief move constructor
      */
     Channel ( const Channel<MessageType>&& c ) :
          buffer ( std::move ( c.buffer ) ) {
     }

     /**
      * \brief operator for sending data.
      * \param[in] data data to send.
      */
     void operator<< ( const MessageType& data ) {
          send ( data );
     }

     /**
      * \brief operator for reading data.
      * \param[out] data variable to store data.
      */
     void operator>> ( MessageType& data ) {
          read(data);
     }

     /**
      * \brief reading data from channel.
      * \return next data from channel.
      */
     const bool read(MessageType& m) {
          return std::move ( buffer.pop ( m ) );
     }

     /**
      * \brief send data over the channel.
      * \param[in] data data to send.
      */
     const bool send ( const MessageType& data ) {
          return buffer.push ( data );
     }

     /**
      * \brief get current element count.
      * \return element count.
      */
     const uint size() {
          return buffer.size();
     }

     /**
      * \brief get the capacity of the channel.
      * \return capacity
      */
     const uint capacity() {
          return buffer.capacity();
     }

     /**
      * \brief cancels all operations.
      */
     void stop() {
          buffer.abort();
     }
protected:

     /**
      * \brief the buffer stores the data.
      */
     Buffer<MessageType> buffer;

     /**
      * \brief holds the element count.
      */
     std::atomic_uint maxElementCount;
};

}
}

#endif // CHANNEL_H
