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


#ifndef CHANNEL_H
#define CHANNEL_H

#include "makula/base/buffer.h"

namespace makula {
namespace base {

/**
*	\brief A thread safe buffered channel.
*/
template<typename MessageType>
class Channel {
public:
     /// \brief point type
     using Ptr = std::unique_ptr<Channel<MessageType> >;
     /// \brief shared point type
     using SharedPtr = std::shared_ptr<Channel<MessageType> >;

     /// \brief default constructor
     Channel ( uint maxElementCount = 1 ) :
          maxElementCount ( maxElementCount ),
          buffer ( maxElementCount ) {

     }

     /// \brief destructor
     ~Channel() {

     }

     /// \brief move constructor
     Channel ( const Channel<MessageType>&& c ) {
          this->buffer = std::move ( c.buffer );
     }

     /**
     *	\brief operator for sending data.
     *	\param[in] in data to send.
     */
     void operator<< ( const MessageType& data ) {
          send ( data );
     }

     /**
     *	\brief operator for reading data.
     *	\param[out] out variable to store data.
     */
     void operator>> ( MessageType& data ) {
          data = std::move ( read() );
     }

     /**
      * \brief reading data from channel.
      * \return next data from channel.
      */
     MessageType read() {
          return std::move ( buffer.pop() );
     }

     /**
      * \brief send data over the channel.
      * \param[in] data data to send.
      */
     void send ( const MessageType& data ) {
          buffer.push ( data );
     }

     /**
     *	\brief get current element count.
     *	\return element count.
     */
     uint size() {
          return buffer.size();
     }

     /**
      * \brief get the capacity of the channel.
      * \return capacity
      */
     uint capacity() {
          return buffer.capacity();
     }


protected:
     Buffer<MessageType> buffer;
     std::atomic_uint maxElementCount;
};

}
}

#endif // CHANNEL_H
