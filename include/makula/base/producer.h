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

#ifndef PRODUCER_H
#define PRODUCER_H

#include <makula/base/channel.h>

namespace makula {
namespace base {

/**
 * \brief Abstract class to define a Producer. A Producer can send data to a Consumer over an channel
 * \param OutputT type to send.
 */
template<typename OutputT>
class Producer {
public:
     virtual ~Producer() {}

     /**
      * \brief set the output channel.
      * \param[in] c channel pointer to set as output channel.
      */
     void setOutputChannel ( typename Channel<OutputT>::SharedPtr c ) {
          chout = c;
     }

     /**
      * \brief get the output channel.
      * \return a const pointer to the output channel.
      */
     const typename Channel<OutputT>::Ptr getOutputChannel() {
          return chout;
     }
protected:

     /**
      * \brief send data to connected Consumer
      * \param[in] data data to send.
      * \return false if Producer is not connected.
      */
     const bool deliver ( const OutputT& data ) {
          if ( chout )
               return chout->send ( data );
          else
               return false;
     }

     /**
      * \brief stops producing data.
      */
     void stopProducing() {
          if ( chout )
               chout->stop();

          chout.reset();
     }

private:
     typename Channel<OutputT>::SharedPtr chout;
};

}
}

#endif // PRODUCER_H
