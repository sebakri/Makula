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

#ifndef CONSUMER_H
#define CONSUMER_H

#include "makula/base/process.h"
#include "makula/base/channel.h"

namespace makula {
namespace base {

/**
 * \brief Abstract class Consumer can receive data from a Producer.
 *
 * \param InputT type to receive
 */
template<typename InputT>
class Consumer {
public:
     virtual ~Consumer() {}

     /**
      * \brief set the input channel.
      * \param[in] ch channel pointer to set as input channel.
      */
     void setInputChannel ( typename Channel<InputT>::SharedPtr ch ) {
          chin = ch;
     }
private:
     typename Channel<InputT>::SharedPtr chin;
protected:

     /**
      * \brief takes on data from channel.
      * \param[in,out] data stores the read data.
      * \return false if channel is not connected.
      */
     bool takeOn ( InputT& data ) {
          if ( chin )
               data = chin->read();
          else
               return false;

          return true;
     }
};

}
}

#endif //CONSUMER_H
