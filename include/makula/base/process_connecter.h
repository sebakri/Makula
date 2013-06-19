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

#ifndef PROCESSCONNECTER_H
#define PROCESSCONNECTER_H

#include "makula/base/channel.h"
#include "makula/base/producer.h"
#include "makula/base/consumer.h"

namespace makula {
namespace base {

/**
 * \brief ProcessConnecter can connect a Producer with Consumer
 */
class ProcessConnecter {
public:
     /**
      * \brief connect a Producer with a Consumer
      * \param[in,out] p Producer-based Type
      * \param[in,out] c Consumer-based Type
      * \param[in] bufferSize size of channel's buffer
      */
     template<typename T>
     static void connect ( Producer<T>& p, Consumer<T>& c, uint bufferSize = 10 ) {
          auto ch = typename Channel<T>::SharedPtr ( new Channel<T> ( bufferSize ) );
          p.setOutputChannel ( ch );
          c.setInputChannel ( ch );
     }
};

}
}

#endif // PROCESSCONNECTER_H
