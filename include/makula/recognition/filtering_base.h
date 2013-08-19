// Copyright (c) 2013 Sebastian Krieger <sebastian-krieger@online.de>

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

#ifndef FILTERING_H
#define FILTERING_H

// #include <makula/base/process.h>
#include <makula/base/consumer.h>
#include <makula/base/producer.h>
#include <makula/base/process.h>

using namespace makula::base;

namespace makula {
namespace recognition {

/**
 * \brief Baseclass for filtering process.
 */
template<typename InputT, typename OutputT>
class FilteringBase : public Process, public Producer<OutputT>, public Consumer<InputT> {
public:
     virtual ~FilteringBase() {}
};

}
}
#endif // FILTERING_H

