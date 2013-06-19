#Makula#

At the beginning Makula was planned as an object-recognition framework. Finally it becomes a
High-Level Library for task-based parallel programming.

## Base ##

The Base contains the following parts:

- [x] a concurrent buffer for writing and reading data from multiple threads.
- [x] a concurrent channel for communicating between threads.
- [x] multiple abstract classes to define process and how they communicate with eachother.
- [x] a workerpool for a higher throughput.

## Requirements ##

- Clone the latest PCL and install it.

> git clone https://github.com/PointCloudLibrary/pcl.git

## How to build ##

- Clone it from Github

> git clone https://github.com/Seppone/Makula.git

- Build it!

> mkdir build
>
> cd build
>
> cmake ..
>
> make && make run_makula_tests

- THIS STEP REQUIRES DOXYGEN AND GRAPHVIZ. If you want some documentation, then run

> make doc

## Copyright ##

Copyright (c) 2013 Sebastian Krieger "sebastian-krieger@online.de"

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
