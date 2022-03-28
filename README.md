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
