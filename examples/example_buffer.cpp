#include <makula/base/buffer.h>
#include <future>

using namespace makula::base;

int main(int argc, char** argv)
{
     /*
      * Create a new buffer with capacity = 20.
      */
     auto b = Buffer<int>(20);
     
     /*
      * Start a task writing 40 elements, ...
      */
     auto f1 = std::async(std::launch::async, [&b] {
        for(int i = 0; i < 40; i++)
        {
             b.push(i);
        }
     });
     
     /*
      * ...a task reading 20 elements ...
      */
     auto f2 = std::async(std::launch::async, [&b] {
        for(int i = 0; i < 20; i++)
        {
             int v = b.pop();
        }
     });
     
     /*
      * and another task reading also 20 elements.
      */
     auto f3 = std::async(std::launch::async, [&b] {
        for(int i = 0; i < 20; i++)
        {
             int v = b.pop();
        }
     });
     
     /*
      * wait until all tasks finished.
      */
     f1.wait();
     f2.wait();
     f3.wait();
     
     return 0;
}