#include <makula/base/channel.h>
#include <future>
#include <iostream>

using namespace makula::base;

int main(int argc, char** argv)
{
     /*
      * Create a new string-channel...
      */
     Channel<std::string> c;
     
     /*
      * Start a task sending data to the channel...
      */
     auto f1 = std::async(std::launch::async, [&c]
     {
          c << "Hello";
     });
     
     /*
      * ...and a task reading data and manipulate it.
      */
     auto f2 = std::async(std::launch::async, [&c]
     {
          std::string s = c.read();
          s += " Welt!!";
          c << s;
     });
     
     f1.wait();
     f2.wait();
     
     /*
      * Say Hello to the world.
      */
     std::cout << c.read();
     
     return 0;
}