#include <makula/base/channel.h>
#include <future>
#include <iostream>

using namespace makula::base;

int main(int argc, char** argv)
{
     /*
      * Create a new string-channel...
      */
     Channel<std::string> c(5);
     
     /*
      * Start a task sending data to the channel...
      */
     auto f1 = std::async(std::launch::async, [&c]
     {
          for(int i = 0; i < 10; i++) {
               c << "Hello";
          }
     });
     
     /*
      * ...and a task reading data and manipulate it.
      */
     auto f2 = std::async(std::launch::async, [&c]
     {
          for(int i = 0; i < 10; i++) {
               std::string s;
               c >> s;
               s += " Welt!!";
               std::cout << s;
          }
     });
     
     f1.wait();
     f2.wait();
     
     return 0;
}