#include <makula/base/process.h>
#include <iostream>
#include <sstream>

using namespace makula::base;

class ExampleProcess : public Process
{
public:
     ExampleProcess() {}
     virtual ~ExampleProcess() {}
     
protected:
     virtual int main() {
          std::stringstream s;
          
          std::cout << "My thread id is " << std::this_thread::get_id();
          
          return 0;
     }
     
     virtual void exit() {}
};

int main(int argc, char** argv)
{
     ExampleProcess p;
     
     p.execute();
     
     p.wait_until_finished();
     
     return 0;
}