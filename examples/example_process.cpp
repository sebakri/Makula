#include <makula/base/process.h>
#include <makula/base/process_connecter.h>
#include <makula/base/producer.h>
#include <makula/base/consumer.h>
#include <sstream>
#include <iostream>

using namespace makula::base;

class ProducerProcess : public Process, public Producer<std::string>
{
public:
     ProducerProcess() {}
     virtual ~ProducerProcess() {}
     
protected:
     virtual int main() {
          std::stringstream s;
          
          s << "Hi. My thread id is " << std::this_thread::get_id() << ". And yours?";
          
          if(!deliver(s.str()))
               return -1;
          else
               return 0;
     }
};

class ConsumerProcess : public Process, public Consumer<std::string>
{
public:
     ConsumerProcess() {}
     ~ConsumerProcess() {}
protected:
     virtual int main()
     {
          std::string s;
          std::stringstream ss;
          
          if(!takeOn(s))
               return -1;
          
          ss << s << "\nOh Hi...Mine is " << std::this_thread::get_id() << ".";
          std::cout << ss.str() << std::endl;
          
          return 0;
     }
};

int main(int argc, char** argv)
{
     ProducerProcess p;
     ConsumerProcess c;
     
     ProcessConnecter::connect(p, c);
     
     p.execute();
     c.execute();
     
     std::cout << "Producer returned with " << p.wait_until_finished() << std::endl;
     std::cout << "Consumer returned with " << c.wait_until_finished() << std::endl;
     
     return 0;
}