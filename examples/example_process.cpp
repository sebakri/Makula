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
          
          s << "My thread id is " << std::this_thread::get_id() << ". And yours?";
          
          if(!deliver(s.str()))
               return -1;
          else
               return 0;
     }
     
     virtual void exit() {}
};

class ConsumerProcess : public Process, public Consumer<std::string>
{
public:
     ConsumerProcess() {}
     virtual ~ConsumerProcess() {}
protected:
     virtual int main()
     {
          std::string s;
          std::stringstream ss;
          
          if(!takeOn(s))
               return -1;
          
          ss << s << "\nMine is " << std::this_thread::get_id() << ".";
          std::cout << ss.str() << std::endl;
          
          return 0;
     }
     
     virtual void exit() {}
};

class PipeProcess : public Process, public Producer<std::string>, public Consumer<std::string>
{
public:
     PipeProcess() {}
     virtual ~PipeProcess() {}
protected:
     virtual int main() {
          std::string s;
          std::stringstream ss;
          
          if(!takeOn(s))
               return -1;
          
          ss << s << "\nMine is " << std::this_thread::get_id() << ". And yours?";
          
          if(!deliver(ss.str()))
               return -2;
          else
               return 0;
          
          
          return 0;
     }
     
     virtual void exit() {}
};

int main(int argc, char** argv)
{
     ProducerProcess p;
     PipeProcess pp;
     ConsumerProcess c;
     
     ProcessConnecter::connect(p, pp);
     ProcessConnecter::connect(pp, c);
     
     p.execute();
     pp.execute();
     c.execute();
     
     p.wait_until_finished();
     pp.wait_until_finished();
     c.wait_until_finished();
     
     return p.get() + pp.get() + c.get();
}