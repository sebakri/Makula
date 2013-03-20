#include "pclkinectstream.h"

int main()
{
  PclKinectStream grabber(true);
  grabber.init();
  grabber.start();
  for(int i = 0; i < 100; i++)
  {
    grabber.get();
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  }
  grabber.stop();
  
  return 0;
}

