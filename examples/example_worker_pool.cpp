#include <makula/base/worker_pool.h>
#include <makula/base/channel.h>
#include <iostream>

int long_task(int i) {
     std::this_thread::sleep_for(std::chrono::seconds(1));
     return i;
}

using namespace makula::base;

int main(int argc, char** argv) {
     
     auto f1 = std::async(std::launch::async, [] {
          std::chrono::time_point<std::chrono::system_clock> start, end;
          start = std::chrono::system_clock::now();
          
          int sum = 0;
          for(int i = 0; i < std::thread::hardware_concurrency(); i++) {
               sum += long_task(i);
          }
          
          end = std::chrono::system_clock::now();
          int elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>
                             (end-start).count();
          std::cout << "Without WorkerPool: " << sum;
          std::cout << " (elapsed time: " << elapsed_seconds << "s)" << std::endl;
     });
     
     auto f2 = std::async(std::launch::async, [] {
          std::chrono::time_point<std::chrono::system_clock> start, end;
          start = std::chrono::system_clock::now();
          
          auto ch = Channel<int>(std::thread::hardware_concurrency());
     
          auto w = WorkerPool<int>([&ch] (int value) {
               ch << long_task(value);
          });
          
          int sum = 0;
          for(int i = 0; i < std::thread::hardware_concurrency(); i++) {
               w.startWorker(i);
          }
          
          for(int i = 0; i < std::thread::hardware_concurrency(); i++) {
               int v;
               ch >> v;
               sum += v;
          }
          end = std::chrono::system_clock::now();
          int elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>
                             (end-start).count();
          std::cout << "With WorkerPool: " << sum;
          std::cout << " (elapsed time: " << elapsed_seconds << "s)" << std::endl;
     });
     
     f1.wait();
     f2.wait();
     
     return 0;
}