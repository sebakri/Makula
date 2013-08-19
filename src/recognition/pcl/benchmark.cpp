// Copyright (c) 2013 Sebastian Krieger <sebastian-krieger@online.de>

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <makula/recognition/pcl/point_cloud_source_mock.h>
#include <makula/recognition/pcl/filtering.h>
#include <makula/recognition/pcl/clustering.h>
#include <makula/recognition/pcl/point_cloud_viewer.h>
#include <makula/recognition/pcl/classifying.h>
#include <makula/base/process_connecter.h>
#include <csignal>

using namespace makula::recognition;

Filtering f;
PointCloudSourceMock s("test.pcd");
Clustering c;
Classifying cl ( "./trained_svm_final.txt" );
Localization l ( "./models.txt" );

void interrupt ( int );

int main ( int argc, char** argv )
{
     std::signal ( SIGINT, interrupt );
     std::signal ( SIGKILL, interrupt );
     std::signal ( SIGTERM, interrupt );

     ProcessConnecter::connect ( s, f );
     ProcessConnecter::connect ( f, c );
     ProcessConnecter::connect ( c, cl );
     ProcessConnecter::connect ( cl, l );


     std::cout << " -> Start Kinect..." << std::endl;
     s.execute();

     std::cout << " -> Start Filtering..." << std::endl;
     f.execute();

     std::cout << " -> Start Clustering..." << std::endl;
     c.execute();

     std::cout << " -> Start Classifying..." << std::endl;
     cl.execute();

     std::cout << " -> Start Localization..." << std::endl;
     l.execute();

     return pause();
}

void interrupt ( int signal )
{
     std::cout << std::endl << " <- Interrupt by user" << std::endl;

     l.exit();
     l.wait_until_finished();

     std::cout << " <- Localization finished..." << std::endl;

     cl.exit();
     cl.wait_until_finished();

     std::cout << " <- Classifying finished..." << std::endl;

     c.exit();
     c.wait_until_finished();

     std::cout << " <- Clustering finished..." << std::endl;

     f.exit();
     f.wait_until_finished();

     std::cout << " <- Filtering finished..." << std::endl;

     s.exit();
     s.wait_until_finished();

     std::cout << " <- Kinect finished..." << std::endl;


     int ret = 0;

     ret += c.get() + f.get() + s.get() + cl.get() + l.get();

     std::cout << " <- Returned " << ret << ". BYE BYE" << std::endl;
     exit ( ret );
}
