/*
 * Copyright (c) 2013 Sebastian Krieger <sebastian-krieger@online.de>
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <makula/recognition/pcl/point_cloud_source_mock.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>

using namespace makula::recognition;

PointCloudSourceMock::PointCloudSourceMock(std::string filename) :
     quit ( false ),
     mock (new pcl::PointCloud<pcl::PointXYZRGBA>)
{
  pcl::io::loadPCDFile<pcl::PointXYZRGBA> (filename, *mock);
}

PointCloudSourceMock::~PointCloudSourceMock()
{

}

int PointCloudSourceMock::main()
{
     std::unique_ptr<pcl::Grabber> g ( new pcl::OpenNIGrabber );

     boost::function<void ( const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& ) > f =
          boost::bind ( &PointCloudSourceMock::grab, this, _1 );

     g->registerCallback ( f );

     g->start();

     while ( !quit ) {
          std::this_thread::sleep_for ( std::chrono::milliseconds ( 100 ) );
     }

     g->stop();

     return 0;
}

void PointCloudSourceMock::grab ( const pcl::PointCloud< pcl::PointXYZRGBA >::ConstPtr& pc )
{

     auto p = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ( new pcl::PointCloud<pcl::PointXYZRGBA> ( *pc ) );

     if ( !deliver ( mock ) )
          quit = true;
}

void PointCloudSourceMock::exit()
{
     quit = true;
     stopProducing();
}



