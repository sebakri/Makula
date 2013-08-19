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

#include <makula/recognition/pcl/point_cloud_source.h>
#include <pcl/io/openni_grabber.h>

using namespace makula::recognition;

PointCloudSource::PointCloudSource() :
     quit ( false )
{

}

PointCloudSource::~PointCloudSource()
{

}

int PointCloudSource::main()
{
     std::unique_ptr<pcl::Grabber> g ( new pcl::OpenNIGrabber );

     boost::function<void ( const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& ) > f =
          boost::bind ( &PointCloudSource::grab, this, _1 );

     g->registerCallback ( f );

     g->start();

     while ( !quit ) {
          std::this_thread::sleep_for ( std::chrono::milliseconds ( 100 ) );
     }

     g->stop();

     return 0;
}

void PointCloudSource::grab ( const pcl::PointCloud< pcl::PointXYZRGBA >::ConstPtr& pc )
{

     auto p = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ( new pcl::PointCloud<pcl::PointXYZRGBA> ( *pc ) );

     if ( !deliver ( p ) )
          quit = true;
}

void PointCloudSource::exit()
{
     quit = true;
     stopProducing();
}



