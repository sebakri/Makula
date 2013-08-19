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

#include <makula/recognition/pcl/point_cloud_viewer.h>

using namespace makula::recognition;

std::mutex PointCloudViewer::mtx;

PointCloudViewer::PointCloudViewer(const std::string name) :
     viewer ( name )
{

}

PointCloudViewer::~PointCloudViewer()
{
}

void PointCloudViewer::exit()
{
     stopConsuming();
     stopProducing();
     viewer.close();
}

int PointCloudViewer::main()
{
     viewer.setBackgroundColor (0, 0, 0);
     viewer.addCoordinateSystem (0.1);
     viewer.initCameraParameters ();
     viewer.setSize(400, 400);
     
     while ( !viewer.wasStopped()) {
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p;
          if ( takeOn ( p ) ) {
               const auto cp = p;
               deliver(p);
    
               f = std::move(std::async(std::launch::async,[this, &mtx, &cp] {
                    std::unique_lock<std::mutex> lock(mtx);
                    
                    if(!cp)
                         return;
                    
                    if( !viewer.updatePointCloud(cp))
                         viewer.addPointCloud( cp );
                    
                    viewer.spinOnce(100);
               }));
          }
     }

     return 0;
}

