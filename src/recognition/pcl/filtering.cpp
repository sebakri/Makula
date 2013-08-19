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

#include <makula/recognition/pcl/filtering.h>
#include <makula/base/worker_pool.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

using namespace makula::recognition;

Filtering::Filtering()

{

}

Filtering::~Filtering()
{

}

void Filtering::exit()
{
     stopConsuming();
     stopProducing();
     quit = true;
}

int Filtering::main()
{
     auto chPassthrough = Channel<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>();
     auto chDownsample = Channel<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>();
     auto downsampleWorkers = WorkerPool<float> (
     [this, &chDownsample] ( float leaf_size ) {
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input;
          chDownsample >> input;
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downsampledCloud = downsample ( input, leaf_size );
          deliver ( downsampledCloud );
     });

     auto passthroughWorkers = WorkerPool<float, float, float, float, float, float> (
     [this, &chPassthrough, &chDownsample] ( float min_depth_x, float max_depth_x, float min_depth_y, float max_depth_y, float min_depth_z, float max_depth_z ) {
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input;
          chPassthrough >> input;
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr passthroughedCloud = passthrough ( input, min_depth_x, max_depth_x, min_depth_y, max_depth_y, min_depth_z, max_depth_z );
          chDownsample << passthroughedCloud;
     });

     while ( !quit ) {
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cp;

          if ( takeOn ( cp ) ) {
               chPassthrough << cp;
               passthroughWorkers.startWorker ( -2.0, 2.0, 0.0, 2.0, 0.0, 1.5 );
               downsampleWorkers.startWorker ( 0.001f );
          } else
               quit = true;
     }
     

     passthroughWorkers.wait();
     downsampleWorkers.wait();

     return 0;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Filtering::downsample ( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pc, float leaf_size )
{
     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downsampled ( new pcl::PointCloud<pcl::PointXYZRGBA> );
     pcl::VoxelGrid<pcl::PointXYZRGBA> voxel_grid;
     voxel_grid.setInputCloud ( pc );
     voxel_grid.setLeafSize ( leaf_size, leaf_size, leaf_size );
     voxel_grid.filter ( *downsampled );

     return std::move ( downsampled );
}


pcl::PointCloud< pcl::PointXYZRGBA >::Ptr Filtering::passthrough ( pcl::PointCloud< pcl::PointXYZRGBA >::Ptr input, float min_depth_x, float max_depth_x, float min_depth_y, float max_depth_y, float min_depth_z, float max_depth_z )
{
     auto filtered_x = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ( new pcl::PointCloud<pcl::PointXYZRGBA> );
     auto filtered_y = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ( new pcl::PointCloud<pcl::PointXYZRGBA> );
     auto filtered_z = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ( new pcl::PointCloud<pcl::PointXYZRGBA> );

     pcl::PassThrough<pcl::PointXYZRGBA > pass_z;
     pass_z.setInputCloud ( input );
     pass_z.setFilterFieldName ( "z" );
     pass_z.setFilterLimits ( min_depth_z, max_depth_z );
     pass_z.filter ( *filtered_z );

     pcl::PassThrough<pcl::PointXYZRGBA> pass_y;
     pass_y.setInputCloud ( filtered_z );
     pass_y.setFilterFieldName ( "y" );
     pass_y.setFilterLimits ( min_depth_y, max_depth_y );
     pass_y.filter ( *filtered_y );

     pcl::PassThrough<pcl::PointXYZRGBA> pass_x;
     pass_x.setInputCloud ( filtered_y );
     pass_x.setFilterFieldName ( "x" );
     pass_x.setFilterLimits ( min_depth_x, max_depth_x );
     pass_x.filter ( *filtered_x );

     return filtered_x;
}
