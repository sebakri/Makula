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

#include <makula/recognition/pcl/clustering.h>
#include <makula/base/worker_pool.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace makula::base;
using namespace makula::recognition;

Clustering::Clustering() :
     quit ( false )
{

}

Clustering::~Clustering()
{

}

int Clustering::main()
{
     auto chSegmentation = Channel<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>();
     auto chClustering = Channel<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>();
     auto segmentationWorkers = WorkerPool<> ( [this, &chSegmentation, &chClustering] () {
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
          chSegmentation >> cloud;
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segmentedCloud = segmentate ( cloud );
          chClustering << segmentedCloud;
     } );
     auto clusteringWorkers = WorkerPool<> ( [this, &chClustering] () {
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segmentedCloud;
          chClustering >> segmentedCloud;
          
          auto clusters = clustering ( segmentedCloud );

//           std::cout << " -> Found " << clusters.size() << " clusters..." << std::endl;

          deliver ( clusters );
     });

     while ( !quit ) {
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p;
          if ( takeOn ( p ) ) {
               chSegmentation << p;
               segmentationWorkers.startWorker();
               clusteringWorkers.startWorker();
          } else
               quit = true;
     }

     segmentationWorkers.wait();
     clusteringWorkers.wait();

     return 0;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Clustering::segmentate ( const pcl::PointCloud< pcl::PointXYZRGBA >::Ptr& input )
{
     pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
     pcl::PointIndices::Ptr inliers ( new pcl::PointIndices );
     pcl::ModelCoefficients::Ptr coefficients ( new pcl::ModelCoefficients );
     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_segmentated ( new pcl::PointCloud<pcl::PointXYZRGBA> () );
     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZRGBA> () );

     cloud_segmentated = input;

     seg.setOptimizeCoefficients ( true );
     seg.setModelType ( pcl::SACMODEL_PLANE );
     seg.setMethodType ( pcl::SAC_RANSAC );
     seg.setMaxIterations ( 100 );
     seg.setDistanceThreshold ( 0.01 );

     seg.setInputCloud ( cloud_segmentated );
     seg.segment ( *inliers, *coefficients );

     pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
     extract.setInputCloud ( cloud_segmentated );
     extract.setIndices ( inliers );
     extract.setNegative ( true );
     extract.filter ( *cloud_segmentated );

     pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
     sor.setInputCloud ( cloud_segmentated );
     sor.setMeanK ( 50 );
     sor.setStddevMulThresh ( 1.0 );
     sor.filter ( *cloud_segmentated );

     return cloud_segmentated;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > Clustering::clustering ( const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& input )
{
     std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > clusters;
     pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree ( new pcl::search::KdTree<pcl::PointXYZRGBA> );
     tree->setInputCloud ( input );

     std::vector<pcl::PointIndices> cluster_indices;
     pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
     ec.setClusterTolerance ( 0.008 );
     ec.setMinClusterSize ( 500 );
     ec.setMaxClusterSize ( 25000 );
     ec.setSearchMethod ( tree );
     ec.setInputCloud ( input );
     ec.extract ( cluster_indices );

     for ( std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it ) {
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster ( new pcl::PointCloud<pcl::PointXYZRGBA> );
          for ( std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++ )
               cloud_cluster->points.push_back ( input->points[*pit] ); //*
          cloud_cluster->width = cloud_cluster->points.size ();
          cloud_cluster->height = 1;
          cloud_cluster->is_dense = true;

          clusters.push_back ( cloud_cluster );
     }

     return std::move ( clusters );
}

void Clustering::exit()
{
     quit = true;
     stopConsuming();
     stopProducing();
}
