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

#ifndef CLUSTERING_H
#define CLUSTERING_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <makula/recognition/clustering_base.h>

namespace makula {
namespace recognition {

class Clustering :  public makula::recognition::ClusteringBase<
     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
     std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> > {
public:
     Clustering();
     ~Clustering();

     void exit();
protected:
     virtual int main();

private:
     std::atomic_bool quit;
     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segmentate ( const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& input );
     std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > clustering ( const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& input );
};

}
}

#endif // CLUSTERING_H
