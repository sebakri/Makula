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

#ifndef FILTERPCL_H
#define FILTERPCL_H

#include <makula/recognition/filtering_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace makula {
namespace recognition {

class Filtering : public FilteringBase<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> {
public:
     using Ptr = std::shared_ptr<Filtering>;

     Filtering();
     ~Filtering();

     virtual void exit() override;
protected:
     virtual int main() override;

private:
     std::atomic_bool quit;
     Channel<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> workerChannel;

     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downsample ( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pc, float leaf_size );
     pcl::PointCloud< pcl::PointXYZRGBA >::Ptr passthrough ( pcl::PointCloud< pcl::PointXYZRGBA >::Ptr input, float min_depth_x, float max_depth_x, float min_depth_y, float max_depth_y, float min_depth_z, float max_depth_z );
};

}
}

#endif // FILTERPCL_H
