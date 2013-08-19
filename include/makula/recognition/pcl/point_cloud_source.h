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

#ifndef POINTCLOUDSOURCE_H
#define POINTCLOUDSOURCE_H

#include <makula/recognition/data_source_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace makula {
namespace recognition {

class PointCloudSource : public DataSourceBase<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > {
public:

     using Ptr = std::shared_ptr<PointCloudSource>;

     PointCloudSource();
     ~PointCloudSource();

     virtual void exit() override;

protected:
     virtual int main() override;
private:
     void grab ( const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& pc );

     std::atomic_bool quit;
};

}
}

#endif // POINTCLOUDSOURCE_H

