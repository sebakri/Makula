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

#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <makula/recognition/pcl/object.h>
#include <makula/recognition/localization_base.h>

namespace makula {
namespace recognition {
     
using namespace makula::base;

/**
 * \brief This type provides information of classification process.
 */
struct LocalizationType {
     pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster;
     int best_match;
};

/**
 * \brief Localization process.
 */
class Localization : public LocalizationBase<LocalizationType> {
public:
     Localization( const std::string file );
     virtual ~Localization();
     
     /**
      * \brief Implementation of exit-method.
      */
     virtual void exit();
protected:
     
     /**
      * \brief Implementation of main-method.
      */
     virtual int main();
private:
     std::atomic_uint transformationCounter;
     std::atomic_bool quit;
     std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> models;
     
     void localisation(pcl::PointCloud<PointT>::Ptr& source,const unsigned best_match);
     pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFPFHFeatures (pcl::PointCloud<PointT>::Ptr& cloud);
     Eigen::Matrix4f prealign (const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &targetFeatures,
                                        const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &sourceFeatures ,
                                        const pcl::PointCloud<PointT>::Ptr &targetCloud,
                                        const pcl::PointCloud<PointT>::Ptr &sourceCloud, pcl::PointCloud<PointT>::Ptr &output);
     Eigen::Matrix4f registrate(const pcl::PointCloud<PointT>::Ptr &target,
                                         const pcl::PointCloud<PointT>::Ptr &source);
};

}
}

#endif // LOCALIZATION_H