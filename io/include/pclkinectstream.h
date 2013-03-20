/*
    Copyright (c) 2013 Sebastian Krieger <sebastian-krieger@online.de>

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use,
    copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following
    conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
*/

#include "iostreambase.h"

#ifndef PCLKINECTSTREAM_H
#define PCLKINECTSTREAM_H

#include <pcl/io/openni_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread.hpp>

/// Kinect Streaming.
class PclKinectStream : public IoStreamBase
{
private:
  pcl::visualization::CloudViewer* mViewer;
  pcl::Grabber* mGrabber;
  pcl::PointCloud<pcl::PointXYZRGBA> mCloud;
  boost::mutex mMutex;
  boost::condition_variable_any mCond;
  bool mCloudRequested;
  bool mCloudFetched;
  bool mVisualization;
  
  void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
public:
  /// Constructor
  /**
   * \param visual enable visualization.
   */
  PclKinectStream(bool visual = false);
  
  /// Destructor
  virtual ~PclKinectStream();
  
  /// Initiate kinect device.
  virtual void init();
  
  /// Start streaming.
  virtual void start();
  
  /// Stop streaming.
  virtual void stop();
  
  /// Fetch a point cloud from stream
  /**
   * \return Fetched point cloud.
   */
  pcl::PointCloud< pcl::PointXYZRGBA > get();
};


#endif // PCLKINECTSTREAM_H
