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


#include "pclkinectstream.h"

PclKinectStream::PclKinectStream(bool visual) :
mGrabber(new pcl::OpenNIGrabber),
mCloudRequested(false),
mCloudFetched(false),
mVisualization(visual)
{
  if(visual)
    mViewer = new pcl::visualization::CloudViewer("Kinect Streaming");
}

PclKinectStream::~PclKinectStream()
{
  delete mGrabber;
  delete mViewer;
}

void PclKinectStream::init()
{
  boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind(&PclKinectStream::cloud_cb_, this, _1);
  mGrabber->registerCallback(f);
}

void PclKinectStream::start()
{
  mGrabber->start();
}

void PclKinectStream::stop()
{
  mGrabber->stop();
}

pcl::PointCloud<pcl::PointXYZRGBA> PclKinectStream::get()
{
  mCloudRequested = true;
  boost::unique_lock<boost::mutex> lock(mMutex);
  
  while(!mCloudFetched)
    mCond.wait(mMutex);
  
  mCloudFetched = false;
  return mCloud;
}

void PclKinectStream::cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
  if(!mCloudRequested) {
    boost::this_thread::sleep(boost::posix_time::milliseconds(200));
    return;
  }
  
  if(mVisualization && !mViewer->wasStopped())
  {
    mViewer->showCloud(cloud);
  }
  
  mCloud = *cloud;
  mCloudFetched = true;
  mCloudRequested = false;
  mCond.notify_all();
  
}