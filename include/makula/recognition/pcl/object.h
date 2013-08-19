/*
 * object.h
 *
 *  Created on: 07.03.2012
 *      Author: Aaron Klein
 */

#ifndef OBJECT_H_
#define OBJECT_H_

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/cvfh.h>
#include "pcl/features/fpfh.h"
#include <pcl/features/vfh.h>
#include "pcl/keypoints/sift_keypoint.h"
#include <pcl/features/spin_image.h>


namespace makula { namespace recognition {

typedef pcl::PointXYZRGBA PointT;

class Object {
public:
     Object ( const std::string& filename, unsigned int label )
          :m_name ( filename ),m_label ( label ) {
          loadCloud();
          computeSurfaceNormals();
          computeVFHFeatures();
     }
     Object ( pcl::PointCloud<PointT>::Ptr& cloud )
          :m_cloud ( cloud ) {
          computeSurfaceNormals();
          computeVFHFeatures();
     }

     Object ( const Object&& o )
          : m_cloud ( std::move ( o.m_cloud ) ),
            m_features ( std::move ( o.m_features ) ),
            m_normals ( std::move ( o.m_normals ) ) {

     }
     
     Object ( pcl::PointCloud<PointT>::Ptr& cloud, const std::string& filename, const unsigned label = 0 )
          : m_name ( filename ), m_cloud ( cloud ), m_label ( label ) {
          computeSurfaceNormals();
          computeVFHFeatures();
     }

     pcl::PointCloud<PointT>::Ptr getCloud() const {
          return m_cloud;
     }

     pcl::PointCloud<pcl::VFHSignature308>::Ptr getFeatures() const {
          return m_features;
     }

     std::string getName() const {
          return m_name;
     }

     pcl::PointCloud<pcl::Normal>::Ptr getNormals() const {
          return m_normals;
     }

     const unsigned getLabel() const {
          return m_label;
     }

private:
     void computeVFHFeatures();
     void computeSurfaceNormals();
     void loadCloud ();

     std::string m_name;
     pcl::PointCloud <PointT>::Ptr m_cloud;
     pcl::PointCloud<pcl::VFHSignature308>::Ptr m_features;
     pcl::PointCloud<pcl::Normal>::Ptr m_normals;

     unsigned int m_label;
};

}}

#endif /* OBJECT_H_ */
