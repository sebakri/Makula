/*
 * object.cpp
 *
 *  Created on: 14.03.2012
 *      Author: Aaron Klein
 */

#include <makula/recognition/pcl/object.h>


using namespace makula::recognition;

void Object::computeVFHFeatures()
{
     pcl::VFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> vfh_est;
     pcl::search::KdTree<PointT>::Ptr tree ( new pcl::search::KdTree<PointT> () );
     m_features = pcl::PointCloud<pcl::VFHSignature308>::Ptr ( new pcl::PointCloud<pcl::VFHSignature308> () );

     vfh_est.setInputCloud ( m_cloud );
     vfh_est.setInputNormals ( m_normals );
     vfh_est.setSearchMethod ( tree );
     vfh_est.compute ( *m_features );
}

void Object::computeSurfaceNormals()
{
     m_normals = pcl::PointCloud<pcl::Normal>::Ptr ( new pcl::PointCloud<pcl::Normal> );
     pcl::search::KdTree<PointT>::Ptr tree ( new pcl::search::KdTree<PointT> () );
     pcl::NormalEstimation<PointT, pcl::Normal> norm_est;

     norm_est.setInputCloud ( m_cloud );
     norm_est.setSearchMethod ( tree );
     norm_est.setRadiusSearch ( 0.02 );

     norm_est.compute ( *m_normals );
}

void Object::loadCloud ()
{
     m_cloud = pcl::PointCloud<PointT>::Ptr ( new pcl::PointCloud<PointT> );
     pcl::io::loadPCDFile ( m_name, *m_cloud );
}
