#ifndef __TYPES_H__
#define __TYPES_H__
#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>
#include <vector>

namespace Planning3D {
// Eigen
typedef Eigen::MatrixXd MatXX;
typedef Eigen::VectorXd VecX;

typedef Eigen::Matrix3d Mat3d;
typedef Eigen::Matrix3f Mat3f;

typedef Eigen::Vector4f Vec4f;
typedef Eigen::Vector4d Vec4d;
typedef Eigen::Vector3f Vec3f;
typedef Eigen::Vector3d Vec3d;

typedef Eigen::Quaternionf Quaternion;

// pcl
typedef pcl::PointXYZ PointXYZ;
typedef pcl::Normal Normal;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::Normal> PointCloudNormal;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudPN;
// typedef pcl::

// ros
typedef sensor_msgs::PointCloud2 PointCloud2;

// stl
typedef std::string string;
} // namespace Planning3D

#endif // __TYPES_H__