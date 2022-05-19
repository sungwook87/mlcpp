#ifndef UTILITY_H
#define UTILITY_H

///// ROS
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

///// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>>

///// Eigen, Linear Algebra
#include <Eigen/Eigen> //whole Eigen library

using namespace std;
using namespace Eigen;


////////////////////////// utils
////////// PCL
template <typename T>
sensor_msgs::PointCloud2 cloud2msg(const pcl::PointCloud<T> &cloud, string frame_id = "map")
{
  sensor_msgs::PointCloud2 cloud_ROS;
  pcl::toROSMsg(cloud, cloud_ROS);
  cloud_ROS.header.frame_id = frame_id;
  return cloud_ROS;
}
pcl::PointCloud<pcl::PointXYZ> cloudmsg2cloud(const sensor_msgs::PointCloud2 &cloudmsg)
{
  pcl::PointCloud<pcl::PointXYZ> cloudresult;
  pcl::fromROSMsg(cloudmsg,cloudresult);
  return cloudresult;
}


///////// Transform
Quaterniond normal_vector_to_quaternion_double(const double &normal_x, const double &normal_y, const double &normal_z){
  Vector3d u1, u2, u3;
  u1 << normal_x, normal_y, normal_z;
  u1 = u1.normalized();
  if ((fabs(u1(0)) > 0.001) || (fabs(u1(1)) > 0.001)) {
      u2 << -u1(1), u1(0), 0;
  } else {
      u2 << 0, u1(2), -u1(1);
  }
  u2.normalize();
  u3 = u1.cross(u2);
  Matrix3d R;
  R.col(0) = u1;
  R.col(1) = u2;
  R.col(2) = u3;
  Quaterniond q(R);
  return q;
}
geometry_msgs::PoseArray pclnormal_to_posearray(const pcl::PointCloud<pcl::PointNormal> &pcl_normals)
{
  geometry_msgs::PoseArray normal_pose_array;
  for (int i = 0; i < pcl_normals.points.size(); ++i)
  {
    geometry_msgs::Pose pose;
    pose.position.x = pcl_normals.points[i].x;
    pose.position.y = pcl_normals.points[i].y;
    pose.position.z = pcl_normals.points[i].z;

    Quaterniond q = normal_vector_to_quaternion_double(pcl_normals.points[i].normal[0], pcl_normals.points[i].normal[1], pcl_normals.points[i].normal[2]);

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    normal_pose_array.poses.push_back(pose);
  }
  return normal_pose_array;
}
geometry_msgs::PoseStamped single_pclnormal_to_posestamped(const pcl::PointNormal &pcl_normals)
{
  geometry_msgs::PoseStamped pcl_normal_to_posestamped;

  pcl_normal_to_posestamped.pose.position.x = pcl_normals.x;
  pcl_normal_to_posestamped.pose.position.y = pcl_normals.y;
  pcl_normal_to_posestamped.pose.position.z = pcl_normals.z;

  Quaterniond q = normal_vector_to_quaternion_double(pcl_normals.normal[0], pcl_normals.normal[1], pcl_normals.normal[2]);

  pcl_normal_to_posestamped.pose.orientation.x = q.x();
  pcl_normal_to_posestamped.pose.orientation.y = q.y();
  pcl_normal_to_posestamped.pose.orientation.z = q.z();
  pcl_normal_to_posestamped.pose.orientation.w = q.w();

  return pcl_normal_to_posestamped;
}


#endif