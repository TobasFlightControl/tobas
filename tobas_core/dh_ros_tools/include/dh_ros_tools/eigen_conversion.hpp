#pragma once

#include <boost/array.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

namespace dh_ros
{
void vectorEigenToMsg(const Eigen::Vector3d& e, geometry_msgs::Vector3& m);
void vectorMsgToEigen(const geometry_msgs::Vector3& m, Eigen::Vector3d& e);

void quaternionEigenToMsg(const Eigen::Quaterniond& e, geometry_msgs::Quaternion& m);
void quaternionMsgToEigen(const geometry_msgs::Quaternion& m, Eigen::Quaterniond& e);

void matrix3EigenToMsg(const Eigen::Matrix3d& e, boost::array<double, 9>& m);
void matrix3MsgToEigen(const boost::array<double, 9>& m, Eigen::Matrix3d& e);
}  // namespace dh_ros
