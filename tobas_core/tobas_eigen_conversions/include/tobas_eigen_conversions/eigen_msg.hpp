#pragma once

#include <array>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace tf
{
void vectorEigenToMsg(const Eigen::Vector3d& e, geometry_msgs::msg::Vector3& m);
void vectorMsgToEigen(const geometry_msgs::msg::Vector3& m, Eigen::Vector3d& e);

void pointEigenToMsg(const Eigen::Vector3d& e, geometry_msgs::msg::Point& m);
void pointMsgToEigen(const geometry_msgs::msg::Point& m, Eigen::Vector3d& e);

void point32EigenToMsg(const Eigen::Vector3f& e, geometry_msgs::msg::Point32& m);
void point32MsgToEigen(const geometry_msgs::msg::Point32& m, Eigen::Vector3f& e);

void quaternionEigenToMsg(const Eigen::Quaterniond& e, geometry_msgs::msg::Quaternion& m);
void quaternionMsgToEigen(const geometry_msgs::msg::Quaternion& m, Eigen::Quaterniond& e);

void matrix3EigenToMsg(const Eigen::Matrix3d& e, std::array<double, 9>& m);
void matrix3MsgToEigen(const std::array<double, 9>& m, Eigen::Matrix3d& e);
}  // namespace tf
