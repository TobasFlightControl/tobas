#include "../include/tobas_ros2_tools/eigen_conversion.hpp"

namespace ros2
{
void vectorEigenToMsg(const Eigen::Vector3d& e, geometry_msgs::msg::Vector3& m)
{
  m.x = e.x();
  m.y = e.y();
  m.z = e.z();
}

void vectorMsgToEigen(const geometry_msgs::msg::Vector3& m, Eigen::Vector3d& e)
{
  e.x() = m.x;
  e.y() = m.y;
  e.z() = m.z;
}

void quaternionEigenToMsg(const Eigen::Quaterniond& e, geometry_msgs::msg::Quaternion& m)
{
  m.w = e.w();
  m.x = e.x();
  m.y = e.y();
  m.z = e.z();
}

void quaternionMsgToEigen(const geometry_msgs::msg::Quaternion& m, Eigen::Quaterniond& e)
{
  e.w() = m.w;
  e.x() = m.x;
  e.y() = m.y;
  e.z() = m.z;
}

void matrix3EigenToMsg(const Eigen::Matrix3d& e, std::array<double, 9>& m)
{
  std::memcpy(m.data(), e.data(), sizeof(double) * 9);
}

void matrix3MsgToEigen(const std::array<double, 9>& m, Eigen::Matrix3d& e)
{
  std::memcpy(e.data(), m.data(), sizeof(double) * 9);
}
}  // namespace ros2
