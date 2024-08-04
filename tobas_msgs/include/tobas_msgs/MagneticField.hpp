#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_eigen_msgs/Matrix3d.hpp>
#include <tobas_kdl_msgs/Vector.hpp>

#include <tobas_msgs/msg/magnetic_field.hpp>

namespace tobas_msgs
{
struct MagneticField
{
  std_msgs::msg::Header header;
  kdl::Vector magnetic_field;
  Eigen::Matrix3d covariance;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::MagneticField, tobas_msgs::msg::MagneticField>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::MagneticField;
  using ros_message_type = tobas_msgs::msg::MagneticField;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.magnetic_field, dst.magnetic_field);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.covariance, dst.covariance);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.magnetic_field, dst.magnetic_field);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.covariance, dst.covariance);
  }
};

namespace tobas_msgs
{
using MagneticFieldAdapter = rclcpp::TypeAdapter<tobas_msgs::MagneticField, tobas_msgs::msg::MagneticField>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_msgs::MagneticField, tobas_msgs::msg::MagneticField);
