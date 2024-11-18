#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/Vector.hpp>

#include <tobas_msgs/msg/magnetic_field_raw.hpp>

namespace tobas_msgs
{
struct MagneticFieldRaw
{
  std_msgs::msg::Header header;
  kdl::Vector magnetic_field;

  using SharedPtr = std::shared_ptr<MagneticFieldRaw>;
  using ConstSharedPtr = std::shared_ptr<const MagneticFieldRaw>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::MagneticFieldRaw, tobas_msgs::msg::MagneticFieldRaw>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::MagneticFieldRaw;
  using ros_message_type = tobas_msgs::msg::MagneticFieldRaw;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.magnetic_field, dst.magnetic_field);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.magnetic_field, dst.magnetic_field);
  }
};

namespace tobas_msgs
{
using MagneticFieldRawAdapter = rclcpp::TypeAdapter<tobas_msgs::MagneticFieldRaw, tobas_msgs::msg::MagneticFieldRaw>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_msgs::MagneticFieldRaw, tobas_msgs::msg::MagneticFieldRaw);
