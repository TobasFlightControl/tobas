#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/vector.hpp>

#include <tobas_msgs/msg/magnetic_field_stamped.hpp>

namespace tobas_msgs
{
struct MagneticFieldStamped
{
  std_msgs::msg::Header header;
  kdl::Vector mag;

  using SharedPtr = std::shared_ptr<MagneticFieldStamped>;
  using ConstSharedPtr = std::shared_ptr<const MagneticFieldStamped>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::MagneticFieldStamped, tobas_msgs::msg::MagneticFieldStamped>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::MagneticFieldStamped;
  using ros_message_type = tobas_msgs::msg::MagneticFieldStamped;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.mag, dst.mag);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.mag, dst.mag);
  }
};

namespace tobas_msgs
{
using MagneticFieldStampedAdapter =
  rclcpp::TypeAdapter<tobas_msgs::MagneticFieldStamped, tobas_msgs::msg::MagneticFieldStamped>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_msgs::MagneticFieldStamped, tobas_msgs::msg::MagneticFieldStamped);
