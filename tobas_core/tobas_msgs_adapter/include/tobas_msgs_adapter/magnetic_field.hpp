#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/vector.hpp>
#include <tobas_msgs/msg/magnetic_field.hpp>

namespace tobas_msgs
{
struct MagneticField
{
  std_msgs::msg::Header header;
  kdl::Vector mag;

  using SharedPtr = std::shared_ptr<MagneticField>;
  using ConstSharedPtr = std::shared_ptr<const MagneticField>;
  using UniquePtr = std::unique_ptr<MagneticField>;
  using ConstUniquePtr = std::unique_ptr<const MagneticField>;
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
using MagneticFieldAdapter = rclcpp::TypeAdapter<tobas_msgs::MagneticField, tobas_msgs::msg::MagneticField>;
}  // namespace tobas_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_msgs::MagneticField, tobas_msgs::msg::MagneticField);
