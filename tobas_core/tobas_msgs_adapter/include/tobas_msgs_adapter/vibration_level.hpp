#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/vector.hpp>
#include <tobas_msgs/msg/vibration_level.hpp>

namespace tobas_msgs
{
struct VibrationLevel
{
  std_msgs::msg::Header header;
  tobas::kdl::Vector data;

  using SharedPtr = std::shared_ptr<VibrationLevel>;
  using ConstSharedPtr = std::shared_ptr<const VibrationLevel>;
  using UniquePtr = std::unique_ptr<VibrationLevel>;
  using ConstUniquePtr = std::unique_ptr<const VibrationLevel>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::VibrationLevel, tobas_msgs::msg::VibrationLevel>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::VibrationLevel;
  using ros_message_type = tobas_msgs::msg::VibrationLevel;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.data, dst.data);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.data, dst.data);
  }
};

namespace tobas_msgs
{
using VibrationLevelAdapter = rclcpp::TypeAdapter<tobas_msgs::VibrationLevel, tobas_msgs::msg::VibrationLevel>;
}  // namespace tobas_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_msgs::VibrationLevel, tobas_msgs::msg::VibrationLevel);
