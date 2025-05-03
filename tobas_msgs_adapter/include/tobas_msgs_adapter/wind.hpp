#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_eigen_msgs_adapter/core.hpp>
#include <tobas_kdl_msgs_adapter/vector.hpp>

#include <tobas_msgs/msg/wind.hpp>

namespace tobas_msgs
{
struct Wind
{
  std_msgs::msg::Header header;
  kdl::Vector vel;

  using SharedPtr = std::shared_ptr<Wind>;
  using ConstSharedPtr = std::shared_ptr<const Wind>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::Wind, tobas_msgs::msg::Wind>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::Wind;
  using ros_message_type = tobas_msgs::msg::Wind;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.vel, dst.vel);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.vel, dst.vel);
  }
};

namespace tobas_msgs
{
using WindAdapter = rclcpp::TypeAdapter<tobas_msgs::Wind, tobas_msgs::msg::Wind>;
}  // namespace tobas_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_msgs::Wind, tobas_msgs::msg::Wind);
