#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/rotational_inertia.hpp>

#include <tobas_kdl_msgs/msg/rotational_inertia_stamped.hpp>

#include "./rotational_inertia.hpp"

namespace tobas_kdl_msgs
{
struct RotationalInertiaStamped
{
  std_msgs::msg::Header header;
  kdl::RotationalInertia inertia;

  using SharedPtr = std::shared_ptr<RotationalInertiaStamped>;
  using ConstSharedPtr = std::shared_ptr<const RotationalInertiaStamped>;
};
}  // namespace tobas_kdl_msgs

template <>
struct rclcpp::TypeAdapter<tobas_kdl_msgs::RotationalInertiaStamped, tobas_kdl_msgs::msg::RotationalInertiaStamped>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_kdl_msgs::RotationalInertiaStamped;
  using ros_message_type = tobas_kdl_msgs::msg::RotationalInertiaStamped;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::RotationalInertiaAdapter::convert_to_ros_message(src.inertia, dst.inertia);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::RotationalInertiaAdapter::convert_to_custom(src.inertia, dst.inertia);
  }
};

namespace tobas_kdl_msgs
{
using RotationalInertiaStampedAdapter =
  rclcpp::TypeAdapter<tobas_kdl_msgs::RotationalInertiaStamped, tobas_kdl_msgs::msg::RotationalInertiaStamped>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  tobas_kdl_msgs::RotationalInertiaStamped,
  tobas_kdl_msgs::msg::RotationalInertiaStamped);
