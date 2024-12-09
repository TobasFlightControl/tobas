#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/rigid_body_inertia.hpp>
#include <tobas_kdl_msgs/msg/rigid_body_inertia_stamped.hpp>

#include "./rigid_body_inertia.hpp"

namespace tobas_kdl_msgs
{
struct RigidBodyInertiaStamped
{
  std_msgs::msg::Header header;
  kdl::RigidBodyInertia inertia;

  using SharedPtr = std::shared_ptr<RigidBodyInertiaStamped>;
  using ConstSharedPtr = std::shared_ptr<const RigidBodyInertiaStamped>;
};
}  // namespace tobas_kdl_msgs

template <>
struct rclcpp::TypeAdapter<tobas_kdl_msgs::RigidBodyInertiaStamped, tobas_kdl_msgs::msg::RigidBodyInertiaStamped>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_kdl_msgs::RigidBodyInertiaStamped;
  using ros_message_type = tobas_kdl_msgs::msg::RigidBodyInertiaStamped;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::RigidBodyInertiaAdapter::convert_to_ros_message(src.inertia, dst.inertia);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::RigidBodyInertiaAdapter::convert_to_custom(src.inertia, dst.inertia);
  }
};

namespace tobas_kdl_msgs
{
using RigidBodyInertiaStampedAdapter =
  rclcpp::TypeAdapter<tobas_kdl_msgs::RigidBodyInertiaStamped, tobas_kdl_msgs::msg::RigidBodyInertiaStamped>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  tobas_kdl_msgs::RigidBodyInertiaStamped,
  tobas_kdl_msgs::msg::RigidBodyInertiaStamped);
