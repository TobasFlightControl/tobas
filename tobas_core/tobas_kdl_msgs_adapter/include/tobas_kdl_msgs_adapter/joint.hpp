#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/joint.hpp>

#include <tobas_kdl_msgs/msg/joint.hpp>

#include "./vector.hpp"

template <>
struct rclcpp::TypeAdapter<kdl::Joint, tobas_kdl_msgs::msg::Joint>
{
  using is_specialized = std::true_type;
  using custom_type = kdl::Joint;
  using ros_message_type = tobas_kdl_msgs::msg::Joint;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.name = src.name;
    dst.type = static_cast<uint8_t>(src.type);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.origin, dst.origin);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.axis(), dst.axis);
    dst.damping = src.damping;
    dst.friction = src.friction;
    dst.lower_limit = src.lower_limit;
    dst.upper_limit = src.upper_limit;
    dst.max_effort = src.max_effort;
    dst.max_velocity = src.max_velocity;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.name = src.name;
    dst.type = static_cast<kdl::Joint::joint_type_t>(src.type);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.origin, dst.origin);

    kdl::Vector axis;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.axis, axis);
    dst.axis(axis);

    dst.damping = src.damping;
    dst.friction = src.friction;
    dst.lower_limit = src.lower_limit;
    dst.upper_limit = src.upper_limit;
    dst.max_effort = src.max_effort;
    dst.max_velocity = src.max_velocity;
  }
};

namespace tobas_kdl_msgs
{
using JointAdapter = rclcpp::TypeAdapter<kdl::Joint, tobas_kdl_msgs::msg::Joint>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(kdl::Joint, tobas_kdl_msgs::msg::Joint);
