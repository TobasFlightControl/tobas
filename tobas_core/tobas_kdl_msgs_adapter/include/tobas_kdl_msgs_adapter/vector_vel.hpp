#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/vectorvel.hpp>
#include <tobas_kdl_msgs/msg/vector_vel.hpp>

#include "./vector.hpp"

template <>
struct rclcpp::TypeAdapter<kdl::VectorVel, tobas_kdl_msgs::msg::VectorVel>
{
  using is_specialized = std::true_type;
  using custom_type = kdl::VectorVel;
  using ros_message_type = tobas_kdl_msgs::msg::VectorVel;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.p, dst.p);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.v, dst.v);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.p, dst.p);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.v, dst.v);
  }
};

namespace tobas_kdl_msgs
{
using VectorVelAdapter = rclcpp::TypeAdapter<kdl::VectorVel, tobas_kdl_msgs::msg::VectorVel>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(kdl::VectorVel, tobas_kdl_msgs::msg::VectorVel);
