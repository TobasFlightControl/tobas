#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/jntarray_vel.hpp>
#include <tobas_kdl_msgs/msg/jnt_array_vel.hpp>

#include "./util/util.hpp"

template <>
struct rclcpp::TypeAdapter<kdl::JntArrayVel, tobas_kdl_msgs::msg::JntArrayVel>
{
  using is_specialized = std::true_type;
  using custom_type = kdl::JntArrayVel;
  using ros_message_type = tobas_kdl_msgs::msg::JntArrayVel;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    tobas_kdl_msgs::vectorXdEigenToStd(src.q.data, dst.q);
    tobas_kdl_msgs::vectorXdEigenToStd(src.qdot.data, dst.qdot);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    tobas_kdl_msgs::vectorXdStdToEigen(src.q, dst.q.data);
    tobas_kdl_msgs::vectorXdStdToEigen(src.qdot, dst.qdot.data);
  }
};

namespace tobas_kdl_msgs
{
using JntArrayVelAdapter = rclcpp::TypeAdapter<kdl::JntArrayVel, tobas_kdl_msgs::msg::JntArrayVel>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(kdl::JntArrayVel, tobas_kdl_msgs::msg::JntArrayVel);
