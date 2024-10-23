#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/jntarrayacc.hpp>
#include <tobas_kdl_msgs/msg/jnt_array_acc.hpp>

#include "./util/util.hpp"

template <>
struct rclcpp::TypeAdapter<kdl::JntArrayAcc, tobas_kdl_msgs::msg::JntArrayAcc>
{
  using is_specialized = std::true_type;
  using custom_type = kdl::JntArrayAcc;
  using ros_message_type = tobas_kdl_msgs::msg::JntArrayAcc;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    tobas_kdl_msgs::vectorXdEigenToStd(src.q.data, dst.q);
    tobas_kdl_msgs::vectorXdEigenToStd(src.qdot.data, dst.qdot);
    tobas_kdl_msgs::vectorXdEigenToStd(src.qdotdot.data, dst.qdotdot);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    tobas_kdl_msgs::vectorXdStdToEigen(src.q, dst.q.data);
    tobas_kdl_msgs::vectorXdStdToEigen(src.qdot, dst.qdot.data);
    tobas_kdl_msgs::vectorXdStdToEigen(src.qdotdot, dst.qdotdot.data);
  }
};

namespace tobas_kdl_msgs
{
using JntArrayAccAdapter = rclcpp::TypeAdapter<kdl::JntArrayAcc, tobas_kdl_msgs::msg::JntArrayAcc>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(kdl::JntArrayAcc, tobas_kdl_msgs::msg::JntArrayAcc);
