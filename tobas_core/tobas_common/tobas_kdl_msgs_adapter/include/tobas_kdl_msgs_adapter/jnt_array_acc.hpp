// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/jntarray_acc.hpp>

#include <tobas_kdl_msgs/msg/jnt_array_acc.hpp>

#include "./util/util.hpp"

template <>
struct rclcpp::TypeAdapter<tobas::kdl::JntArrayAcc, tobas_kdl_msgs::msg::JntArrayAcc>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::kdl::JntArrayAcc;
  using ros_message_type = tobas_kdl_msgs::msg::JntArrayAcc;

  static void convert_to_ros_message(const custom_type& _src, ros_message_type& _dst)
  {
    tobas_kdl_msgs::vectorXdEigenToStd(_src.q.data, _dst.q);
    tobas_kdl_msgs::vectorXdEigenToStd(_src.qdot.data, _dst.qdot);
    tobas_kdl_msgs::vectorXdEigenToStd(_src.qdotdot.data, _dst.qdotdot);
  }

  static void convert_to_custom(const ros_message_type& _src, custom_type& _dst)
  {
    tobas_kdl_msgs::vectorXdStdToEigen(_src.q, _dst.q.data);
    tobas_kdl_msgs::vectorXdStdToEigen(_src.qdot, _dst.qdot.data);
    tobas_kdl_msgs::vectorXdStdToEigen(_src.qdotdot, _dst.qdotdot.data);
  }
};

namespace tobas_kdl_msgs
{
using JntArrayAccAdapter = rclcpp::TypeAdapter<tobas::kdl::JntArrayAcc, tobas_kdl_msgs::msg::JntArrayAcc>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::kdl::JntArrayAcc, tobas_kdl_msgs::msg::JntArrayAcc);
