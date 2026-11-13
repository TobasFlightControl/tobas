// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/segment.hpp>

#include <tobas_kdl_msgs/msg/segment.hpp>

#include "./frame.hpp"
#include "./joint.hpp"
#include "./rigid_body_inertia.hpp"

template <>
struct rclcpp::TypeAdapter<tobas::kdl::Segment, tobas_kdl_msgs::msg::Segment>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::kdl::Segment;
  using ros_message_type = tobas_kdl_msgs::msg::Segment;

  static void convert_to_ros_message(const custom_type& _src, ros_message_type& _dst)
  {
    _dst.name = _src.name();
    tobas_kdl_msgs::JointAdapter::convert_to_ros_message(_src.joint(), _dst.joint);
    tobas_kdl_msgs::FrameAdapter::convert_to_ros_message(_src.frame(), _dst.frame);
    tobas_kdl_msgs::RigidBodyInertiaAdapter::convert_to_ros_message(_src.inertia(), _dst.inertia);
  }

  static void convert_to_custom(const ros_message_type& _src, custom_type& _dst)
  {
    std::string name;
    tobas::kdl::Joint joint;
    tobas::kdl::Frame frame;
    tobas::kdl::RigidBodyInertia inertia;

    name = _src.name;
    tobas_kdl_msgs::JointAdapter::convert_to_custom(_src.joint, joint);
    tobas_kdl_msgs::FrameAdapter::convert_to_custom(_src.frame, frame);
    tobas_kdl_msgs::RigidBodyInertiaAdapter::convert_to_custom(_src.inertia, inertia);

    _dst = tobas::kdl::Segment(name, joint, frame, inertia);
  }
};

namespace tobas_kdl_msgs
{
using SegmentAdapter = rclcpp::TypeAdapter<tobas::kdl::Segment, tobas_kdl_msgs::msg::Segment>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::kdl::Segment, tobas_kdl_msgs::msg::Segment);
