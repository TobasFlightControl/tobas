// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/rigid_body_inertia.hpp>

#include <tobas_kdl_msgs/msg/rigid_body_inertia.hpp>

#include "./rotational_inertia.hpp"
#include "./vector.hpp"

template <>
struct rclcpp::TypeAdapter<tobas::kdl::RigidBodyInertia, tobas_kdl_msgs::msg::RigidBodyInertia>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::kdl::RigidBodyInertia;
  using ros_message_type = tobas_kdl_msgs::msg::RigidBodyInertia;

  static void convert_to_ros_message(const custom_type& _src, ros_message_type& _dst)
  {
    _dst.mass = _src.getMass();
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(_src.getCOG(), _dst.cog);
    tobas_kdl_msgs::RotationalInertiaAdapter::convert_to_ros_message(_src.getRotationalInertiaCoG(), _dst.i_cog);
  }

  static void convert_to_custom(const ros_message_type& _src, custom_type& _dst)
  {
    double mass;
    tobas::kdl::Vector cog;
    tobas::kdl::RotationalInertia i_cog;

    mass = _src.mass;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(_src.cog, cog);
    tobas_kdl_msgs::RotationalInertiaAdapter::convert_to_custom(_src.i_cog, i_cog);

    _dst = custom_type(mass, cog, i_cog);
  }
};

namespace tobas_kdl_msgs
{
using RigidBodyInertiaAdapter =
  rclcpp::TypeAdapter<tobas::kdl::RigidBodyInertia, tobas_kdl_msgs::msg::RigidBodyInertia>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::kdl::RigidBodyInertia, tobas_kdl_msgs::msg::RigidBodyInertia);
