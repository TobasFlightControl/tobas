#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/rigid_body_inertia.hpp>
#include <tobas_kdl_msgs/msg/rigid_body_inertia.hpp>

#include "./vector.hpp"
#include "./rotational_inertia.hpp"

template <>
struct rclcpp::TypeAdapter<kdl::RigidBodyInertia, tobas_kdl_msgs::msg::RigidBodyInertia>
{
  using is_specialized = std::true_type;
  using custom_type = kdl::RigidBodyInertia;
  using ros_message_type = tobas_kdl_msgs::msg::RigidBodyInertia;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.mass = src.getMass();
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.getCOG(), dst.cog);
    tobas_kdl_msgs::RotationalInertiaAdapter::convert_to_ros_message(src.getRotationalInertiaCoG(), dst.i_cog);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    double mass;
    kdl::Vector cog;
    kdl::RotationalInertia i_cog;

    mass = src.mass;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.cog, cog);
    tobas_kdl_msgs::RotationalInertiaAdapter::convert_to_custom(src.i_cog, i_cog);

    dst = custom_type(mass, cog, i_cog);
  }
};

namespace tobas_kdl_msgs
{
using RigidBodyInertiaAdapter = rclcpp::TypeAdapter<kdl::RigidBodyInertia, tobas_kdl_msgs::msg::RigidBodyInertia>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(kdl::RigidBodyInertia, tobas_kdl_msgs::msg::RigidBodyInertia);
