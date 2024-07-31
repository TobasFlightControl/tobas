#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/rotationalinertia.hpp>
#include <tobas_kdl_msgs/msg/rotational_inertia.hpp>

#include "./util/util.hpp"

template <>
struct rclcpp::TypeAdapter<kdl::RotationalInertia, tobas_kdl_msgs::msg::RotationalInertia>
{
  using is_specialized = std::true_type;
  using custom_type = kdl::RotationalInertia;
  using ros_message_type = tobas_kdl_msgs::msg::RotationalInertia;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    tobas_kdl_msgs::matrix3dEigenToStd(src.data, dst.data);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    tobas_kdl_msgs::matrix3dStdToEigen(src.data, dst.data);
  }
};

namespace tobas_kdl_msgs
{
using RotationalInertia = rclcpp::TypeAdapter<kdl::RotationalInertia, tobas_kdl_msgs::msg::RotationalInertia>;
}
