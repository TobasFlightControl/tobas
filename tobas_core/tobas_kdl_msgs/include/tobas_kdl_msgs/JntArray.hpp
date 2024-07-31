#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/jntarray.hpp>
#include <tobas_kdl_msgs/msg/jnt_array.hpp>

#include "./util/util.hpp"

template <>
struct rclcpp::TypeAdapter<kdl::JntArray, tobas_kdl_msgs::msg::JntArray>
{
  using is_specialized = std::true_type;
  using custom_type = kdl::JntArray;
  using ros_message_type = tobas_kdl_msgs::msg::JntArray;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    tobas_kdl_msgs::vectorXdEigenToStd(src.data, dst.data);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    tobas_kdl_msgs::vectorXdStdToEigen(src.data, dst.data);
  }
};

namespace tobas_kdl_msgs
{
using JntArray = rclcpp::TypeAdapter<kdl::JntArray, tobas_kdl_msgs::msg::JntArray>;
}
