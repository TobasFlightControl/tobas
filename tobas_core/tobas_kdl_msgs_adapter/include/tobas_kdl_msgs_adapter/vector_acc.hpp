#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/vector_acc.hpp>
#include <tobas_kdl_msgs/msg/vector_acc.hpp>

#include "./vector.hpp"

template <>
struct rclcpp::TypeAdapter<kdl::VectorAcc, tobas_kdl_msgs::msg::VectorAcc>
{
  using is_specialized = std::true_type;
  using custom_type = kdl::VectorAcc;
  using ros_message_type = tobas_kdl_msgs::msg::VectorAcc;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.p, dst.p);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.v, dst.v);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.dv, dst.dv);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.p, dst.p);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.v, dst.v);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.dv, dst.dv);
  }
};

namespace tobas_kdl_msgs
{
using VectorAccAdapter = rclcpp::TypeAdapter<kdl::VectorAcc, tobas_kdl_msgs::msg::VectorAcc>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(kdl::VectorAcc, tobas_kdl_msgs::msg::VectorAcc);
