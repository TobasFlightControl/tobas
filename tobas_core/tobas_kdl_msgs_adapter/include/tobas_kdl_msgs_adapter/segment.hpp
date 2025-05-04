#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/segment.hpp>

#include <tobas_kdl_msgs/msg/segment.hpp>

#include "./joint.hpp"
#include "./frame.hpp"
#include "./rigid_body_inertia.hpp"

template <>
struct rclcpp::TypeAdapter<kdl::Segment, tobas_kdl_msgs::msg::Segment>
{
  using is_specialized = std::true_type;
  using custom_type = kdl::Segment;
  using ros_message_type = tobas_kdl_msgs::msg::Segment;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.name = src.name();
    tobas_kdl_msgs::JointAdapter::convert_to_ros_message(src.joint(), dst.joint);
    tobas_kdl_msgs::FrameAdapter::convert_to_ros_message(src.frame(), dst.frame);
    tobas_kdl_msgs::RigidBodyInertiaAdapter::convert_to_ros_message(src.inertia(), dst.inertia);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    std::string name;
    kdl::Joint joint;
    kdl::Frame frame;
    kdl::RigidBodyInertia inertia;

    name = src.name;
    tobas_kdl_msgs::JointAdapter::convert_to_custom(src.joint, joint);
    tobas_kdl_msgs::FrameAdapter::convert_to_custom(src.frame, frame);
    tobas_kdl_msgs::RigidBodyInertiaAdapter::convert_to_custom(src.inertia, inertia);

    dst = kdl::Segment(name, joint, frame, inertia);
  }
};

namespace tobas_kdl_msgs
{
using SegmentAdapter = rclcpp::TypeAdapter<kdl::Segment, tobas_kdl_msgs::msg::Segment>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(kdl::Segment, tobas_kdl_msgs::msg::Segment);
