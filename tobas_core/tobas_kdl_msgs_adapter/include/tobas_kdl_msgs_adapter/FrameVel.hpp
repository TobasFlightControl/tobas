#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/framevel.hpp>
#include <tobas_kdl_msgs/msg/frame_vel.hpp>

#include "./Frame.hpp"
#include "./Twist.hpp"

template <>
struct rclcpp::TypeAdapter<kdl::FrameVel, tobas_kdl_msgs::msg::FrameVel>
{
  using is_specialized = std::true_type;
  using custom_type = kdl::FrameVel;
  using ros_message_type = tobas_kdl_msgs::msg::FrameVel;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    tobas_kdl_msgs::FrameAdapter::convert_to_ros_message(src.getFrame(), dst.frame);
    tobas_kdl_msgs::TwistAdapter::convert_to_ros_message(src.getTwist(), dst.twist);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    kdl::Frame frame;
    kdl::Twist twist;
    tobas_kdl_msgs::FrameAdapter::convert_to_custom(src.frame, frame);
    tobas_kdl_msgs::TwistAdapter::convert_to_custom(src.twist, twist);

    dst.setFrame(frame);
    dst.setTwist(twist);
  }
};

namespace tobas_kdl_msgs
{
using FrameVelAdapter = rclcpp::TypeAdapter<kdl::FrameVel, tobas_kdl_msgs::msg::FrameVel>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(kdl::FrameVel, tobas_kdl_msgs::msg::FrameVel);
