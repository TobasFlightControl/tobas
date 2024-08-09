#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs/Frame.hpp>
#include <tobas_kdl_msgs/Twist.hpp>
#include <tobas_kdl_msgs/Accel.hpp>
#include <tobas_kdl_msgs/Wrench.hpp>

#include <tobas_msgs/msg/link_state.hpp>

namespace tobas_msgs
{
struct LinkState
{
  std::string name;
  kdl::Frame frame;
  kdl::Twist twist;
  kdl::Accel accel;
  kdl::Wrench wrench;

  using SharedPtr = std::shared_ptr<LinkState>;
  using ConstSharedPtr = std::shared_ptr<const LinkState>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::LinkState, tobas_msgs::msg::LinkState>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::LinkState;
  using ros_message_type = tobas_msgs::msg::LinkState;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.name = src.name;
    tobas_kdl_msgs::FrameAdapter::convert_to_ros_message(src.frame, dst.frame);
    tobas_kdl_msgs::TwistAdapter::convert_to_ros_message(src.twist, dst.twist);
    tobas_kdl_msgs::AccelAdapter::convert_to_ros_message(src.accel, dst.accel);
    tobas_kdl_msgs::WrenchAdapter::convert_to_ros_message(src.wrench, dst.wrench);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.name = src.name;
    tobas_kdl_msgs::FrameAdapter::convert_to_custom(src.frame, dst.frame);
    tobas_kdl_msgs::TwistAdapter::convert_to_custom(src.twist, dst.twist);
    tobas_kdl_msgs::AccelAdapter::convert_to_custom(src.accel, dst.accel);
    tobas_kdl_msgs::WrenchAdapter::convert_to_custom(src.wrench, dst.wrench);
  }
};

namespace tobas_msgs
{
using LinkStateAdapter = rclcpp::TypeAdapter<tobas_msgs::LinkState, tobas_msgs::msg::LinkState>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_msgs::LinkState, tobas_msgs::msg::LinkState);
