#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/accel.hpp>
#include <tobas_kdl_msgs_adapter/frame.hpp>
#include <tobas_kdl_msgs_adapter/twist.hpp>
#include <tobas_msgs/msg/odometry.hpp>

namespace tobas_msgs
{
struct Odometry
{
  tobas::kdl::Frame frame;
  tobas::kdl::Twist twist;
  tobas::kdl::Accel accel;

  using SharedPtr = std::shared_ptr<Odometry>;
  using ConstSharedPtr = std::shared_ptr<const Odometry>;
  using UniquePtr = std::unique_ptr<Odometry>;
  using ConstUniquePtr = std::unique_ptr<const Odometry>;

  void setNaN()
  {
    frame.setNaN();
    twist.setNaN();
    accel.setNaN();
  }
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::Odometry, tobas_msgs::msg::Odometry>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::Odometry;
  using ros_message_type = tobas_msgs::msg::Odometry;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    tobas_kdl_msgs::FrameAdapter::convert_to_ros_message(src.frame, dst.frame);
    tobas_kdl_msgs::TwistAdapter::convert_to_ros_message(src.twist, dst.twist);
    tobas_kdl_msgs::AccelAdapter::convert_to_ros_message(src.accel, dst.accel);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    tobas_kdl_msgs::FrameAdapter::convert_to_custom(src.frame, dst.frame);
    tobas_kdl_msgs::TwistAdapter::convert_to_custom(src.twist, dst.twist);
    tobas_kdl_msgs::AccelAdapter::convert_to_custom(src.accel, dst.accel);
  }
};

namespace tobas_msgs
{
using OdometryAdapter = rclcpp::TypeAdapter<tobas_msgs::Odometry, tobas_msgs::msg::Odometry>;
}  // namespace tobas_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_msgs::Odometry, tobas_msgs::msg::Odometry);
