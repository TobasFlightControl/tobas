#pragma once

#include <rclcpp/rclcpp.hpp>


#include <tobas_msgs/msg/joint_command_array.hpp>

class TobasBridge
{
  using self = TobasBridge;

public:
  explicit TobasBridge();

private:

  rclcpp::Node::SharedPtr pnh_;

  ros2::PublisherPtr<> js_pub_;
  ros2::SubscriberPtr<> tar_pos_sub_;
  ros2::SubscriberPtr<> tar_vel_sub_;
  ros2::SubscriberPtr<> tar_eff_sub_;

  ros2::TimerPtr main_timer_;

  void jntPosCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& tar_pos);
  void jntVelCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& tar_vel);
  void jntEffCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& tar_eff);

  void mainTimerCb();
};
