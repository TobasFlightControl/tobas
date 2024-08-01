#pragma once

#include <rclcpp/rclcpp.hpp>
#include <ros/timer.h>

#include <tobas_msgs/JointCommandArray.h>

class TobasBridge
{
  using self = TobasBridge;

public:
  explicit TobasBridge();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr pnh_;

  rclcpp::Publisher js_pub_;
  rclcpp::Subscriber tar_pos_sub_;
  rclcpp::Subscriber tar_vel_sub_;
  rclcpp::Subscriber tar_eff_sub_;

  rclcpp::Timer main_timer_;

  void jntPosCmdCb(const tobas_msgs::JointCommandArrayConstPtr& tar_pos);
  void jntVelCmdCb(const tobas_msgs::JointCommandArrayConstPtr& tar_vel);
  void jntEffCmdCb(const tobas_msgs::JointCommandArrayConstPtr& tar_eff);

  void mainTimerCb(const rclcpp::TimerEvent& event);
};
