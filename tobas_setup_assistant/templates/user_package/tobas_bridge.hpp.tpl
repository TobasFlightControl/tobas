#pragma once

#include <ros/ros.h>
#include <ros/timer.h>

#include <tobas_msgs/JointCommandArray.h>

class TobasBridge
{
  using self = TobasBridge;

public:
  explicit TobasBridge(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher js_pub_;
  ros::Subscriber tar_pos_sub_;
  ros::Subscriber tar_vel_sub_;
  ros::Subscriber tar_eff_sub_;

  ros::Timer main_timer_;

  void jntPosCmdCb(const tobas_msgs::JointCommandArrayConstPtr& tar_pos);
  void jntVelCmdCb(const tobas_msgs::JointCommandArrayConstPtr& tar_vel);
  void jntEffCmdCb(const tobas_msgs::JointCommandArrayConstPtr& tar_eff);

  void mainTimerCb(const ros::TimerEvent& event);
};
