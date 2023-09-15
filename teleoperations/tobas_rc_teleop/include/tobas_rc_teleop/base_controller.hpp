#pragma once

#include <ros/ros.h>

#include <tobas_msgs/PoseTwist.h>

namespace tobas_rc_teleop
{
class BaseController
{
public:
  explicit BaseController();

  virtual void initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) = 0;
  virtual void reset(const tobas_msgs::PoseTwist& pt) = 0;
};
}  // namespace tobas_rc_teleop
