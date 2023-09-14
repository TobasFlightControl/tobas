#pragma once

#include <ros/ros.h>

namespace tobas_rc_teleop
{
class BaseController
{
public:
  explicit BaseController();

  virtual void initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) = 0;
};
}  // namespace tobas_rc_teleop
