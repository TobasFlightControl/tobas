#pragma once

#include <ros/ros.h>

#include <dh_std_tools/range.hpp>

#include <tobas_msgs/PoseTwist.h>
#include <tobas_msgs/RCInput.h>

namespace tobas_rc_teleop
{
class BaseController
{
public:
  explicit BaseController();

  virtual void initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) = 0;
  virtual void reset(const tobas_msgs::PoseTwist& pt) = 0;
  virtual void update(
    const tobas_msgs::RCInput& rcin,
    const tobas_msgs::PoseTwist& pt,
    const double& battery_voltage,
    const dh_std::Range<double>& dead_zone) = 0;

protected:
};
}  // namespace tobas_rc_teleop
