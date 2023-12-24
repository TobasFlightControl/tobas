#pragma once

#include <ros/ros.h>

#include <tobas_std_tools/range.hpp>

#include <tobas_msgs/Odometry.h>
#include <tobas_msgs/RCInput.h>

namespace tobas_rc_teleop
{
class BaseController
{
public:
  explicit BaseController();

  virtual void initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh) = 0;
  virtual void reset(const tobas_msgs::Odometry& odom) = 0;
  virtual void update(
    const tobas_msgs::RCInput& rcin,
    const tobas_msgs::Odometry& odom,
    const double& battery_voltage,
    const tobas_std::Range<double>& dead_zone) = 0;

protected:
};
}  // namespace tobas_rc_teleop
