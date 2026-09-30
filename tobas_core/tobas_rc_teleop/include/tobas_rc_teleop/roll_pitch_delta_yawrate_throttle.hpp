// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_trajectory_generation/online/velocity_limited.hpp>

#include <tobas_command_msgs/msg/roll_pitch_delta_yawrate_throttle.hpp>

#include "./base_controller.hpp"

namespace tobas
{
namespace rc
{
class RollPitchDeltaYawrateThrottleController : public BaseController
{
  using self = RollPitchDeltaYawrateThrottleController;
  using super = BaseController;

public:
  explicit RollPitchDeltaYawrateThrottleController();

  bool requireHorizontalPosition() override;
  bool requireVerticalPosition() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(BaseNode* node, FlightMode mode) override;
  void reset(const builtin_interfaces::msg::Time& stamp, const tobas_msgs::Odometry& setpoint, bool landed) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed) override;

private:
  rclcpp::Time t_last_rcin_;
  traj::VelocityLimitedOnlineTrajectoryGenerator roll_filt_, pitch_filt_;
  double tar_yaw_;

  // ROS parameters.
  double max_roll_;       // [rad]
  double max_pitch_;      // [rad]
  double max_head_rate_;  // [rad/s]
  double atti_expo_;
  double head_expo_;
  double throt_expo_;

  // PubSub
  ros2::PublisherPtr<tobas_command_msgs::msg::RollPitchDeltaYawrateThrottle> cmd_pub_;

  bool maxRollCb(const double& p);
  bool maxRollRateCb(const double& p);
  bool maxPitchCb(const double& p);
  bool maxPitchRateCb(const double& p);
  bool maxHeadingRateCb(const double& p);
  bool attitudeExpoCb(const double& p);
  bool headingExpoCb(const double& p);
  bool throttleExpoCb(const double& p);
};
}  // namespace rc
}  // namespace tobas
