// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_trajectory_generation/online/velocity_limited.hpp>

#include <tobas_command_msgs/msg/roll_pitch_side_slip_throttle.hpp>

#include "./base_controller.hpp"

namespace tobas
{
namespace rc
{
class RollPitchSideSlipThrottleController : public BaseController
{
  using self = RollPitchSideSlipThrottleController;
  using super = BaseController;

public:
  explicit RollPitchSideSlipThrottleController();

  bool requireHorizontalPosition() override;
  bool requireVerticalPosition() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(BaseNode* node, FlightMode mode) override;
  void reset(const builtin_interfaces::msg::Time& stamp, const tobas_msgs::Odometry& setpoint, bool landed) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed) override;

private:
  rclcpp::Time t_last_rcin_;
  traj::VelocityLimitedOnlineTrajectoryGenerator roll_filt_, pitch_filt_, side_slip_filt_;

  // ROS parameters.
  double max_roll_;       // [rad]
  double max_pitch_;      // [rad]
  double max_side_slip_;  // [rad]
  double atti_expo_;
  double head_expo_;
  double throt_expo_;

  // PubSub
  ros2::PublisherPtr<tobas_command_msgs::msg::RollPitchSideSlipThrottle> cmd_pub_;

  bool maxRollCb(const double& p);
  bool maxRollRateCb(const double& p);
  bool maxPitchCb(const double& p);
  bool maxPitchRateCb(const double& p);
  bool maxSideSlipCb(const double& p);
  bool maxSideSlipRateCb(const double& p);
  bool attitudeExpoCb(const double& p);
  bool headingExpoCb(const double& p);
  bool throttleExpoCb(const double& p);
};
}  // namespace rc
}  // namespace tobas
