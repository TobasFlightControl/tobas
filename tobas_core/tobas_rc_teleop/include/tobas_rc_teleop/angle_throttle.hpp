// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_trajectory_generation/online/velocity_limited.hpp>

#include <tobas_command_msgs_adapter/angle_throttle.hpp>

#include "./base_controller.hpp"

namespace tobas
{
namespace rc
{
class AngleThrottleController : public BaseController
{
  using self = AngleThrottleController;
  using super = BaseController;

public:
  explicit AngleThrottleController();

  bool requirePosition() override;
  bool requireVelocity() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(BaseNode* node, FlightMode mode) override;
  void reset(const builtin_interfaces::msg::Time& stamp, const tobas_msgs::Odometry& setpoint, bool landed) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed) override;

private:
  rclcpp::Time t_last_rcin_;
  traj::VelocityLimitedOnlineTrajectoryGenerator roll_filt_, pitch_filt_;
  double tar_yaw_;

  // rosparams
  double max_attitude_;   // [rad]
  double max_head_rate_;  // [rad/s]
  double atti_expo_;
  double head_expo_;
  double throt_expo_;

  // PubSub
  ros2::PublisherPtr<tobas_command_msgs::AngleThrottle> cmd_pub_;

  bool maxAttitudeCb(const double& p);
  bool maxAttitudeRateCb(const double& p);
  bool maxHeadingRateCb(const double& p);
  bool attitudeExpoCb(const double& p);
  bool headingExpoCb(const double& p);
  bool throttleExpoCb(const double& p);
};
}  // namespace rc
}  // namespace tobas
