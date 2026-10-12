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

public:
  explicit AngleThrottleController();

  bool requireHorizontalPosition() override;
  bool requireVerticalPosition() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(BaseNode* _node, FlightMode _mode) override;
  void reset(const builtin_interfaces::msg::Time& _stamp, const tobas_msgs::Odometry& _setpoint, bool _landed) override;
  void update(const tobas_msgs::RCInput& _rcin, const tobas_msgs::Odometry& _odom, bool _landed) override;

private:
  rclcpp::Time t_last_rcin_;
  traj::VelocityLimitedOnlineTrajectoryGenerator roll_filt_, pitch_filt_;
  double tar_yaw_;

  // ROS parameters
  double max_attitude_;   ///< [rad]
  double max_head_rate_;  ///< [rad/s]
  double atti_expo_;
  double head_expo_;
  double throt_expo_;

  // PubSub
  ros2::PublisherPtr<tobas_command_msgs::AngleThrottle> cmd_pub_;

  void maxAttitudeCb(double _p);
  void maxAttitudeRateCb(double _p);
  void maxHeadingRateCb(double _p);
  void attitudeExpoCb(double _p);
  void headingExpoCb(double _p);
  void throttleExpoCb(double _p);
};
}  // namespace rc
}  // namespace tobas
