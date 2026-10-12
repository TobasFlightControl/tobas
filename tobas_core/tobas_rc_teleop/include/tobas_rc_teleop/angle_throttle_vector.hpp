// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_trajectory_generation/online/velocity_limited.hpp>

#include <tobas_command_msgs_adapter/angle_throttle_vector.hpp>

#include "./base_controller.hpp"

namespace tobas
{
namespace rc
{
class AngleThrottleVectorController : public BaseController
{
  using self = AngleThrottleVectorController;

public:
  explicit AngleThrottleVectorController();

  bool requireHorizontalPosition() override;
  bool requireVerticalPosition() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(BaseNode* _node, FlightMode _mode) override;
  void reset(const builtin_interfaces::msg::Time& _stamp, const tobas_msgs::Odometry& _setpoint, bool _landed) override;
  void update(const tobas_msgs::RCInput& _rcin, const tobas_msgs::Odometry& _odom, bool _landed) override;

private:
  rclcpp::Time t_last_rcin_;
  traj::VelocityLimitedOnlineTrajectoryGenerator roll_filt_, pitch_filt_, thrust_angle_filt_;
  double tar_yaw_;

  // ROS parameters
  double max_roll_;          ///< [rad]
  double max_pitch_;         ///< [rad]
  double max_yaw_rate_;      ///< [rad/s]
  double max_thrust_angle_;  ///< [rad]
  double roll_expo_;
  double yaw_expo_;
  double throt_expo_;
  double thrust_angle_expo_;

  // PubSub
  ros2::PublisherPtr<tobas_command_msgs::AngleThrottleVector> cmd_pub_;

  void maxRollCb(double _p);
  void maxRollRateCb(double _p);
  void maxPitchCb(double _p);
  void maxPitchRateCb(double _p);
  void maxYawRateCb(double _p);
  void maxThrustAngleCb(double _p);
  void maxThrustAngleRateCb(double _p);
  void rollExpoCb(double _p);
  void yawExpoCb(double _p);
  void throttleExpoCb(double _p);
  void thrustAngleExpoCb(double _p);
};
}  // namespace rc
}  // namespace tobas
