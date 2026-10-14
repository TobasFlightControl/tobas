// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_trajectory_generation/online/velocity_limited.hpp>

#include <tobas_command_msgs_adapter/accel_pitch_yaw.hpp>

#include "./base_controller.hpp"

namespace tobas::rc
{
class AccelPitchYawController : public BaseController
{
  using self = AccelPitchYawController;

public:
  explicit AccelPitchYawController();

  bool requireHorizontalPosition() override;
  bool requireVerticalPosition() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(BaseNode* _node, FlightMode _mode) override;
  void reset(const builtin_interfaces::msg::Time& _stamp, const tobas_msgs::Odometry& _setpoint, bool _landed) override;
  void update(const tobas_msgs::RCInput& _rcin, const tobas_msgs::Odometry& _odom, bool _landed) override;

private:
  rclcpp::Time t_last_rcin_;
  traj::VelocityLimitedOnlineTrajectoryGenerator ax_filt_, ay_filt_, pitch_filt_;
  double tar_yaw_;

  // ROS parameters
  double max_hor_acc_;   ///< [m/s]
  double max_ver_acc_;   ///< [m/s]
  double max_pitch_;     ///< [rad]
  double max_yaw_rate_;  ///< [rad/s]
  double hor_acc_expo_;
  double ver_acc_expo_;
  double pitch_expo_;
  double yaw_expo_;

  // Publisher
  ros2::PublisherPtr<tobas_command_msgs::AccelPitchYaw> cmd_pub_;

  void maxHorizontalAccelCb(double _p);
  void maxHorizontalJerkCb(double _p);
  void maxVerticalAccelCb(double _p);
  void maxPitchCb(double _p);
  void maxPitchRateCb(double _p);
  void maxYawRateCb(double _p);
  void horizontalAccelExpoCb(double _p);
  void verticalAccelExpoCb(double _p);
  void pitchExpoCb(double _p);
  void yawExpoCb(double _p);
};
}  // namespace tobas::rc
